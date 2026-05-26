#include "WifiRadioWrapper.h"
#include "Transport/TransferSession.h"
#include "Transport/LoraTransport.h"

#include <iostream>
#include <unistd.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <span>

#define PORT 3333

int set_nonblocking(int fd) {
    return fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
}

namespace ra::turtleford
{
    
WifiRadioWrapper::WifiRadioWrapper(/* args */)
{
}

WifiRadioWrapper::~WifiRadioWrapper(){ }

bool WifiRadioWrapper::Init(){
    const char* server_ip = "192.168.4.1";

    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("socket");
        return -1;
    }

    set_nonblocking(sock);

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(PORT);
    inet_pton(AF_INET, server_ip, &addr.sin_addr);

    int res = connect(sock, (sockaddr*)&addr, sizeof(addr));
    if (res < 0 && errno != EINPROGRESS) {
        perror("connect");
        return -1;
    }

    std::cout << "Connecting...\n";
    return true;
}
bool WifiRadioWrapper::Deinit(){
    close(sock);
    return true;
}

TransportType WifiRadioWrapper::Transport() const{
    return TransportType::Lora;
}

std::shared_ptr<ITransferSession> WifiRadioWrapper::CreateSession(std::unique_ptr<ITransferConfig> cfg){
    if (!cfg.get()) { return nullptr; }

    std::unique_ptr<LoraTransferConfig> LoraConfigPtr(static_cast<LoraTransferConfig*>(cfg.release()));
    auto* pSession = new WifiRadioSession([this](std::span<const std::byte> data) {
        return this->Send(data);
    }, m_NextSessionId++);
    if (!pSession) { return nullptr; }

    std::shared_ptr<WifiRadioSession> Session {pSession};
    // ManagedSession owns the per-session state (config + callback)
    m_Sessions.push_back(Session);

    return Session;
}
bool WifiRadioWrapper::Close(const ITransferSession&){
    return true;
}
std::shared_ptr<ITransferSession> WifiRadioWrapper::GetSession(size_t id) const{
    for (auto &&i : m_Sessions)
    {
        if(i->ID() == id){
            return i;
        }
    }
    return nullptr;

}
void WifiRadioWrapper::Process(){
    struct pollfd fds[2];
    fds[0].fd = sock;
    fds[0].events = POLLIN | POLLOUT;

    fds[1].fd = STDIN_FILENO;
    fds[1].events = POLLIN;

    char buffer[1024];
    int ret = poll(fds, 2, 10);

    if (ret < 0) {
        perror("poll");
        return;
    }

    // Socket events
    if (fds[0].revents & POLLOUT && !connected) {
        int err;
        socklen_t len = sizeof(err);
        getsockopt(sock, SOL_SOCKET, SO_ERROR, &err, &len);
        if (err == 0) {
            std::cout << "Connected to ESP32\n";
            connected = true;
        } else {
            std::cerr << "Connection failed\n";
            return;
        }
    }

    if (fds[0].revents & POLLIN) {
        int bytes = read(sock, buffer, sizeof(buffer) - 1);
        if (bytes > 0) {
            buffer[bytes] = '\0';
            std::cout << "Received: " << buffer;
        } else {
            std::cout << "Disconnected\n";
            return;
        }

        for (auto &&i : m_Sessions)
        {
            i->Send(std::span<std::byte>(reinterpret_cast<std::byte*>(buffer),bytes));
        }
        
    }
}

bool WifiRadioWrapper::Send(std::span<const std::byte> Data) 
{
    struct pollfd fds[2];
    fds[0].fd = sock;
    fds[0].events = POLLIN | POLLOUT;

    fds[1].fd = STDIN_FILENO;
    fds[1].events = POLLIN;

    // char buffer[1024];
    int ret = poll(fds, 2, 10);

    if (ret < 0) {
        perror("poll");
        return false;
    }

    // Socket events
    if (fds[0].revents & POLLOUT && !connected) {
        int err;
        socklen_t len = sizeof(err);
        getsockopt(sock, SOL_SOCKET, SO_ERROR, &err, &len);
        if (err == 0) {
            std::cout << "Connected to ESP32\n";
            connected = true;
        } else {
            std::cerr << "Connection failed\n";
            return false;
        }
        
    }
        // User input
    if (fds[1].revents & POLLIN) {
        send(sock,Data.data(), Data.size(), 0);
    }
    return true; 
}

bool WifiRadioSession::Send(std::span<const std::byte> Data){
    return wrapperSendFunc(Data);
}

void WifiRadioSession::RegisterCallback(ReceiveCallback Cb) {
    callbacks.push_back(Cb);
}

bool WifiRadioSession::IsOpen() { return true; }

size_t WifiRadioSession::ID() const { return id; }

TransportType WifiRadioSession::Transport() const { return TransportType::BLE; }

const ITransferConfig& WifiRadioSession::Config() const
{
    // TODO: insert return statement here
    return LoraTransferConfig(0,0);
}

WifiRadioSession::~WifiRadioSession() {}

WifiRadioSession::WifiRadioSession(std::function<bool(std::span<const std::byte>)> sendFunc, size_t id_in) 
{
    wrapperSendFunc = sendFunc;
    id = id_in;
}

void WifiRadioSession::SendToCallbacks(std::span<const std::byte> Data){
    for (auto &&i : callbacks)
    {
        i(
        TransferContext {.SessionID = ID(),
                        .Type      = TransportType::Lora,
                        .SourceID  = 0,
                        .Session   = shared_from_this()},
        Data);
    }
    
}

} // namespace ra::turtleford