#pragma once

#include <string_view>

#include <boost/asio.hpp>
#include "UdpMulticast.h"

namespace asio = boost::asio;
namespace ip   = asio::ip;

class Server
{
public:
    Server(std::string_view Addr, std::string_view Port, std::string_view UnicastPort);

    // server multicast
    std::size_t SendData(auto&& Buffer) { return m_Multicast.SendTo(std::forward<decltype(Buffer)>(Buffer)); }

    std::size_t SendData(auto&& Buffer, std::size_t Size)
    {
        return m_Multicast.SendTo(std::forward<decltype(Buffer)>(Buffer), Size);
    }

    ~Server();

private:
    void StartReceiving();
    void ReceiveClientUpdate();

private:
    static constexpr std::size_t BufferSize = 1024;
    std::array<std::byte, BufferSize> m_ReceiveBuffer;
    ip::udp::endpoint m_RemoteEndpoint;

    // unicast receive
    asio::io_context m_io_ctx;
    ip::udp::socket m_UnicastSocket;
    std::thread m_MessageHandler; // this needs to come after socket creation

    // multicast send
    UdpMulticast m_Multicast;
};
