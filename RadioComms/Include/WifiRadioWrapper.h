#pragma once
#include "Transport/TransferManager.h"
#include <span>
#include <vector>

namespace ra::turtleford
{

class WifiRadioSession : public ITransferSession,
      public std::enable_shared_from_this<WifiRadioSession>
{
public:
    using ReceiveCallback = std::function<void(const TransferContext& Context, std::span<const std::byte> Data)>;
    virtual bool Send(std::span<const std::byte> Data);
    virtual void RegisterCallback(ReceiveCallback Cb);

    virtual bool IsOpen();
    virtual size_t ID() const;
    virtual TransportType Transport() const;
    virtual const ITransferConfig& Config() const;

    void SendToCallbacks(std::span<const std::byte> Data);

    ~WifiRadioSession();
    WifiRadioSession(std::function<bool(std::span<const std::byte>)> sendFunc, size_t id);

private:
    friend std::shared_ptr<ITransferSession> std::make_shared<ITransferSession>();
    std::vector<ReceiveCallback> callbacks;
    std::function<bool(std::span<const std::byte>)> wrapperSendFunc;
    size_t id;
};
    
class WifiRadioWrapper : public ITransferManager
{
private:
    /* data */
    std::shared_ptr<ITransferSession> session;
    std::weak_ptr<WifiRadioWrapper> m_Manager;
    size_t m_SessionId {};
    std::vector<std::shared_ptr<WifiRadioSession>> m_Sessions {}; // Active sessions

    // std::unique_ptr< ITelemetryRadio> m_Radio;
    // Next session id to assign (monotonic)
    size_t m_NextSessionId {1};

    bool Send(std::span<const std::byte> Data);

public:
    // wifi stuff
    int sock;
    bool connected = false;

    // Interface requirements
    virtual bool Init();
    virtual bool Deinit();

    virtual TransportType Transport() const;

    virtual std::shared_ptr<ITransferSession> CreateSession(std::unique_ptr<ITransferConfig> cfg);
    virtual bool Close(const ITransferSession&);
    virtual std::shared_ptr<ITransferSession> GetSession(size_t id) const;
    virtual void Process();

    WifiRadioWrapper(/* args */);
    ~WifiRadioWrapper();
};

} // namespace ra::turtleford

