#pragma once

#include <string_view>
#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>

namespace asio = boost::asio;
namespace ip   = asio::ip;

// udp multicast
class UdpMulticast
{
public:
    UdpMulticast(std::string_view Addr, std::string_view Port, boost::asio::io_context& io_ctx, bool Host) :
        m_Host(Host), m_Socket(io_ctx, ip::udp::v4())
    {
        const auto MulticastAddress = boost::asio::ip::make_address(Addr);
        const auto MulticastPort    = boost::lexical_cast<std::uint16_t>(Port);

        m_Socket.set_option(boost::asio::ip::udp::socket::reuse_address(true));
        m_Socket.set_option(boost::asio::ip::multicast::enable_loopback(true));

        if (!m_Host) { m_Socket.bind(boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), MulticastPort)); }

        m_Socket.set_option(boost::asio::ip::multicast::join_group(MulticastAddress));
        m_Endpoint = {MulticastAddress, MulticastPort};
    }

    // Send
    std::size_t SendTo(auto&&... Buffer)
        requires requires { boost::asio::buffer(std::forward<decltype(Buffer)>(Buffer)...); }
    {
        return m_Socket.send_to(boost::asio::buffer(std::forward<decltype(Buffer)>(Buffer)...), m_Endpoint);
    }

    // Synchronous receive
    std::size_t Receive(asio::mutable_buffer Buffer,
                        ip::udp::endpoint& Sender,
                        asio::socket_base::message_flags Flags = 0,
                        boost::system::error_code* Error       = nullptr)
    {
        return Error ? m_Socket.receive_from(Buffer, Sender, Flags, *Error)
                     : m_Socket.receive_from(Buffer, Sender, Flags);
    }

    // Asynchronous receive (explicit)
    template <typename ReadHandler>
    auto AsyncReceive(asio::mutable_buffer Buffer, ip::udp::endpoint& Sender, ReadHandler&& Handler)
    {
        return m_Socket.async_receive_from(Buffer, Sender, std::forward<ReadHandler>(Handler));
    }

    // Cancel all asynchronous operations
    auto Cancel() { return m_Socket.cancel(); }

private:
    void
        CreateSocket(boost::asio::io_context& io_ctx, const ip::address& MulticastAddress, std::uint16_t MulticastPort);

private:
    bool m_Host;
    boost::asio::ip::udp::socket m_Socket;
    boost::asio::ip::udp::endpoint m_Endpoint;
};
