#include "Server.h"

#include <iostream>
#include <functional>
#include <array>
#include <string>

#include <boost/lexical_cast.hpp>

// https://stackoverflow.com/questions/12708558/c-multiple-multicast-receiver-with-boost-asio

Server::Server(std::string_view Addr, std::string_view Port, std::string_view UnicastPort) :
    m_UnicastSocket(
        ip::udp::socket(m_io_ctx, ip::udp::endpoint(ip::udp::v4(), boost::lexical_cast<std::uint16_t>(UnicastPort)))),
    m_Multicast(Addr, Port, m_io_ctx, true)
{
    m_MessageHandler = std::thread(std::bind(&Server::StartReceiving, this));
}

Server::~Server()
{
    m_UnicastSocket.cancel();
    if (m_MessageHandler.joinable()) { m_MessageHandler.join(); }
}

void Server::StartReceiving()
{
    ReceiveClientUpdate();
    m_io_ctx.run();
}

void Server::ReceiveClientUpdate()
{
    m_UnicastSocket.async_receive_from(asio::buffer(m_ReceiveBuffer),
                                       m_RemoteEndpoint,
                                       [&](const boost::system::error_code& Error, std::size_t Length)
                                       {
                                           if (Error == boost::asio::error::operation_aborted) { return; }

                                           if (Length == 0) { return ReceiveClientUpdate(); }

                                           // Construct string safely from received length
                                           std::string Rsp {reinterpret_cast<const char*>(m_ReceiveBuffer.data()),
                                                            Length};
                                           std::cout << Rsp << '\n';

                                           // run forever
                                           return ReceiveClientUpdate();
                                       });
}
