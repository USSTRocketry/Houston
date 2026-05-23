#include <iostream>
#include <thread>
#include <chrono>
#include <array>
#include <cmath>

#include <boost/asio.hpp>

// Turtleford includes
#include "ProtoCodec.h"
#include "Type.h"

namespace asio = boost::asio;
namespace ip = asio::ip;
using namespace ra::turtleford;

int main() {
    try {
        asio::io_context io_ctx;
        ip::udp::endpoint endpoint(ip::make_address("239.255.0.1"), 33333);
        ip::udp::socket socket(io_ctx, endpoint.protocol());
        
        socket.set_option(ip::udp::socket::reuse_address(true));
        socket.set_option(ip::multicast::enable_loopback(true));

        std::cout << "Starting Mock C++ Telemetry Sender (UDP Multicast 239.255.0.1:33333)\n";

        uint32_t timestamp = 0;
        float time_sec = 0.0f;

        while (true) {
            ra::type::FlightData data{};
            
            // Generate mock data
            data.BMP_Data.altitude = 100.0f + 50.0f * std::sin(time_sec * 0.5f);
            data.BMP_Data.temperature = 25.0f + 2.0f * std::cos(time_sec * 0.1f);
            data.BMP_Data.pressure = 1013.25f - 0.12f * data.BMP_Data.altitude;
            data.Accel.X = std::sin(time_sec);
            data.Accel.Y = std::cos(time_sec);
            data.Accel.Z = -9.8f + std::sin(time_sec * 0.2f);
            data.Gyro.X = 0.1f * std::sin(time_sec);
            data.Gyro.Y = 0.1f * std::cos(time_sec);
            data.Gyro.Z = 0.05f * std::sin(time_sec * 0.5f);
            data.Magnetometer.X = 0.5f;
            data.Magnetometer.Y = 0.2f;
            data.Magnetometer.Z = 0.8f;
            data.AccelGyroTemperature = 30.0f;
            data.Thermometer = 20.0f;

            // Encode using Turtleford
            Proto_MainMessage msg = PbGen_FlightData(data);
            msg.message_type.in_flight_data.timestamp_ms = timestamp;

            std::array<std::byte, 512> buffer;
            size_t written = ProtoEncode(timestamp, msg, buffer, ProtoFlags::None);

            if (written > 0) {
                socket.send_to(asio::buffer(buffer.data(), written), endpoint);
                std::cout << "Sent " << written << " bytes. Alt=" << data.BMP_Data.altitude << "m\n";
            } else {
                std::cerr << "Failed to encode protobuf message\n";
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            timestamp += 500;
            time_sec += 0.5f;
        }
    } catch (std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
    }

    return 0;
}
