#include "TurtlefordMain.h"
#include "Transport/TransferManager.h"
#include <iostream>
#include "WifiRadioWrapper.h"
#include <thread>
#include <chrono>
#include <atomic>
#include <csignal>

#include "Server.h"

namespace
{
std::atomic<bool> Run {true};
}

void SignalHandler(int signal)
{
    if (signal == SIGINT) { Run = false; }
}

void ReadInput()
{
    while (Run)
    {
        std::string Str;
        std::cin >> Str;
        if (Str == "stop") { Run = false; }
    }
}


typedef enum _Proto_FlightState {
    Proto_FlightState_Unknown = 0,
    Proto_FlightState_Unarmed = 1,
    Proto_FlightState_GroundIdle = 2,
    Proto_FlightState_InFlight = 3,
    Proto_FlightState_MainChute = 4
} Proto_FlightState;


typedef struct _Proto_Control {
    uint32_t timestamp;
    bool has_state;
    Proto_FlightState state;
} Proto_Control;

typedef struct _Proto_InFlightData_BMP_Data {
    float temperature;
    float pressure;
    float altitude;
} Proto_InFlightData_BMP_Data;

typedef struct _Proto_InFlightData_VectorF {
    float X;
    float Y;
    float Z;
} Proto_InFlightData_VectorF;

typedef struct _Proto_InFlightData {
    bool has_control;
    Proto_Control control;
    Proto_InFlightData_BMP_Data bmp_data;
    float accel_gyro_temperature;
    Proto_InFlightData_VectorF accel;
    Proto_InFlightData_VectorF gyro;
    Proto_InFlightData_VectorF magnetometer;
    float thermometer;
} Proto_InFlightData;

void recieve_data(Proto_InFlightData data){
    
}

int main(int argc, char* argv[])
{
    auto wifiManager = new ra::turtleford::WifiRadioWrapper();
    ra::turtleford::TurtlefordMain main(std::shared_ptr<ra::turtleford::ITransferManager>(wifiManager),0.0f);

    main.SetRecieveDataCallback(recieve_data);
    
    if (argc != 1 + 3)
    {
        std::cout << "multicast addr, multicast port, unicast port\n";
        return 1;
    }
    // Install a signal handler
    std::signal(SIGINT, SignalHandler);
    std::puts("to exit : <type stop> or <Ctrl-C>");

    Server Comm {argv[1], argv[2], argv[3]};

    // server inputs
    std::thread Input(ReadInput);

    while (Run)
    {
        Comm.SendData("abc");

        std::this_thread::sleep_for(std::chrono::seconds {3});
        wifiManager->Process();
        main.Update();
    }
    Input.join();

    std::cout << "Shutting down...\n";
    return 0;
}