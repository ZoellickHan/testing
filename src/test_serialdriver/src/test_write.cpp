
#include "rclcpp/rclcpp.hpp"
#include "newDriver.hpp"
#include <time.h>
#include "protocol.hpp"
#include "crc.hpp"

using namespace std;
using namespace newSerialDriver;
using namespace rm_serial_driver;
using namespace crc16;

shared_ptr<SerialConfig> config = make_shared<SerialConfig>(3000000,8,0,StopBit::ONE,Parity::NONE);
shared_ptr<Port>         port   = make_shared<Port>(config);
struct timespec ts2;

int single          = 0;
int sum             = 0;
double last_time    = 0;
double period       = 0;
double throughput   = 0;

GimbalCommand gimbalCommand;
SentryGimbalMsg sentryGimbalMsg;

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("testing");
    
    RCLCPP_INFO(node->get_logger(), "Start ch343 in node testing.");
    port->openPort();
    port->init();
    
    clock_gettime(CLOCK_MONOTONIC, &ts2);
    last_time = ts2.tv_sec;

    while(true)
    {
        gimbalCommand.target_yaw = 0.5;
        gimbalCommand.target_pitch = 0.5;
        gimbalCommand.shoot_mode = 2;
        gimbalCommand.dataLen = sizeof(GimbalCommand) - 5;
        gimbalCommand.protocolID = GIMBAL_CMD;
        Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&gimbalCommand), sizeof(GimbalCommand));
        std::vector<uint8_t> data = toVector(gimbalCommand);

        single = port->transmit(data);
        sum += single;

        clock_gettime(CLOCK_MONOTONIC, &ts2);
        period = last_time - ts2.tv_sec;

        if(period != 0) throughput = port->getNumRead()/period*10000;
        printf("time: %ld, throuhtput: %f \n",period,throughput);
    }   

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}