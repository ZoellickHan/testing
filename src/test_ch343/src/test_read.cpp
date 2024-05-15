
#include "rclcpp/rclcpp.hpp"
#include "newDriver.hpp"
#include <time.h>
#include "protocol.hpp"
#include "crc.hpp"

using namespace std;
using namespace newSerialDriver;
using namespace rm_serial_driver;
using namespace crc16;

shared_ptr<SerialConfig> config = make_shared<SerialConfig>(2000000,8,0,StopBit::ONE,Parity::NONE);
shared_ptr<Port>         port   = make_shared<Port>(config);
struct timespec ts1;

int    single       = 0;
int    sum          = 0;
int    crcError     = 0;
int    errorCounter = 0;
double errorRate    = 0;
double last_time    = 0;
double period       = 0;
double throughput   = 0;

int main(int argc, char **argv)
{
    GimbalMsg       gimbalmsg;
    SentryGimbalMsg sentryGimbalMsg;
    vector<uint8_t> header(3);
    vector<uint8_t> data;

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("testing");
    RCLCPP_INFO(node->get_logger(), "Start ch343 in node testing.");

    data.reserve(sizeof(GimbalMsg));
    port->openPort();
    port->init();
    
    clock_gettime(CLOCK_MONOTONIC, &ts1);
    last_time = ts1.tv_sec;
    while(true)
    {
        single = port->receive(header);
        sum   += single;
        if(header[2] == GIMBAL_MSG)
        {
            single = port->receive(data);
            data.insert(data.begin(), header[2]);
            data.insert(data.begin(), header[1]);
            data.insert(data.begin(), header[0]);
            gimbalmsg = fromVector(data);
            
            bool crc_ok = Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&gimbalmsg), sizeof(gimbalmsg));
            sum += single; 
            if (!crc_ok) {
                errorCounter += single;
                crcError++;
                continue;
            }
        }

        clock_gettime(CLOCK_MONOTONIC, &ts1);
        period = ts1.tv_sec - last_time;
        
        if(sum != 0) errorRate =(double)errorCounter / (double)sum;
        if(period != 0){throughput = sum/period*10000;} 
    }   

    printf("time: %f, throuhtput: %f, errorRate: %f \n",period,throughput,errorRate);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}