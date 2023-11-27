/*
    pinger code copied from 
    https://gitlab.ensta-bretagne.fr/narvorpi/seatrac_driver/-/blob/master/examples/simple_pinger/src/simple_pinger.cpp
*/

#include <iostream>

#include <seatrac_driver/SeatracDriver.h>
#include <seatrac_driver/messages/Messages.h>
#include <seatrac_driver/commands.h>
using namespace narval::seatrac;

class MyDriver : public SeatracDriver
{
    public:

    MyDriver(const std::string& serialPort = "/dev/ttyUSB0") :
        SeatracDriver(serialPort)
    {}

    void ping_beacon(BID_E target, AMSGTYPE_E pingType = MSG_REQU) {
        messages::PingSend::Request req;
        req.target   = target;
        req.pingType = pingType;
        this->send(sizeof(req), (const uint8_t*)&req);
    }

    // this method is called on any message returned by the beacon.
    void on_message(CID_E msgId, const std::vector<uint8_t>& data) {
        //replace code in this method by your own
        switch(msgId) {
            default:
                std::cout << "Got message : " << msgId << std::endl << std::flush;
                break;
            case CID_PING_ERROR:
                {
                    messages::PingError response;
                    response = data;
                    std::cout << response << std::endl;
                    this->ping_beacon(response.beaconId);
                }
                break;
            case CID_PING_RESP:
                // std::cout << "Got a Ping Response" << std::endl << std::flush;
                {
                    messages::PingResp response;
                    response = data;
                    std::cout << response << std::endl;
                    this->ping_beacon(response.acoFix.srcId);
                }
                break;
            case CID_STATUS:
                // too many STATUS messages so bypassing display.
                break;
        }
    }
};

int main()
{
    MyDriver seatrac("/dev/ttyUSB0");
    
    command::ping_send(seatrac, BEACON_ID_10);

    getchar();

    return 0;
}

