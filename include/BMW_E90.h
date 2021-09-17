#ifndef BMW_E90_h
#define BMW_E90_h

/*  This library supports the Powertrain CAN messages for the BMW E90 for driving dash gauges, putting out malf lights etc
    Also reads gear lever, brake lights etc

*/

#include <stdint.h>
#include "my_fp.h"

class BMW_E90Class
{

public:
    static		void Tacho(int16_t speed);
    static		void absdsc(bool Brake_In);
    static		void GDis(); // geardisplay
    void Cas(int id, uint32_t data[2]);
    void DashOn();
    void DashOff();
    void Gear(int id, uint32_t data[2]);
    void Speed(int id, uint32_t data[2]);
    void BrakeStatus(int id, uint32_t data[2]);

    uint8_t getGear();
    void setGear();
    bool getTerminal15();
    void setTerminal15(bool);

    uint8_t handbrakeStat();

    BMW_E90Class()
    {
        Terminal15On = false;
        dashInit = false;
        gear = 0;
    }

private:
    bool Terminal15On;
    bool dashInit;
    uint8_t gear;
};

#endif /* BMW_E90_h */

