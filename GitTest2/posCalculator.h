#ifndef POSCALCULATOR_H
#define POSCALCULATOR_H

#include <magnet.h>
#include <Sensor.h>
class POSCALCULATOR_H
{
public:
    //should likely break up these functions within the mainfn into things like calibrate(), acquireCoVar(), beginTracking(), etc...
    //and then make all other functions private
    int mainfn();

private:
    magnet M1;
};


#endif // POSCALCULATOR_H
