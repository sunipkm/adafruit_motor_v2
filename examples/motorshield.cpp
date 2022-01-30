#include <adafruit/Adafruit_MotorShield.hpp>

int main()
{
    Adafruit::MotorDir state = Adafruit::MotorDir::FORWARD;
    int start = 10000;
    int stop = 90000;
    int loc = start;
    Adafruit::MotorShield AFMS;
    AFMS.begin();
    // 200 steps per rev (1.8 deg) on port 2 (M3 and M4)
    Adafruit::StepperMotor *motor = AFMS.getStepper(200, 2);

    motor->setSpeed(100); // 100 rpm
    // FORWARD == LS 2 (increase in wavelength)
    // BACKWARD == LS 1 (decrease in wavelength)
    while (loc < stop)
    {
        motor->step(1, state, Adafruit::MotorStyle::DOUBLE); // double coil
        if (state == Adafruit::MotorDir::FORWARD)
            loc++;
        else if (state == Adafruit::MotorDir::BACKWARD)
            loc--;
    }
    motor->release();
}