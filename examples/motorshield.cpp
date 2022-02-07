#include <adafruit/Adafruit_MotorShield.hpp>
#include <signal.h>

volatile sig_atomic_t done = 0;
void sighandler(int sig)
{
    done = 1;
}

int main()
{
    signal(SIGINT | SIGHUP, sighandler);
    Adafruit::MotorDir state = Adafruit::MotorDir::FORWARD;
    Adafruit::MotorShield AFMS;
    AFMS.begin();
    // 200 steps per rev (1.8 deg) on port 2 (M3 and M4)
    Adafruit::StepperMotor *motor = AFMS.getStepper(200, 2);
    printf("Starting revolution at 1 RPM, press Ctrl + C to exit\n");
    motor->setSpeed(1); // 100 rpm
    // FORWARD == LS 2 (increase in wavelength)
    // BACKWARD == LS 1 (decrease in wavelength)
    int num_rev = 0;
    while (!done)
    {
        motor->step(200, state, Adafruit::MotorStyle::DOUBLE); // double coil, one full rotation
        printf("Performed %d rotations\n", ++num_rev);
    }
    motor->release();
    printf("\n\n");
}