#include <Adafruit/MotorShield.hpp>
#include <signal.h>
#include <inttypes.h>
#include <chrono>
uint64_t get_ts_now()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

volatile sig_atomic_t done = 0;
void sighandler(int sig)
{
    done = 1;
}

int main()
{
    signal(SIGINT, sighandler);
    Adafruit::MotorDir state = Adafruit::MotorDir::FORWARD;
    Adafruit::MotorShield AFMS;
    AFMS.begin(); // initializes the motor shield
    // 200 steps per rev (1.8 deg) on port 2 (M1 and M2)
    Adafruit::StepperMotor *motor = AFMS.getStepper(200, 1, Adafruit::MicroSteps::STEP64); // Sets it to 64 step mode.
    double rpm = 0.3;
    printf("Starting revolution at %.3lf RPM, press Ctrl + C to exit\n", rpm);
    motor->setSpeed(rpm); // set speed

    uint64_t told = get_ts_now(), tnow = 0; // get starting timestamp

    motor->step(200, state, Adafruit::MotorStyle::MICROSTEP); // double coil, one full rotation

    tnow = get_ts_now();
    uint64_t tdelta = tnow - told;
    printf("Took %" PRIu64 " microseconds (%.02lf seconds).", tdelta, tdelta / 1e6);

    motor->release(); // release from stall
    printf("\n\nExiting\n");
}