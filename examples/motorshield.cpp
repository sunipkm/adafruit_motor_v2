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
    AFMS.begin();
    // 200 steps per rev (1.8 deg) on port 2 (M3 and M4)
    Adafruit::StepperMotor *motor = AFMS.getStepper(200, 2, Adafruit::MicroSteps::STEP64); // Sets it to 64 step mode.
    double rpm = 0.3;
    printf("Starting revolution at %.3lf RPM, press Ctrl + C to exit\n", rpm);
    motor->setSpeed(rpm); // 100 rpm
    // FORWARD == LS 2 (increase in wavelength)
    // BACKWARD == LS 1 (decrease in wavelength)
    // float num_rev = 0;
    uint64_t told = get_ts_now(), tnow = 0;
    // while (!done)
    // {

    motor->step(200, state, Adafruit::MotorStyle::MICROSTEP); // double coil, one full rotation
    tnow = get_ts_now();
    // printf("Performed %.3f rotations in %.3f ms\n", (++num_rev) / 200, (tnow - told) * 0.001);
    uint64_t tdelta = tnow - told;
    printf("Took %" PRIu64 " microseconds (%.02lf seconds).", tdelta, tdelta / 1e6);
    
    // }
    motor->release();
    printf("\n\nExiting\n");
}