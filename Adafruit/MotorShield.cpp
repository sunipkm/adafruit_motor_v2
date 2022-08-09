/*!
 * @file MotorShield.cpp
 *
 * @author Sunip K. Mukherjee (sunipkmukherjee@gmail.com), based on work by Limor Fried/Ladyada for Adafruit Industries.
 *
 * @brief This is the implementation file for the Adafruit Motor Shield V2 for Arduino.
 * It supports DC motors & Stepper motors with microstepping as well
 * as stacking-support. It is *not* compatible with the V1 library.
 *
 * @version Refer to changelog.
 * @date Refer to changelog.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include "MotorShield.hpp"
#include "meb_print.h"
#include <stdio.h>
#include <signal.h>
#include <math.h>

#include <algorithm>
#include <thread>
#include <vector>
#include <list>

#ifndef _DOXYGEN_
#define LOW 0
#define HIGH 1

static std::list<void *> lib_steppers;
static std::list<void *> lib_dcmotors;
static std::mutex handler_lock;


static void (*old_handler_sigint)(int) = nullptr;
static void (*old_handler_sighup)(int) = nullptr;
#ifdef SIGPIPE
static void (*old_handler_sigpipe)(int) = nullptr;
#endif

#endif // _DOXYGEN_

namespace Adafruit
{
    void MotorShield::sighandler(int sig)
    {
        std::lock_guard<std::mutex> lock(handler_lock);
        for (void *_stepper : lib_steppers)
        {
            Adafruit::StepperMotor *stepper = (Adafruit::StepperMotor *)_stepper;
            stepper->stopMotor();
        }

        for (void *_dcmot : lib_dcmotors)
        {
            Adafruit::DCMotor *motor = (Adafruit::DCMotor *)_dcmot;
            motor->fullOff();
        }

        if (old_handler_sigint != nullptr)
            old_handler_sigint(sig);
        if (old_handler_sighup != nullptr)
            old_handler_sighup(sig);
#ifdef SIGPIPE
        if (old_handler_sigpipe != nullptr)
            old_handler_sigpipe(sig);
#endif
    }

    /**
     * @brief Sinusoidal microstepping curve (sine curve between 0 and pi/2)
     * for the PWM output (12-bit range), with n + 1 points (in this case, n = 8).
     * The last point is the beginning of the next step.
     *
     */
    static uint16_t microstepcurve8[] = {0, 798, 1567, 2275, 2895,
                                         3404, 3783, 4016, 4095};

    /**
     * @brief Microstep curve for n = 16.
     *
     */
    static uint16_t microstepcurve16[] = {0, 401, 798, 1188, 1567, 1930, 2275, 2597, 2895, 3165, 3404,
                                          3611, 3783, 3918, 4016, 4075, 4095};

    /**
     * @brief Microstep curve for n = 32.
     *
     */
    static uint16_t microstepcurve32[] = {0, 200, 401, 600, 798, 995, 1188, 1379, 1567, 1750, 1930,
                                          2105, 2275, 2439, 2597, 2750, 2895, 3034, 3165, 3289, 3404, 3512,
                                          3611, 3701, 3783, 3855, 3918, 3972, 4016, 4050, 4075, 4090, 4095};

    /**
     * @brief Microstep curve for n = 64.
     *
     */
    static uint16_t microstepcurve64[] = {0, 100, 200, 301, 401, 501, 600, 700, 798, 897, 995,
                                          1092, 1188, 1284, 1379, 1473, 1567, 1659, 1750, 1841, 1930, 2018,
                                          2105, 2190, 2275, 2357, 2439, 2519, 2597, 2674, 2750, 2823, 2895,
                                          2965, 3034, 3100, 3165, 3228, 3289, 3348, 3404, 3459, 3512, 3563,
                                          3611, 3657, 3701, 3743, 3783, 3820, 3855, 3888, 3918, 3946, 3972,
                                          3995, 4016, 4034, 4050, 4064, 4075, 4083, 4090, 4093, 4095};

    /**
     * @brief Microstep curve for n = 128.
     *
     */
    static uint16_t microstepcurve128[] = {0, 50, 100, 150, 200, 251, 301, 351, 401, 451, 501,
                                           551, 600, 650, 700, 749, 798, 848, 897, 946, 995, 1043,
                                           1092, 1140, 1188, 1236, 1284, 1332, 1379, 1426, 1473, 1520, 1567,
                                           1613, 1659, 1705, 1750, 1796, 1841, 1885, 1930, 1974, 2018, 2061,
                                           2105, 2148, 2190, 2233, 2275, 2316, 2357, 2398, 2439, 2479, 2519,
                                           2558, 2597, 2636, 2674, 2712, 2750, 2787, 2823, 2859, 2895, 2930,
                                           2965, 3000, 3034, 3067, 3100, 3133, 3165, 3197, 3228, 3258, 3289,
                                           3318, 3348, 3376, 3404, 3432, 3459, 3486, 3512, 3537, 3563, 3587,
                                           3611, 3634, 3657, 3680, 3701, 3723, 3743, 3763, 3783, 3802, 3820,
                                           3838, 3855, 3872, 3888, 3903, 3918, 3932, 3946, 3959, 3972, 3984,
                                           3995, 4006, 4016, 4025, 4034, 4042, 4050, 4057, 4064, 4070, 4075,
                                           4079, 4083, 4087, 4090, 4092, 4093, 4094, 4095};

    /**
     * @brief Microstep curve for n = 256.
     *
     */
    static uint16_t microstepcurve256[] = {
        0, 25, 50, 75, 100, 125, 150, 175, 200, 226, 251,
        276, 301, 326, 351, 376, 401, 426, 451, 476, 501, 526,
        551, 575, 600, 625, 650, 675, 700, 724, 749, 774, 798,
        823, 848, 872, 897, 921, 946, 970, 995, 1019, 1043, 1067,
        1092, 1116, 1140, 1164, 1188, 1212, 1236, 1260, 1284, 1308, 1332,
        1355, 1379, 1403, 1426, 1450, 1473, 1497, 1520, 1543, 1567, 1590,
        1613, 1636, 1659, 1682, 1705, 1728, 1750, 1773, 1796, 1818, 1841,
        1863, 1885, 1908, 1930, 1952, 1974, 1996, 2018, 2040, 2061, 2083,
        2105, 2126, 2148, 2169, 2190, 2212, 2233, 2254, 2275, 2295, 2316,
        2337, 2357, 2378, 2398, 2419, 2439, 2459, 2479, 2499, 2519, 2539,
        2558, 2578, 2597, 2617, 2636, 2655, 2674, 2693, 2712, 2731, 2750,
        2768, 2787, 2805, 2823, 2841, 2859, 2877, 2895, 2913, 2930, 2948,
        2965, 2983, 3000, 3017, 3034, 3051, 3067, 3084, 3100, 3117, 3133,
        3149, 3165, 3181, 3197, 3212, 3228, 3243, 3258, 3274, 3289, 3304,
        3318, 3333, 3348, 3362, 3376, 3390, 3404, 3418, 3432, 3446, 3459,
        3473, 3486, 3499, 3512, 3525, 3537, 3550, 3563, 3575, 3587, 3599,
        3611, 3623, 3634, 3646, 3657, 3668, 3680, 3691, 3701, 3712, 3723,
        3733, 3743, 3753, 3763, 3773, 3783, 3792, 3802, 3811, 3820, 3829,
        3838, 3847, 3855, 3864, 3872, 3880, 3888, 3896, 3903, 3911, 3918,
        3925, 3932, 3939, 3946, 3953, 3959, 3966, 3972, 3978, 3984, 3989,
        3995, 4000, 4006, 4011, 4016, 4021, 4025, 4030, 4034, 4038, 4042,
        4046, 4050, 4054, 4057, 4061, 4064, 4067, 4070, 4072, 4075, 4077,
        4079, 4081, 4083, 4085, 4087, 4088, 4090, 4091, 4092, 4093, 4093,
        4094, 4094, 4094, 4095};

    /**
     * @brief Microstep curve for n = 512.
     *
     */
    static uint16_t microstepcurve512[] = {
        0, 12, 25, 37, 50, 62, 75, 87, 100, 113, 125,
        138, 150, 163, 175, 188, 200, 213, 226, 238, 251, 263,
        276, 288, 301, 313, 326, 338, 351, 363, 376, 388, 401,
        413, 426, 438, 451, 463, 476, 488, 501, 513, 526, 538,
        551, 563, 575, 588, 600, 613, 625, 638, 650, 662, 675,
        687, 700, 712, 724, 737, 749, 761, 774, 786, 798, 811,
        823, 835, 848, 860, 872, 884, 897, 909, 921, 933, 946,
        958, 970, 982, 995, 1007, 1019, 1031, 1043, 1055, 1067, 1080,
        1092, 1104, 1116, 1128, 1140, 1152, 1164, 1176, 1188, 1200, 1212,
        1224, 1236, 1248, 1260, 1272, 1284, 1296, 1308, 1320, 1332, 1344,
        1355, 1367, 1379, 1391, 1403, 1414, 1426, 1438, 1450, 1462, 1473,
        1485, 1497, 1508, 1520, 1532, 1543, 1555, 1567, 1578, 1590, 1601,
        1613, 1624, 1636, 1647, 1659, 1670, 1682, 1693, 1705, 1716, 1728,
        1739, 1750, 1762, 1773, 1784, 1796, 1807, 1818, 1829, 1841, 1852,
        1863, 1874, 1885, 1897, 1908, 1919, 1930, 1941, 1952, 1963, 1974,
        1985, 1996, 2007, 2018, 2029, 2040, 2051, 2061, 2072, 2083, 2094,
        2105, 2116, 2126, 2137, 2148, 2158, 2169, 2180, 2190, 2201, 2212,
        2222, 2233, 2243, 2254, 2264, 2275, 2285, 2295, 2306, 2316, 2327,
        2337, 2347, 2357, 2368, 2378, 2388, 2398, 2409, 2419, 2429, 2439,
        2449, 2459, 2469, 2479, 2489, 2499, 2509, 2519, 2529, 2539, 2548,
        2558, 2568, 2578, 2588, 2597, 2607, 2617, 2626, 2636, 2646, 2655,
        2665, 2674, 2684, 2693, 2703, 2712, 2721, 2731, 2740, 2750, 2759,
        2768, 2777, 2787, 2796, 2805, 2814, 2823, 2832, 2841, 2850, 2859,
        2868, 2877, 2886, 2895, 2904, 2913, 2922, 2930, 2939, 2948, 2957,
        2965, 2974, 2983, 2991, 3000, 3008, 3017, 3025, 3034, 3042, 3051,
        3059, 3067, 3076, 3084, 3092, 3100, 3108, 3117, 3125, 3133, 3141,
        3149, 3157, 3165, 3173, 3181, 3189, 3197, 3204, 3212, 3220, 3228,
        3235, 3243, 3251, 3258, 3266, 3274, 3281, 3289, 3296, 3304, 3311,
        3318, 3326, 3333, 3340, 3348, 3355, 3362, 3369, 3376, 3383, 3390,
        3397, 3404, 3411, 3418, 3425, 3432, 3439, 3446, 3452, 3459, 3466,
        3473, 3479, 3486, 3492, 3499, 3505, 3512, 3518, 3525, 3531, 3537,
        3544, 3550, 3556, 3563, 3569, 3575, 3581, 3587, 3593, 3599, 3605,
        3611, 3617, 3623, 3629, 3634, 3640, 3646, 3652, 3657, 3663, 3668,
        3674, 3680, 3685, 3691, 3696, 3701, 3707, 3712, 3717, 3723, 3728,
        3733, 3738, 3743, 3748, 3753, 3758, 3763, 3768, 3773, 3778, 3783,
        3788, 3792, 3797, 3802, 3806, 3811, 3816, 3820, 3825, 3829, 3834,
        3838, 3842, 3847, 3851, 3855, 3859, 3864, 3868, 3872, 3876, 3880,
        3884, 3888, 3892, 3896, 3899, 3903, 3907, 3911, 3915, 3918, 3922,
        3925, 3929, 3932, 3936, 3939, 3943, 3946, 3949, 3953, 3956, 3959,
        3962, 3966, 3969, 3972, 3975, 3978, 3981, 3984, 3987, 3989, 3992,
        3995, 3998, 4000, 4003, 4006, 4008, 4011, 4013, 4016, 4018, 4021,
        4023, 4025, 4028, 4030, 4032, 4034, 4036, 4038, 4040, 4042, 4044,
        4046, 4048, 4050, 4052, 4054, 4056, 4057, 4059, 4061, 4062, 4064,
        4065, 4067, 4068, 4070, 4071, 4072, 4074, 4075, 4076, 4077, 4078,
        4079, 4080, 4081, 4082, 4083, 4084, 4085, 4086, 4087, 4088, 4088,
        4089, 4090, 4090, 4091, 4091, 4092, 4092, 4093, 4093, 4093, 4094,
        4094, 4094, 4094, 4094, 4094, 4094, 4095};

    MotorShield::MotorShield(uint8_t addr, int bus, bool register_sighandler)
    {
        _addr = addr;
        _bus = bus;
        initd = false;
        if (register_sighandler)
        {
            struct sigaction sa, sa_old;
            memset(&sa, 0x0, sizeof(struct sigaction));
            memset(&sa_old, 0x0, sizeof(struct sigaction));
            int ret = sigaction(SIGINT, NULL, &sa_old);
            if (ret)
            {
                throw std::runtime_error("Error " + std::to_string(errno) + " getting old signal handler: " + std::string(strerror(errno)));
            }
            old_handler_sigint = sa_old.sa_handler;
            sa.sa_handler = sighandler;
            ret = sigaction(SIGINT, &sa, NULL);
            if (ret)
            {
                throw std::runtime_error("Error " + std::to_string(errno) + " getting old signal handler: " + std::string(strerror(errno)));
            }

#ifdef ADAFRUIT_ENABLE_SIGHUP
            memset(&sa, 0x0, sizeof(struct sigaction));
            memset(&sa_old, 0x0, sizeof(struct sigaction));
            ret = sigaction(SIGHUP, NULL, &sa_old);
            if (ret)
            {
                throw std::runtime_error("Error " + std::to_string(errno) + " getting old signal handler: " + std::string(strerror(errno)));
            }
            old_handler_sighup = sa_old.sa_handler;
            sa.sa_handler = sighandler;
            ret = sigaction(SIGINT, &sa, NULL);
            if (ret)
            {
                throw std::runtime_error("Error " + std::to_string(errno) + " getting old signal handler: " + std::string(strerror(errno)));
            }
#endif

#ifdef ADAFRUIT_ENABLE_SIGPIPE
#ifdef SIGPIPE
            memset(&sa, 0x0, sizeof(struct sigaction));
            memset(&sa_old, 0x0, sizeof(struct sigaction));
            ret = sigaction(SIGPIPE, NULL, &sa_old);
            if (ret)
            {
                throw std::runtime_error("Error " + std::to_string(errno) + " getting old signal handler: " + std::string(strerror(errno)));
            }
            old_handler_sigpipe = sa_old.sa_handler;
            sa.sa_handler = sighandler;
            ret = sigaction(SIGPIPE, &sa, NULL);
            if (ret)
            {
                throw std::runtime_error("Error " + std::to_string(errno) + " getting old signal handler: " + std::string(strerror(errno)));
            }
#endif
#endif
        }
    }

    MotorShield::~MotorShield()
    {
        std::lock_guard<std::mutex> lock(handler_lock);
        for (int i = 0; i < 4; i++)
        {
            // remove motor from list of vectors for signal handler
            for (std::list<void *>::iterator it = lib_dcmotors.begin(); it != lib_dcmotors.end(); it++)
            {
                if ((*it) == (void *)&dcmotors[i])
                {
                    lib_dcmotors.erase(it);
                    break;
                }
            }
            if (dcmotors[i].initd)
            {
                dcmotors[i].fullOff();
            }
        }
        for (int i = 0; i < 2; i++)
        {
            for (std::list<void *>::iterator it = lib_steppers.begin(); it != lib_steppers.end(); it++)
            {
                if ((*it) == (void *)&steppers[i])
                {
                    lib_steppers.erase(it);
                    break;
                }
            }
            if (steppers[i].initd)
            {
                steppers[i].release();
            }
        }
        i2cbus_close(bus);
    }

    bool _Catchable MotorShield::begin(uint16_t freq)
    {
        if (i2cbus_open(bus, _bus, _addr) < 0)
        {
            dbprintlf("Error opening I2C bus %d", _bus);
            throw std::runtime_error("Could not open device " + std::to_string(_addr) + " on bus " + std::to_string(_bus));
        }
        bool status = true;
        status &= reset();
        _freq = freq;
        status &= setPWMFreq(_freq); // This is the maximum PWM frequency
        for (uint8_t i = 0; i < 16; i++)
            status &= setPWM(i, 0, 0);
        initd = status;
        return status;
    }

    bool MotorShield::setPWM(uint8_t pin, uint16_t value)
    {
        if (!initd)
        {
            bprintlf("MotorShield object not initialized, please invoke begin().");
            return false;
        }
        if (value > 4095)
        {
            return setPWM(pin, 4096, 0);
        }
        else
        {
            return setPWM(pin, 0, value);
        }
        return false;
    }

    bool MotorShield::setPin(uint8_t pin, bool value)
    {
        if (!initd)
        {
            bprintlf("MotorShield object not initialized, please invoke begin().");
            return false;
        }
        if (value == LOW)
        {
            return setPWM(pin, 0, 0);
        }
        else
        {
            return setPWM(pin, 4096, 0);
        }
        return false;
    }

    DCMotor *MotorShield::getMotor(uint8_t num)
    {
        if (!initd)
        {
            bprintlf("MotorShield object not initialized, please invoke begin().");
            return NULL;
        }
        if (num > 4)
        {
            bprintlf("Motor number %u out of range [1-4]", num);
            return NULL;
        }

        num--;

        if (dcmotors[num].initd == false)
        {
            // not init'd yet!
            dcmotors[num].initd = true;
            dcmotors[num].MC = this;
            uint8_t pwm = 8, in1 = 9, in2 = 10;
            if (num == 0)
            {
                pwm = 8;
                in2 = 9;
                in1 = 10;
            }
            else if (num == 1)
            {
                pwm = 13;
                in2 = 12;
                in1 = 11;
            }
            else if (num == 2)
            {
                pwm = 2;
                in2 = 3;
                in1 = 4;
            }
            else if (num == 3)
            {
                pwm = 7;
                in2 = 6;
                in1 = 5;
            }
            dcmotors[num].PWMpin = pwm;
            dcmotors[num].IN1pin = in1;
            dcmotors[num].IN2pin = in2;
        }
        std::lock_guard<std::mutex> lock(handler_lock);
        lib_dcmotors.push_back(&dcmotors[num]);
        return &dcmotors[num];
    }

    StepperMotor *MotorShield::getStepper(uint16_t steps, uint8_t port, MicroSteps microsteps)
    {
        if (!initd)
        {
            bprintlf("MotorShield object not initialized, please invoke begin().");
            return NULL;
        }
        if (port > 2)
        {
            bprintlf("Motor number %u out of range [1-2]", port);
            return NULL;
        }

        port--;

        if (steppers[port].initd == false)
        {
            // not init'd yet!
            steppers[port].initd = true;
            steppers[port].revsteps = steps;
            steppers[port].MC = this;
            steppers[port].microsteps = microsteps;
            switch (microsteps)
            {
#ifndef _DOXYGEN_
#define MCASE(x)                                           \
    case (STEP##x):                                        \
        steppers[port].microsteps = STEP##x;               \
        steppers[port].microstepcurve = microstepcurve##x; \
        break;
#endif // _DOXYGEN_

                MCASE(8)
                MCASE(16)
                MCASE(32)
                MCASE(64)
                MCASE(128)
                MCASE(256)
                MCASE(512)
#undef MCASE

            default:
                dbprintlf("Microsteps %u not valid, setting microsteps to %u", (uint8_t)microsteps, (uint8_t)STEP16);
                steppers[port].microsteps = STEP16;
                steppers[port].microstepcurve = microstepcurve16;
                break;
            }
            uint8_t pwma = 8, pwmb = 13, ain1 = 9, ain2 = 10, bin1 = 11, bin2 = 12;
            if (port == 0)
            {
                pwma = 8;
                ain2 = 9;
                ain1 = 10;
                pwmb = 13;
                bin2 = 12;
                bin1 = 11;
            }
            else if (port == 1)
            {
                pwma = 2;
                ain2 = 3;
                ain1 = 4;
                pwmb = 7;
                bin2 = 6;
                bin1 = 5;
            }
            steppers[port].PWMApin = pwma;
            steppers[port].PWMBpin = pwmb;
            steppers[port].AIN1pin = ain1;
            steppers[port].AIN2pin = ain2;
            steppers[port].BIN1pin = bin1;
            steppers[port].BIN2pin = bin2;
        }
        std::lock_guard<std::mutex> lock(handler_lock);
        lib_steppers.push_back((void *)&steppers[port]);
        return &steppers[port];
    }

    /*************** Motors ****************/
    /***************************************/

    DCMotor::DCMotor(void)
    {
        MC = NULL;
        initd = false;
        PWMpin = IN1pin = IN2pin = 0;
    }

    void DCMotor::run(MotorDir cmd)
    {
        switch (cmd)
        {
        case FORWARD:
            MC->setPin(IN2pin, LOW); // take low first to avoid 'break'
            MC->setPin(IN1pin, HIGH);
            break;
        case BACKWARD:
            MC->setPin(IN1pin, LOW); // take low first to avoid 'break'
            MC->setPin(IN2pin, HIGH);
            break;
        case RELEASE:
            MC->setPin(IN1pin, LOW);
            MC->setPin(IN2pin, LOW);
            break;
        case BRAKE:
            dbprintlf("Feature not implemented.");
            break;
        default:
            dbprintlf("Direction %u unknown", cmd);
            break;
        }
    }

    void DCMotor::setSpeed(uint8_t speed)
    {
        MC->setPWM(PWMpin, speed * 16);
    }

    void DCMotor::setSpeedFine(uint16_t speed)
    {
        MC->setPWM(PWMpin, speed > 4095 ? 4095 : speed);
    }

    void DCMotor::fullOff()
    {
        MC->setPWM(PWMpin, 0);
    }

    void DCMotor::fullOn()
    {
        MC->setPWM(PWMpin, 4095);
    }

    /*************** Motors ****************/
    /***************************************/

    /*************** Steppers **************/
    /***************************************/

    StepperMotor::StepperMotor(void)
    {
        revsteps = currentstep = 0;
        MC = nullptr;
        microsteps = STEP16;
        initd = false;
        microstepcurve = microstepcurve16;
        usperstep = 0;
        stop = false;
        moving = false;
    }

    void StepperMotor::release(void)
    {
        MC->setPin(AIN1pin, LOW);
        MC->setPin(AIN2pin, LOW);
        MC->setPin(BIN1pin, LOW);
        MC->setPin(BIN2pin, LOW);
        MC->setPWM(PWMApin, 0);
        MC->setPWM(PWMBpin, 0);
    }

    bool _Catchable StepperMotor::setSpeed(double rpm)
    {
        if (rpm <= 0)
            throw std::runtime_error("Motor speed can not be negative or zero.");
        std::unique_lock<std::mutex> lock(cs, std::try_to_lock);
        if (lock.owns_lock())
        {
            usperstep = 60000000ULL / ((uint32_t)revsteps * rpm);
            return true;
        }
        return false;
    }

    bool StepperMotor::setStep(MicroSteps microsteps)
    {
        std::unique_lock<std::mutex> lock(cs, std::try_to_lock);
        if (lock.owns_lock())
        {
            switch (microsteps)
            {
#ifndef _DOXYGEN_
#define MCASE(x)                            \
    case (STEP##x):                         \
        this->microsteps = STEP##x;         \
        microstepcurve = microstepcurve##x; \
        break;
#endif // _DOXYGEN_

                MCASE(8)
                MCASE(16)
                MCASE(32)
                MCASE(64)
                MCASE(128)
                MCASE(256)
                MCASE(512)
#undef MCASE

            default:
                dbprintlf("Microsteps %u not valid, setting microsteps to %u", (uint8_t)microsteps, (uint8_t)STEP16);
                this->microsteps = STEP16;
                microstepcurve = microstepcurve16;
                break;
            }
            return true;
        }
        return false;
    }

    void _Catchable StepperMotor::step(uint16_t steps, MotorDir dir, MotorStyle style, bool blocking, StepperMotorCB_t callback_fn, void *callback_fn_data)
    {
        if (usperstep == 0)
            throw std::runtime_error("RPM has to be set before stepping the motor.");
        if (blocking)
        {
            std::unique_lock<std::mutex> lock(cs);
            uint64_t uspers = usperstep;

            if (style == INTERLEAVE)
            {
                uspers /= 2;
            }
            else if (style == MICROSTEP)
            {
                uspers /= microsteps;
                steps *= microsteps;
                dbprintlf("steps = %d", steps);
            }
            stop = false;
            StepperMotorTimerData data = {this, steps, dir, style, microsteps, callback_fn, callback_fn_data};
            clkgen_t clk = create_clk(uspers * 1000LLU, stepHandlerFn, &data);
            if (cond.wait_for(lock, std::chrono::microseconds(steps * uspers)) == std::cv_status::timeout)
            {
                while (data.steps)
                {
                    usleep(uspers);
                }
            }
            destroy_clk(clk);
            moving = false;
        }
        else // non-blocking, spawn thread
        {
            std::thread stepThread(stepThreadFn, this, steps, dir, style, callback_fn, callback_fn_data);
            stepThread.detach();
        }
    }

    bool StepperMotor::isMoving() const
    {
        return moving;
    }

    void StepperMotor::stopMotor()
    {
        if (moving)
            stop = true;
    }

    uint64_t _Catchable StepperMotor::getStepPeriod() const
    {
        if (usperstep == 0)
            throw std::runtime_error("RPM has to be set before stepping the motor.");
        return usperstep;
    }

    uint8_t StepperMotor::onestep(MotorDir dir, MotorStyle style)
    {
        uint16_t ocrb, ocra;

        ocra = ocrb = 4095;

        // next determine what sort of stepping procedure we're up to
        if (style == SINGLE)
        {
            if ((currentstep / (microsteps / 2)) % 2)
            { // we're at an odd step, weird
                if (dir == FORWARD)
                {
                    currentstep += microsteps / 2;
                }
                else
                {
                    currentstep -= microsteps / 2;
                }
            }
            else
            { // go to the next even step
                if (dir == FORWARD)
                {
                    currentstep += microsteps;
                }
                else
                {
                    currentstep -= microsteps;
                }
            }
        }
        else if (style == DOUBLE)
        {
            if (!(currentstep / (microsteps / 2) % 2))
            { // we're at an even step, weird
                if (dir == FORWARD)
                {
                    currentstep += microsteps / 2;
                }
                else
                {
                    currentstep -= microsteps / 2;
                }
            }
            else
            { // go to the next odd step
                if (dir == FORWARD)
                {
                    currentstep += microsteps;
                }
                else
                {
                    currentstep -= microsteps;
                }
            }
        }
        else if (style == INTERLEAVE)
        {
            if (dir == FORWARD)
            {
                currentstep += microsteps / 2;
            }
            else
            {
                currentstep -= microsteps / 2;
            }
        }

        if (style == MICROSTEP)
        {
            if (dir == FORWARD)
            {
                currentstep++;
            }
            else
            {
                // BACKWARDS
                currentstep--;
            }

            currentstep += microsteps * 4;
            currentstep %= microsteps * 4;

            ocra = ocrb = 0;
            if (currentstep < microsteps)
            {
                ocra = microstepcurve[microsteps - currentstep];
                ocrb = microstepcurve[currentstep];
            }
            else if ((currentstep >= microsteps) && (currentstep < microsteps * 2))
            {
                ocra = microstepcurve[currentstep - microsteps];
                ocrb = microstepcurve[microsteps * 2 - currentstep];
            }
            else if ((currentstep >= microsteps * 2) &&
                     (currentstep < microsteps * 3))
            {
                ocra = microstepcurve[microsteps * 3 - currentstep];
                ocrb = microstepcurve[currentstep - microsteps * 2];
            }
            else if ((currentstep >= microsteps * 3) &&
                     (currentstep < microsteps * 4))
            {
                ocra = microstepcurve[currentstep - microsteps * 3];
                ocrb = microstepcurve[microsteps * 4 - currentstep];
            }
        }

        currentstep += microsteps * 4;
        currentstep %= microsteps * 4;

        dbprintlf("current step: %u, pwmA = %u, pwmB = %u", currentstep, ocra, ocrb);
        MC->setPWM(PWMApin, ocra);
        MC->setPWM(PWMBpin, ocrb);

        // release all
        uint8_t latch_state = 0; // all motor pins to 0

        if (style == MICROSTEP)
        {
            if (currentstep < microsteps)
                latch_state |= 0x03;
            if ((currentstep >= microsteps) && (currentstep < microsteps * 2))
                latch_state |= 0x06;
            if ((currentstep >= microsteps * 2) && (currentstep < microsteps * 3))
                latch_state |= 0x0C;
            if ((currentstep >= microsteps * 3) && (currentstep < microsteps * 4))
                latch_state |= 0x09;
        }
        else
        {
            switch (currentstep / (microsteps / 2))
            {
            case 0:
                latch_state |= 0x1; // energize coil 1 only
                break;
            case 1:
                latch_state |= 0x3; // energize coil 1+2
                break;
            case 2:
                latch_state |= 0x2; // energize coil 2 only
                break;
            case 3:
                latch_state |= 0x6; // energize coil 2+3
                break;
            case 4:
                latch_state |= 0x4; // energize coil 3 only
                break;
            case 5:
                latch_state |= 0xC; // energize coil 3+4
                break;
            case 6:
                latch_state |= 0x8; // energize coil 4 only
                break;
            case 7:
                latch_state |= 0x9; // energize coil 1+4
                break;
            }
        }
        dbprintlf("Latch: 0x%02x", latch_state);

        if (latch_state & 0x1)
        {
            MC->setPin(AIN2pin, HIGH);
        }
        else
        {
            MC->setPin(AIN2pin, LOW);
        }
        if (latch_state & 0x2)
        {
            MC->setPin(BIN1pin, HIGH);
        }
        else
        {
            MC->setPin(BIN1pin, LOW);
        }
        if (latch_state & 0x4)
        {
            MC->setPin(AIN1pin, HIGH);
        }
        else
        {
            MC->setPin(AIN1pin, LOW);
        }
        if (latch_state & 0x8)
        {
            MC->setPin(BIN2pin, HIGH);
        }
        else
        {
            MC->setPin(BIN2pin, LOW);
        }

        return currentstep;
    }

    void StepperMotor::stepHandlerFn(clkgen_t clk, void *data_)
    {
        struct StepperMotorTimerData *data = (struct StepperMotorTimerData *)data_;
        StepperMotor *_this = data->_this;
        // if at odd microstep we HAVE to step until we reach an integral step
        if ((data->steps % data->msteps) && (data->style == MotorStyle::MICROSTEP))
        {
            _this->moving = true;
            _this->onestep(data->dir, data->style);
            data->steps--;
            return; // can not let this reach the unblock check
        }
        else if (data->steps && !(_this->stop)) // integral step/not microstepping, no Ctrl+C received, emergency stop not pressed
        {
            _this->moving = true;
            _this->onestep(data->dir, data->style);
            data->steps--;
        }
        if (data->steps == 0 || _this->stop) // end reached/done = 1
        {
            _this->moving = false;
            _this->cond.notify_all();
        }
        if (_this->moving && (data->callback_fn != nullptr))
        {
            data->callback_fn(_this, data->callback_user_data);
        }
    }

    void StepperMotor::stepThreadFn(StepperMotor *mot, uint16_t steps, MotorDir dir, MotorStyle style, StepperMotorCB_t callback_fn, void *callback_fn_data)
    {
        std::unique_lock<std::mutex> lock(mot->cs);
        uint64_t uspers = mot->usperstep;

        if (style == INTERLEAVE)
        {
            uspers /= 2;
        }
        else if (style == MICROSTEP)
        {
            uspers /= mot->microsteps;
            steps *= mot->microsteps;
        }
        mot->stop = false;
        StepperMotorTimerData data = {mot, steps, dir, style, mot->microsteps, callback_fn, callback_fn_data};
        clkgen_t clk = create_clk(uspers * 1000LLU, stepHandlerFn, &data);
        if (mot->cond.wait_for(lock, std::chrono::microseconds(steps * uspers)) == std::cv_status::timeout)
        {
            while (data.steps)
            {
                usleep(uspers);
            }
        }
        destroy_clk(clk);
        mot->moving = false;
    }

    /*************** Steppers **************/
    /***************************************/

    /*************** MotorShield Private **************/
    /**************************************************/
#ifndef _DOXYGEN_
#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9

#define ALLLED_ON_L 0xFA
#define ALLLED_ON_H 0xFB
#define ALLLED_OFF_L 0xFC
#define ALLLED_OFF_H 0xFD

#define PCA9685_SUBADR1 0x2
#define PCA9685_SUBADR2 0x3
#define PCA9685_SUBADR3 0x4

#define PCA9685_MODE1 0x0
#define PCA9685_PRESCALE 0xFE
#endif // _DOXYGEN_

    bool MotorShield::reset()
    {
        return write8(PCA9685_MODE1, 0x0) > 0;
    }

    bool MotorShield::setPWMFreq(float freq)
    {
        dbprintlf("Attempting to set freq: %f", freq);
        freq *=
            0.9; // Correct for overshoot in the frequency setting (see issue #11).

        float prescaleval = 25000000;
        prescaleval /= 4096;
        prescaleval /= freq;
        prescaleval -= 1;
        dbprintlf("Estimated pre-scale: %f", prescaleval);
        uint8_t prescale = floor(prescaleval + 0.5);
        dbprintlf("Final pre-scale: %d", prescale);

        uint8_t oldmode;
        try
        {
            oldmode = read8(PCA9685_MODE1);
        }
        catch (const std::exception &e)
        {
            dbprintlf("Error reading PCA9685_MODE1: %s", e.what());
            return false;
        }
        uint8_t newmode = (oldmode & 0x7F) | 0x10; // sleep
        write8(PCA9685_MODE1, newmode);            // go to sleep
        write8(PCA9685_PRESCALE, prescale);        // set the prescaler
        write8(PCA9685_MODE1, oldmode);
        usleep(5 * 1000);
        write8(PCA9685_MODE1,
               oldmode |
                   0xa1); //  This sets the MODE1 register to turn on auto increment.
                          // This is why the beginTransmission below was not working.
#if (ADAFRUIT_MOTORSHIELD_DEBUG > 0)
        try
        {
            newmode = read8(PCA9685_MODE1);
        }
        catch (const std::exception &e)
        {
            dbprintlf("Error reading PCA9685_MODE1 after operation: %s", e.what());
        }
        dbprintlf("Mode now: 0x%02x", newmode);
#endif
        return true;
    }

    bool MotorShield::setPWM(uint8_t num, uint16_t on,
                             uint16_t off)
    {
        dbprintlf("Setting PWM %u: 0x%04x -> 0x%04x", num, on, off);

        // this is a single transaction
        uint8_t buf[5] = {LED0_ON_L + 4 * num, on, on >> 8, off, off >> 8};
        int counter = 10;
        bool failed = true;
        while (failed && counter--)
        {
            failed = i2cbus_write(bus, buf, sizeof(buf)) != sizeof(buf);
        }
        if (failed)
        {
            dbprintlf("Failed to write to port 0x%02x", LED0_ON_L + 4 * num);
            return false;
        }
        return true;
    }

    uint8_t _Catchable MotorShield::read8(uint8_t addr)
    {
        uint8_t data = 0x0;
        int counter = 10;
        bool failed = true;
        while (failed && counter--)
        {
            failed = i2cbus_xfer(bus, &addr, 1, &data, 1, 20) != 1;
        }
        if (failed)
            throw std::runtime_error("Could not execute read/write transaction on I2C bus");
        return data;
    }

    bool MotorShield::write8(uint8_t addr, uint8_t d)
    {
        int counter = 10;
        bool failed = true;
        uint8_t buf[2] = {addr, d};
        while (failed && counter--)
        {
            failed = i2cbus_write(bus, buf, 2) != 2;
        }
        if (failed)
            return false;
        return true;
    }

    /*************** MotorShield Private **************/
    /**************************************************/
}