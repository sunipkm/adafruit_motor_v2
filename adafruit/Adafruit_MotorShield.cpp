/*!
 * @file MotorShield.cpp
 *
 * @mainpage Adafruit FXOS8700 accel/mag sensor driver
 *
 * @section intro_sec Introduction
 *
 * This is the library for the Adafruit Motor Shield V2 for Arduino.
 * It supports DC motors & Stepper motors with microstepping as well
 * as stacking-support. It is *not* compatible with the V1 library!
 * For use with the Motor Shield https://www.adafruit.com/products/1483
 * and Motor FeatherWing https://www.adafruit.com/product/2927
 *
 * This shield/wing uses I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section author Author
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include "Adafruit_MotorShield.hpp"
#include "meb_print.h"
#include <unistd.h>
#include <stdio.h>
#include <math.h>

#define LOW 0
#define HIGH 1

namespace Adafruit
{
    static uint8_t microstepcurve8[] = {0, 50, 98, 142, 180, 212, 236, 250, 255};
    ///! A sinusoial microstepping curve for the PWM output (8-bit range) with 17
    /// points - last one is start of next step.
    static uint8_t microstepcurve16[] = {0, 25, 50, 74, 98, 120, 141, 162, 180,
                                         197, 212, 225, 236, 244, 250, 253, 255};

    MotorShield::MotorShield(uint8_t addr, int bus)
    {
        _addr = addr;
        _bus = bus;
        initd = false;
    }

    MotorShield::~MotorShield()
    {
        for (int i = 0; i < 4; i++)
            dcmotors[i].fullOff();
        for (int i = 0; i < 2; i++)
            steppers[i].release();
        i2cbus_close(bus);
    }

    bool MotorShield::begin(uint16_t freq)
    {
        if (i2cbus_open(bus, _bus, _addr) < 0)
        {
            dbprintlf("Error opening I2C bus %d", _bus);
            return false;
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
            if (microsteps == STEP8)
            {
                steppers[port].microstepcurve = microstepcurve8;
            }
            else if (microsteps == STEP16)
            {
                steppers[port].microstepcurve = microstepcurve16;
            }
            else
            {
                dbprintlf("Microsteps %u not valid, setting microsteps to %u", (uint8_t)microsteps, (uint8_t)STEP16);
                steppers[port].microsteps = STEP16;
                steppers[port].microstepcurve = microstepcurve16;
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
        microsteps = STEP16;
        initd = false;
        microstepcurve = nullptr;
    }

    void StepperMotor::setSpeed(uint16_t rpm)
    {
        usperstep = 60000000 / ((uint32_t)revsteps * (uint32_t)rpm);
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

    void StepperMotor::setStep(MicroSteps microsteps)
    {
        std::lock_guard<std::mutex> lock(cs);
        if (microsteps == STEP8)
        {
            this->microsteps = microsteps;
            microstepcurve = microstepcurve8;
        }
        else if (microsteps == STEP16)
        {
            this->microsteps = microsteps;
            microstepcurve = microstepcurve16;
        }
        else
        {
            dbprintlf("Microsteps %u not valid, setting microsteps to %u", (uint8_t)microsteps, (uint8_t)STEP16);
            this->microsteps = STEP16;
            microstepcurve = microstepcurve16;
        }
    }

    void StepperMotor::step(uint16_t steps, MotorDir dir, MotorStyle style)
    {
        uint32_t uspers = usperstep;

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

        while (steps--)
        {
            onestep(dir, style);
            usleep(uspers);
        }
    }

    uint8_t StepperMotor::onestep(MotorDir dir, MotorStyle style)
    {
        std::lock_guard<std::mutex> lock(cs);
        uint8_t ocrb, ocra;

        ocra = ocrb = 255;

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

        dbprintlf("current step: %d, pwmA = %d, pwmB = %d", currentstep, ocra, ocrb);
        MC->setPWM(PWMApin, ocra * 16);
        MC->setPWM(PWMBpin, ocrb * 16);

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

    /*************** Steppers **************/
    /***************************************/

    /*************** MotorShield Private **************/
    /**************************************************/

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

    uint8_t MotorShield::read8(uint8_t addr)
    {
        uint8_t data = 0x0;
        int counter = 10;
        bool failed = true;
        while (failed && counter--)
        {
            failed = i2cbus_write(bus, &addr, 1) != 1;
        }
        if (failed)
            throw std::runtime_error("Could not read from I2C bus");

        usleep(20);

        counter = 2;
        failed = true;
        while (failed && counter--)
        {
            failed = i2cbus_read(bus, &addr, 1) != 1;
        }
        if (failed)
            throw std::runtime_error("Could not read from I2C bus");

        return data;
    }

    bool MotorShield::write8(uint8_t addr, uint8_t d)
    {
        int counter = 10;
        bool failed = true;
        while (failed && counter--)
        {
            failed = i2cbus_write(bus, &addr, 1) != 1;
        }
        if (failed)
            return false;
        return true;
    }

    /*************** MotorShield Private **************/
    /**************************************************/
}