/**
 * @file Adafruit_MotorShield.hpp
 * @author Sunip K. Mukherjee (sunipkmukherjee@gmail.com), based on work by Limor Fried/Ladyada for Adafruit Industries.
 * @brief This is the library for the Adafruit Motor Shield V2 for Arduino. 
 * It supports DC motors & Stepper motors with microstepping as well 
 * as stacking-support. It is *not* compatible with the V1 library!
 * @version 1.1.0
 * @date 2022-01-30
 * 
 * @copyright Copyright (c) 2022
 * 
 */

/******************************************************************
 This is the library for the Adafruit Motor Shield V2 for Arduino.
 It supports DC motors & Stepper motors with microstepping as well
 as stacking-support. It is *not* compatible with the V1 library!

 It will only work with https://www.adafruit.com/products/1483

 Adafruit invests time and resources providing this open
 source code, please support Adafruit and open-source hardware
 by purchasing products from Adafruit!

 Written by Limor Fried/Ladyada for Adafruit Industries.
 BSD license, check license.txt for more information.
 All text above must be included in any redistribution.
 ******************************************************************/

#ifndef _MotorShield_hpp_
#define _MotorShield_hpp_

#include <stdint.h>
#include "i2cbus/i2cbus.h"

#include <mutex>

namespace Adafruit
{
#if !defined(ADAFRUIT_MOTORSHIELD_DEBUG)
/**
 * @brief Enable debug printouts for motor shield.
 * 
 */
#define ADAFRUIT_MOTORSHIELD_DEBUG 0
#define MEB_DBGLVL 0
#elif ADAFRUIT_MOTORSHIELD_DEBUG > 0
#define MEB_DBGLVL MEB_DBG_ALL
#endif
    typedef enum _MotorStyle : uint8_t
    {
        SINGLE = 1,
        DOUBLE = 2,
        INTERLEAVE = 3,
        MICROSTEP = 4
    } MotorStyle;

    typedef enum _MotorDir : uint8_t
    {
        FORWARD = 1,
        BACKWARD = 2,
        BRAKE = 3,
        RELEASE = 4
    } MotorDir;

    typedef enum _MicroSteps : uint8_t
    {
        STEP8 = 8,
        STEP16 = 16
    } MicroSteps;

    class MotorShield;

    /**
     * @brief Object that controls and keeps state for a single DC motor.
     * 
     */
    class DCMotor
    {
    private:
        /**
         * @brief Create a DCMotor object, un-initialized! You should never call this,
         * instead have the {@link Adafruit_MotorShield} give you a DCMotor object
         * with {@link Adafruit_MotorShield.getMotor}
         * 
         */
        DCMotor(void);

    public:
        friend class MotorShield; ///< Let MotorShield create DCMotors

        /**
         * @brief Control the DC Motor direction and action.
         * 
         * @param cmd The action to perform, can be FORWARD, BACKWARD or RELEASE.
         */
        void run(MotorDir cmd);

        /**
         * @brief Control the DC Motor speed/throttle.
         * 
         * @param speed The 8-bit PWM value, 0 is off, 255 is on.
         */
        void setSpeed(uint8_t speed);

        /**
         * @brief Control the DC Motor speed/throttle at 12 bit resolution.
         * 
         * @param speed The 12-bit PWM value, 0 (full off) to 4095 (full on).
         */
        void setSpeedFine(uint16_t speed);

        /**
         * @brief Turn the motor off completely.
         * 
         */
        void fullOff(void);

        /**
         * @brief Turn the motor on at full speed.
         * 
         */
        void fullOn(void);

    private:
        uint8_t PWMpin, IN1pin, IN2pin;
        MotorShield *MC;
        bool initd;
    };

    /**
     * @brief Object that controls and keeps state for a single stepper motor.
     * 
     */
    class StepperMotor
    {
    private:
        /**
         * @brief Create a StepperMotor object, un-initialized!
         * You should never call this, instead have the {@link Adafruit_MotorShield}
         * give you a StepperMotor object with {@link Adafruit_MotorShield.getStepper}.
         * 
         */
        StepperMotor(void);

    public:
        /**
         * @brief Set the delay for the Stepper Motor speed in RPM.
         * 
         * @param rpm The desired RPM, it is not guaranteed to be achieved.
         */
        void setSpeed(uint16_t rpm);

        /**
         * @brief Move the stepper motor with the given RPM speed,
         * don't forget to call {@link Adafruit_StepperMotor.setSpeed} to set the speed!
         * 
         * @param steps Number of steps to move.
         * @param dir The direction of movement, can be FORWARD or BACKWARD.
         * @param style Stepping style, can be SINGLE, DOUBLE, INTERLEAVE or MICROSTEP.
         */
        void step(uint16_t steps, MotorDir dir, MotorStyle style = SINGLE);

        /**
         * @brief Move the stepper motor by one step. No delays implemented.
         * 
         * @param dir The direction of movement, can be FORWARD or BACKWARD.
         * @param style Stepping style, can be SINGLE, DOUBLE, INTERLEAVE or MICROSTEP.
         * @return uint8_t The current step/microstep index, useful for 
         * Adafruit_StepperMotor.step to keep track of the current 
         * location, especially when microstepping.
         */
        uint8_t onestep(MotorDir dir, MotorStyle style);

        /**
         * @brief Set microsteps per step.
         * 
         * @param microsteps MicroSteps::STEP8 or MicroSteps::STEP16.
         */
        void setStep(MicroSteps microsteps);

        /**
         * @brief Release all pins of the stepper motor so it free-spins.
         * 
         */
        void release(void);

        friend class MotorShield; ///< Let MotorShield create StepperMotors

    private:
        uint32_t usperstep;

        std::mutex cs;
        MicroSteps microsteps;
        uint8_t *microstepcurve;
        uint8_t PWMApin, AIN1pin, AIN2pin;
        uint8_t PWMBpin, BIN1pin, BIN2pin;
        uint16_t revsteps; // # steps per revolution
        uint8_t currentstep;
        MotorShield *MC;
        bool initd;
    };

    /**
     * @brief Object to control and maintain state for the entire motor shield.
     * Use this class to create DC and Stepper motor objects.
     * 
     */
    class MotorShield
    {
    public:
        /**
         * @brief Create the Motor Shield object at an I2C address (default: 0x60) on an I2C bus (default: 1).
         * 
         * @param addr Optional, default: 0x60
         * @param bus Optional, default: 1
         */
        MotorShield(uint8_t addr = 0x60, int bus = 1);

        /**
         * @brief Release all motors and the I2C Bus.
         * 
         */
        ~MotorShield();

        /**
         * @brief Initialize the I2C hardware and PWM driver, then turn off all pins.
         * 
         * @param freq The PWM frequency for the driver, used for speed control and microstepping. 
         * By default we use 1600 Hz which is a little audible but efficient.
         * @return bool true on success, false on failure
         */
        bool begin(uint16_t freq = 1600);

        /**
         * @brief Mini factory that will return a pointer to an already-allocated
         * Adafruit_DCMotor object. Initializes the DC motor and turns off all pins.
         * 
         * @param n The DC motor port we want to use: 1 thru 4 are valid
         * @return Adafruit_DCMotor* NULL on error, valid pointer on success
         */
        DCMotor *getMotor(uint8_t n);

        /**
         * @brief  Mini factory that will return a pointer to an already-allocated 
         * Adafruit_StepperMotor object with a given 'steps per rotation. 
         * Then initializes the stepper motor and turns off all pins.
         * 
         * @param steps How many steps per revolution (used for RPM calculation)
         * @param port The stepper motor port we want to use: only 1 or 2 are valid 
         * @return Adafruit_StepperMotor* NULL on error, valid pointer on success
         */
        StepperMotor *getStepper(uint16_t steps, uint8_t port, MicroSteps microsteps = STEP16);

        friend class DCMotor; ///< Let DCMotors control the Shield

        /**
         * @brief Helper that sets the PWM output on a pin and manages 'all on or off'.
         * 
         * @param pin The PWM output on the driver that we want to control (0-15)
         * @param val The 12-bit PWM value we want to set (0-4095) - 4096 is a special 'all on' value.
         * @return bool true on success, false on failure
         */
        bool setPWM(uint8_t pin, uint16_t val);

        /**
         * @brief Helper that sets the PWM output on a pin as if it were a GPIO.
         * 
         * @param pin The PWM output on the driver that we want to control (0-15).
         * @param val HIGH or LOW for pin setting.
         * @return bool true on success, false on failure
         */
        bool setPin(uint8_t pin, bool val);

    private:
        bool initd;
        uint8_t _addr;
        int _bus;
        uint16_t _freq;
        DCMotor dcmotors[4];
        StepperMotor steppers[2];
        i2cbus bus[1];
        bool reset();
        bool setPWMFreq(float freq);
        bool setPWM(uint8_t num, uint16_t on, uint16_t off);
        uint8_t read8(uint8_t addr);
        bool write8(uint8_t addr, uint8_t d);
    };
};

#endif