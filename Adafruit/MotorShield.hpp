/**
 * @file MotorShield.hpp
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
#include "gpiodev/gpiodev.h"

#include "clkgen.h"

#include <mutex>
#include <list>

namespace Adafruit
{
#if !defined(ADAFRUIT_MOTORSHIELD_DEBUG)
/**
 * @brief Enable debug printouts for motor shield.
 *
 */
#define ADAFRUIT_MOTORSHIELD_DEBUG 0
#ifndef MEB_DBGLVL
#define MEB_DBGLVL 0
#endif // MEB_DBGLVL
#elif ADAFRUIT_MOTORSHIELD_DEBUG > 0
#define MEB_DBGLVL MEB_DBG_ALL
#endif // ADAFURUIT_MOTORSHIELD_DEBUG
#define MEB_DBGLVL MEB_DBG_TPRINT
#include "meb_print.h"
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
        STEP16 = 16,
        STEP64 = 64
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
         * @brief Create an uninitialized DCMotor object.
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

    class StepperMotor;
    struct StepperMotorTimerData
    {
        StepperMotor *_this;
        uint16_t steps;
        MotorDir dir;
        MotorStyle style;
    };

    /**
     * @brief Object that controls and keeps state for a single stepper motor.
     *
     */
    class StepperMotor
    {
    protected:
        /**
         * @brief Create an uninitialized StepperMotor object.
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
         * don't forget to call {@link Adafruit::StepperMotor::setSpeed} to set the speed!
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

    protected:
        uint32_t usperstep;
        MicroSteps microsteps;

    private:
        std::mutex cs;
        uint8_t *microstepcurve;
        uint8_t PWMApin, AIN1pin, AIN2pin;
        uint8_t PWMBpin, BIN1pin, BIN2pin;
        uint16_t revsteps; // # steps per revolution
        uint8_t currentstep;
        MotorShield *MC;
        bool initd;
        static void stepHandlerFn(clkgen_t clk, void *data_)
        {
            struct StepperMotorTimerData *data = (struct StepperMotorTimerData *)data_;
            StepperMotor *_this = data->_this;
            if (data->steps)
            {
                _this->onestep(data->dir, data->style);
                data->steps--;
            }
        }
    };

    /**
     * @brief Structure describing a limit switch
     *
     */
    class LimitSW
    {
    public:
        int pos;      ///< Limit switch position
        MotorDir dir; ///< Direction from origin to limit switch
        int pin;      ///< GPIO pin
        int active;   ///< active gpio state

        /**
         * @brief Create a new empty limit switch object.
         *
         */
        LimitSW()
        {
            pos = 0;
            dir = MotorDir::BRAKE;
            pin = -1;
            active = GPIO_HIGH;
        }

        /**
         * @brief Create a new limit switch object with properties.
         *
         * @param p Limit SW position.
         * @param d Direction from origin.
         * @param pin GPIO pin.
         * @param state GPIO state when active.
         */
        LimitSW(int p, MotorDir d, int pin, int state = GPIO_HIGH)
        {
            pos = p;
            dir = d;
            pin = pin;
            active = state;
        }

        /**
         * @brief Assignment operator
         *
         * @param rhs Reference to LimitSW object
         */
        void operator=(const LimitSW &rhs)
        {
            pos = rhs.pos;
            dir = rhs.dir;
        }

        /**
         * @brief Comparison operator
         *
         * @param lhs
         * @param rhs
         * @return bool true if lhs.pos < rhs.pos, else false
         */
        friend bool operator<(const LimitSW &lhs,
                              const LimitSW &rhs)
        {
            return lhs.pos < rhs.pos;
        }
    };

    /**
     * @brief Object that controls and keeps state for a single stepper motor, with advanced location tracking and limit switch support.
     *
     */
    class StepperMotorA : public StepperMotor
    {
    private:
        /**
         * @brief Create an uninitialized StepperMotorA object.
         *
         */
        StepperMotorA(void);

    public:
        friend class MotorShield;

        /**
         * @brief Set the state of the stepper motor
         *
         * @param origin Origin of coordinate system
         * @param currentPos Current position of motor
         * @param sw1 Limit switch 1
         * @param sw2 Limit switch 2
         * @param estop Optional: Emergency stop switch
         * @return true on successful init, false on failure
         */
        bool setState(int origin, int currentPos, const LimitSW &sw1, const LimitSW &sw2, const LimitSW &estop);

        /**
         * @brief Return motor to origin
         *
         * @return int Location of origin
         */
        int goHome();

        /**
         * @brief Step motor to destination
         *
         * @param loc Destination
         * @param style MotorStyle
         * @return int End location
         */
        int goToLoc(int loc, MotorStyle style = SINGLE);

        /**
         * @brief Get the origin of the motor
         *
         * @return int
         */
        int getOrigin() const { return origin; }

        /**
         * @brief Get the current position of the motor
         *
         * @return int
         */
        int getCurrentPos() const { return currentPos; }

        /**
         * @brief Stops the motor if stepping.
         *
         */
        void stopMotor() { stopnow = true; }

    private:
        int origin;
        int currentPos;
        LimitSW sw1,
            sw2,
            estop;
        volatile bool stopnow;
        void moveSteps(int steps, MotorDir dir, bool ignoreSW, MotorStyle style);
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

        StepperMotorA *getStepperA(uint16_t steps, uint8_t port, MicroSteps microsteps = STEP16);

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
        StepperMotorA steppers[2];
        i2cbus bus[1];
        bool reset();
        bool setPWMFreq(float freq);
        bool setPWM(uint8_t num, uint16_t on, uint16_t off);
        uint8_t read8(uint8_t addr);
        bool write8(uint8_t addr, uint8_t d);
    };
};

#endif