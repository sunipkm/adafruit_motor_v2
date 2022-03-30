/**
 * @file MotorShield.hpp
 * @author Sunip K. Mukherjee (sunipkmukherjee@gmail.com), based on work by Limor Fried/Ladyada for Adafruit Industries.
 * @brief This is the library for the Adafruit Motor Shield V2 for Arduino.
 * It supports DC motors & Stepper motors with microstepping as well
 * as stacking-support. It is *not* compatible with the V1 library.
 * @version 2.0.0
 * @date 2022-03-24
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

#include <unistd.h>
#include <stdint.h>
#include <signal.h>
#include "i2cbus/i2cbus.h"
#include "clkgen.h"

#include <mutex>
#include <condition_variable>

namespace Adafruit
{
#if !defined(ADAFRUIT_MOTORSHIELD_DEBUG)
/**
 * @brief Enable debug printouts for motor shield.
 *
 */
#define ADAFRUIT_MOTORSHIELD_DEBUG 0
#ifndef _DOXYGEN_
#ifndef MEB_DBGLVL
#define MEB_DBGLVL 0
#endif // _DOXYGEN_
#endif // MEB_DBGLVL
#elif ADAFRUIT_MOTORSHIELD_DEBUG > 0
#define MEB_DBGLVL MEB_DBG_ALL
#endif // ADAFURUIT_MOTORSHIELD_DEBUG

/**
 * @brief Indicates the function throws exceptions
 * 
 */
#define _Catchable

    /**
     * @brief Defines the stepping technique used to actuate stepper motors.
     *
     * @var SINGLE
     * Single coil stepping
     * @var DOUBLE
     * Double coil stepping
     * @var INTERLEAVE
     * Double coil interleaved stepping
     * @var MICROSTEP
     * Microstepping, achieves a smoother motion by dividing a step into smaller 'micro'steps.
     */
    typedef enum : uint8_t
    {
        SINGLE = 1,
        DOUBLE = 2,
        INTERLEAVE = 3,
        MICROSTEP = 4
    } MotorStyle;

    /**
     * @brief Defines the direction of motor actuation.
     *
     */
    typedef enum : uint8_t
    {
        FORWARD = 1,  /*!< Forward direction */
        BACKWARD = 2, /*!< Backward direction */
        BRAKE = 3,    /*!< Not used */
        RELEASE = 4   /*!< Release the motor. */
        /*!< In case of DC motor, stops running. */
        /*!< In case of stepper motor, removes stall torque and powers down the coils. */
    } MotorDir;

    /**
     * Defines the number of microsteps executed per step
     * of a stepper motor. Increasing microsteps per step limits
     * the maximum RPM achievable by a stepper motor due to I2C bus
     * constraints. Upper limits for each microstep for a 200 steps/revolution,
     * double coil stepper motor are provided.
     *
     */
    typedef enum : uint16_t
    {
        STEP8 = 8,     /*!< 8 microsteps per step, max speed 10 RPM. */
        STEP16 = 16,   /*!< 16 microsteps per step, max speed 5 RPM. */
        STEP32 = 32,   /*!< 32 microsteps per step, max speed 2.5 RPM. */
        STEP64 = 64,   /*!< 64 microsteps per step, max speed 1.25 RPM. */
        STEP128 = 128, /*!< 128 microsteps per step, max speed 0.625 RPM. */
        STEP256 = 256, /*!< 256 microsteps per step, max speed 0.3125 RPM. */
        STEP512 = 512  /*!< 512 microsteps per step, max speed 0.15625 RPM. */
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

#ifndef _DOXYGEN_
    class StepperMotor;
    struct StepperMotorTimerData
    {
        StepperMotor *_this;
        uint16_t steps;
        MotorDir dir;
        MotorStyle style;
        MicroSteps msteps;
    };

    struct StepperMotorDestroyClkData
    {
        StepperMotor *_this;
        clkgen_t clk;
    };
#endif

    /**
     * @brief Object that controls and keeps state for a single stepper motor.
     *
     */
    class StepperMotor
    {
    private:
        static void stepHandlerFn(clkgen_t clk, void *data_);
        static void stepThreadFn(StepperMotor *mot, uint16_t steps, MotorDir dir, MotorStyle style);

    protected:
        /**
         * @brief Create an uninitialized StepperMotor object.
         *
         */
        StepperMotor(void);

    public:
        /**
         * @brief Set the delay for the Stepper Motor speed in RPM.
         * Throws exception in case rpm <= 0.
         *
         * @param rpm The desired RPM, it is not guaranteed to be achieved. In double coil mode upto ~68 RPM is achieved for a 200 steps/rev stepper, in microstep mode ~1.25 RPM is achieved for a 200 steps/rev stepper at STEP64 setting, ~0.3125 RPM at STEP256 setting.
         *
         * @return bool true on success, false on failure.
         */
        bool _Catchable setSpeed(double rpm);

        /**
         * @brief Move the stepper motor with the given RPM speed,
         * at the speed set using {@link Adafruit::StepperMotor::setSpeed}. Throws exception if RPM was not set prior to call.
         *
         * @param steps Number of steps to move.
         * @param dir The direction of movement, can be FORWARD or BACKWARD.
         * @param style Stepping style, can be SINGLE, DOUBLE, INTERLEAVE or MICROSTEP. SINGLE by default.
         * @param blocking Whether the step function blocks until stepping is complete. Set to true by default.
         */
        void _Catchable step(uint16_t steps, MotorDir dir, MotorStyle style = SINGLE, bool blocking = true);

        /**
         * @brief Move the stepper motor by one step. No delays implemented.
         * Care must be taken while using onestep, especially regarding stopping
         * at a non-integral step while microstepping. Use this function in
         * conjunction with {@link Adafruit::StepperMotor::getStepPeriod} function
         * that gives the time (in microseconds) required to execute a full step.
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
         * @param microsteps {@link Adafruit::MicroSteps} members.
         *
         * @return bool true on success, false on failure.
         */
        bool setStep(MicroSteps microsteps);

        /**
         * @brief Release all pins of the stepper motor so it free-spins.
         *
         */
        void release(void);

        /**
         * @brief Check if the motor is stepping.
         *
         * @return bool True if stepping, false otherwise.
         */
        bool isMoving() const;

        /**
         * @brief Stop stepping the motor.
         *
         */
        void stopMotor();

        /**
         * @brief Get the time period of each full step.
         * The time period is useful in case of onestepping/manual stepping. Throws exception if RPM was not set prior to call.
         *
         * @return uint64_t Time period of a full step
         */
        uint64_t _Catchable getStepPeriod() const;

        friend class MotorShield; ///< Let MotorShield create StepperMotors

    protected:
        uint64_t usperstep;
        MicroSteps microsteps;

    private:
        std::mutex cs;
        std::condition_variable cond;
        uint16_t *microstepcurve;
        uint8_t PWMApin, AIN1pin, AIN2pin;
        uint8_t PWMBpin, BIN1pin, BIN2pin;
        uint16_t revsteps; // # steps per revolution
        uint16_t currentstep;
        MotorShield *MC;
        bool initd;
        volatile sig_atomic_t *done;
        volatile bool moving;
        volatile bool stop;
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
         * By default 1600 Hz is used, which is a little audible but efficient.
         * @return bool true on success, false on failure
         */
        bool _Catchable begin(uint16_t freq = 1600);

        /**
         * @brief Returns a pointer to an already-allocated
         * {@link Adafruit::DCMotor} object. Initializes the DC motor and turns off all pins.
         *
         * @param n The DC motor port to be used: 1 thru 4 are valid
         * @return Adafruit::DCMotor* NULL on error, valid pointer on success
         */
        DCMotor *getMotor(uint8_t n);

        /**
         * @brief  Returns a pointer to an already-allocated
         * {@link Adafruit::StepperMotor} object with a given steps per rotation.
         * Initializes the stepper motor and turns off all pins.
         *
         * @param steps How many steps per revolution (used for RPM calculation).
         * @param port The stepper motor port to be used: only 1 or 2 are valid.
         * @param microsteps Number of microsteps per step to use.
         * @return Adafruit::StepperMotor* NULL on error, valid pointer on success.
         */
        StepperMotor *getStepper(uint16_t steps, uint8_t port, MicroSteps microsteps = STEP16);

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
        uint8_t _Catchable read8(uint8_t addr);
        bool write8(uint8_t addr, uint8_t d);
    };
};

#endif