## v3.0.0 (2022-08-09)
1. Signal handler now executes the handler for SIGINT (and SIGHUP + SIGPIPE) registered prior to calling Adafruit::MotorShield().
2. Exported library signal handler for external use (discouraged). 

## v2.2.0 (2022-05-13)
### Release Highlights
1. Added support for a callback function to be executed after each (micro)step.

## v2.1.0 (2022-04-25)
### Release Highlights
1. Enabled better handling of signals in the library.
2. Fixed a bug where `SIGINT` would cause a stopped motor to not restart.
3. Enabled support for disabling internal `SIGINT` handling, and enabling
`SIGHUP` and/or `SIGPIPE` handling through compile-time flags.

No changes have been made to the API in this release. The minor version
number has been bumped up to indicate the internal overhaul.

## v2.0.0 (2022-03-24)
### Release Highlights
1. Removed `StepperMotorA` class and associated `LimitSW` class.
2. Extended `StepperMotor` API by adding non-blocking stepping support.
3. Added helper functions to stop stepping and indicate motor stepping.
4. Added a function to get full step time periods for stepping a motor using the `onestep` function.

## v1.1.0 (2022-01-30)
### Release Highlights
1. `StepperMotorA` class with up to three limit switches and homing functionality.
