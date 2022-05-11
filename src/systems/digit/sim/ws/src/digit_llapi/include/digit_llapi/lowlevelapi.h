/*
 * Copyright 2020 Agility Robotics
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NUM_MOTORS 20
#define NUM_JOINTS 10


// Initializes the low-level api interface. Only call one of these functions.
// llapi_init() uses default values for listen/send ports (25501/25500), while
// llapi_init_custom() allows specifying custom ports
//    pub_addr: a string specifying the ip address of the robot control computer
//              e.g. "10.42.0.1"
//    listen_port: the port on which to listen for llapi observations
//    send_port: the port on which to send llapi commands
void llapi_init(const char* pub_addr);
void llapi_init_custom(const char* pub_addr, int listen_port, int send_port);

// This type explicitly defines the quaternion convention
typedef struct {
  double w;
  double x;
  double y;
  double z;
} llapi_quaternion_t;

// Gets any updates from the artl subscriber and copies it to obs
// Only changes obs when there is new data.
// Returns:
// 1 - New data is available
// 0 - No new data is available, obs unchanged
// -1 - An error has ocurred
typedef struct {
  // All values are in SI units (N-m, rad, rad/s, V). Temperature is in deg C

  // Time since control program started
  double time;

  // Robot status flag. Monitor the JSON API for details if flag is set to true
  // If false: robot is operating normally
  // If true:  An issue occurred that causes the system to have reduced
  //           performance, or a fatal issue may occur unless the user does
  //           something. The system will automatically disable the Low-level
  //           API after a brief delay. This delay can be read by calling
  //           llapi_get_error_shutdown_delay().
  bool error;

  // Estimated pose and velocity of base frame
  struct {
    double translation[3];
    llapi_quaternion_t orientation;
    double linear_velocity[3];
    double angular_velocity[3];
  } base;

  // Raw sensor signals from IMU
  struct {
    llapi_quaternion_t orientation;
    double angular_velocity[3];
    double linear_acceleration[3];
    double magnetic_field[3];
  } imu;

  // Actuated joints
  struct {
    double position[NUM_MOTORS];
    double velocity[NUM_MOTORS];
    double torque[NUM_MOTORS];
  } motor;

  // Unactuated joints
  struct {
    double position[NUM_JOINTS];
    double velocity[NUM_JOINTS];
  } joint;

  // Expressed as percent (0-100)
  int16_t battery_charge;

} llapi_observation_t;
int llapi_get_observation(llapi_observation_t* obs);


// Returns a pointer to a struct describing the maximum torque/damping/velocity
// command values for each actuated joint. Any commanded values with a magnitude
// greater than these limits will be clamped by the robot before being applied.
// The damping limit for each joint is set such that the maximum value is still
// stable, while a value of roughly 20% of the limit is a reasonable value in
// most situations. These values are obtained when the connection to the robot
// or simulator is established, and calling the function beforehand returns a
// null pointer.
typedef struct {
  double torque_limit[NUM_MOTORS];
  double damping_limit[NUM_MOTORS];
  double velocity_limit[NUM_MOTORS];
} llapi_limits_t;
const llapi_limits_t* llapi_get_limits();


// Returns the delay in seconds after receiving an Error status from the robot
// and disabling the Low-level API.
double llapi_get_error_shutdown_delay();


// Sends command to robot
// The motor drivers can track a commanded velocity with the specified damping
// (optional), and/or be controlled directly via the feed-forward torque
// command. Damping is in N-m / (rad/s).
typedef struct __attribute__((packed)) {
  double torque;
  double velocity;
  double damping;
} llapi_motor_t;
typedef struct __attribute__((packed)) {
  llapi_motor_t motors[NUM_MOTORS];
  int32_t fallback_opmode;
  bool apply_command;
} llapi_command_t;
void llapi_send_command(llapi_command_t* cmd);


// Returns true if the llapi subscriber is connected to the robot
bool llapi_connected();


// Values from this enum can be used in place of numeric indices when indexing
// the motor fields in the observation and command arrays.
enum DigitMotors {
  LeftHipRoll,
  LeftHipYaw,
  LeftHipPitch,
  LeftKnee,
  LeftToeA,
  LeftToeB,

  RightHipRoll,
  RightHipYaw,
  RightHipPitch,
  RightKnee,
  RightToeA,
  RightToeB,

  LeftShoulderRoll,
  LeftShoulderPitch,
  LeftShoulderYaw,
  LeftElbow,

  RightShoulderRoll,
  RightShoulderPitch,
  RightShoulderYaw,
  RightElbow,
};

// Values from this enum can be used in place of numeric indices when indexing
// the joint fields in the observation and command arrays.
enum DigitJoints {
  LeftShin,
  LeftTarsus,
  LeftToePitch,
  LeftToeRoll,
  LeftHeelSpring,

  RightShin,
  RightTarsus,
  RightToePitch,
  RightToeRoll,
  RightHeelSpring,
};

// Values in this enum should be used to set the fallback opmode (the
// operation mode that the robot will enter if the user program exits
// unexpectedly)
// When writing custom code, make sure that the numbers for the fallback opmode
// entries stay consistent with this enum
enum LLAPIOpMode {
  Disabled = 0,
  Damping = 1,
  Locomotion = 2,
};


#ifdef __cplusplus
} // extern "C"
#endif
