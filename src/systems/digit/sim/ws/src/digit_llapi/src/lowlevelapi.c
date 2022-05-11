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

#include "lowlevelapi.h"
#include "libartl/artl.h"
#include <stdio.h>
#include <string.h>

// List of names and their indices in the command and observation structs
// This is used in the "X" Macros to concisely generate field mapping code
#define MOTOR_NAMES           \
  X(left_hip_roll, 0)         \
  X(left_hip_yaw, 1)          \
  X(left_hip_pitch, 2)        \
  X(left_knee, 3)             \
  X(left_toe_A, 4)            \
  X(left_toe_B, 5)            \
  X(right_hip_roll, 6)        \
  X(right_hip_yaw, 7)         \
  X(right_hip_pitch, 8)       \
  X(right_knee, 9)            \
  X(right_toe_A, 10)          \
  X(right_toe_B, 11)          \
  X(left_shoulder_roll, 12)   \
  X(left_shoulder_pitch, 13)  \
  X(left_shoulder_yaw, 14)    \
  X(left_elbow, 15)           \
  X(right_shoulder_roll, 16)  \
  X(right_shoulder_pitch, 17) \
  X(right_shoulder_yaw, 18)   \
  X(right_elbow, 19)          \

#define JOINT_NAMES           \
  X(left_shin, 0)             \
  X(left_tarsus, 1)           \
  X(left_toe_pitch, 2)        \
  X(left_toe_roll, 3)         \
  X(left_heel_spring, 4)      \
  X(right_shin, 5)            \
  X(right_tarsus, 6)          \
  X(right_toe_pitch, 7)       \
  X(right_toe_roll, 8)        \
  X(right_heel_spring, 9)     \


// Local variables for storing communication objects
static llapi_limits_t command_limits;
static double error_shutdown_delay = -1;
static artl_publisher_t* pub;
static artl_subscriber_t* sub;


// This mapping function is called every time the local subscriber connects to
// a new publisher. Using the description of the incoming message stream it
// looks for expected fields and maps them to their respective locations in the
// command struct. This enables llapi_command_t objects to be passed to the
// artl_subscriber_update function and have the incoming data stored in their
// previously mapped locations
static artl_map_t* llapi_map_fcn(const artl_description_t* desc, void* arg)
{
  artl_map_t* map = artl_map_init(desc);
  artl_map_t* comment_map = artl_map_init(desc->comment);

  // Map time
  if (artl_check_field(desc, "time", ARTL_FLOAT, 1, 1))
    artl_map_field(map, "time", offsetof(llapi_observation_t, time), ARTL_F64);

  // Map status
  if (artl_check_field(desc, "error", ARTL_BOOL, 1, 1))
    artl_map_field(map, "error",
      offsetof(llapi_observation_t, error), ARTL_BOOL);

  // Map shutdown delay
  if (artl_check_field(desc->comment, "error_shutdown_delay", ARTL_FLOAT, 1, 1))
  {
    artl_map_field_abs(comment_map, "error_shutdown_delay",
      &error_shutdown_delay, ARTL_F64);
  }

  // X macro: iterates over the previously defined list specified on the last
  // line (here it is MOTOR_NAMES), and copies the code between
  // #define X(name, idx) and #undef for each entry in the list. "name" and
  // "idx" are replaced by their respective values in the list.
  //
  // This is used here to generate the repetitive code used to validate each
  // field in the incoming message and map it to the observation struct.
#define X(name, idx)                                                  \
  if (artl_check_field(desc->comment, #name ".torque_limit",          \
                       ARTL_F64, 1, 1)) {                             \
    artl_map_field_abs(comment_map, #name ".torque_limit",            \
      &command_limits.torque_limit[idx], ARTL_F64);                   \
  } else { command_limits.torque_limit[idx] = 0; }                    \
                                                                      \
  if (artl_check_field(desc->comment, #name ".damping_limit",         \
                       ARTL_F64, 1, 1)) {                             \
    artl_map_field_abs(comment_map, #name ".damping_limit",           \
      &command_limits.damping_limit[idx], ARTL_F64);                  \
  } else { command_limits.damping_limit[idx] = 0; }                   \
                                                                      \
  if (artl_check_field(desc->comment, #name ".velocity_limit",        \
                       ARTL_F64, 1, 1)) {                             \
    artl_map_field_abs(comment_map, #name ".velocity_limit",          \
      &command_limits.velocity_limit[idx], ARTL_F64);                 \
  } else { command_limits.velocity_limit[idx] = 0; }                  \
                                                                      \
  if (artl_check_field(desc, #name ".position",                       \
                       ARTL_FLOAT, 1, 1))                             \
    artl_map_field(map, #name ".position",                            \
      offsetof(llapi_observation_t, motor.position[idx]), ARTL_F64);  \
                                                                      \
  if (artl_check_field(desc, #name ".velocity",                       \
                       ARTL_FLOAT, 1, 1))                             \
    artl_map_field(map, #name ".velocity",                            \
      offsetof(llapi_observation_t, motor.velocity[idx]), ARTL_F64);  \
                                                                      \
  if (artl_check_field(desc, #name ".torque",                         \
                       ARTL_FLOAT, 1, 1))                             \
    artl_map_field(map, #name ".torque",                              \
      offsetof(llapi_observation_t, motor.torque[idx]), ARTL_F64);
  MOTOR_NAMES
#undef X

  // Map joint data
#define X(name, idx)                                                  \
  if (artl_check_field(desc, #name ".position",                       \
                        ARTL_FLOAT, 1, 1))                            \
    artl_map_field(map, #name ".position",                            \
      offsetof(llapi_observation_t, joint.position[idx]), ARTL_F64);  \
                                                                      \
  if (artl_check_field(desc, #name ".velocity",                       \
                        ARTL_FLOAT, 1, 1))                            \
    artl_map_field(map, #name ".velocity",                            \
      offsetof(llapi_observation_t, joint.velocity[idx]), ARTL_F64);
  JOINT_NAMES
#undef X

  // Base frame data
  if (artl_check_field(desc, "base.translation", ARTL_FLOAT, 3, 1))
    artl_map_field(map, "base.translation",
                   offsetof(llapi_observation_t, base.translation), ARTL_F64);
  if (artl_check_field(desc, "base.orientation.w", ARTL_FLOAT, 1, 1))
    artl_map_field(map, "base.orientation.w",
                   offsetof(llapi_observation_t, base.orientation.w), ARTL_F64);
  if (artl_check_field(desc, "base.orientation.x", ARTL_FLOAT, 1, 1))
    artl_map_field(map, "base.orientation.x",
                   offsetof(llapi_observation_t, base.orientation.x), ARTL_F64);
  if (artl_check_field(desc, "base.orientation.y", ARTL_FLOAT, 1, 1))
    artl_map_field(map, "base.orientation.y",
                   offsetof(llapi_observation_t, base.orientation.y), ARTL_F64);
  if (artl_check_field(desc, "base.orientation.z", ARTL_FLOAT, 1, 1))
    artl_map_field(map, "base.orientation.z",
                   offsetof(llapi_observation_t, base.orientation.z), ARTL_F64);
  if (artl_check_field(desc, "base.linear_velocity", ARTL_FLOAT, 3, 1))
    artl_map_field(map, "base.linear_velocity",
                offsetof(llapi_observation_t, base.linear_velocity), ARTL_F64);
  if (artl_check_field(desc, "base.angular_velocity", ARTL_FLOAT, 3, 1))
    artl_map_field(map, "base.angular_velocity",
                offsetof(llapi_observation_t, base.angular_velocity), ARTL_F64);

  // IMU data
  if (artl_check_field(desc, "imu.orientation.w", ARTL_FLOAT, 1, 1))
    artl_map_field(map, "imu.orientation.w",
                   offsetof(llapi_observation_t, imu.orientation.w), ARTL_F64);
  if (artl_check_field(desc, "imu.orientation.x", ARTL_FLOAT, 1, 1))
    artl_map_field(map, "imu.orientation.x",
                   offsetof(llapi_observation_t, imu.orientation.x), ARTL_F64);
  if (artl_check_field(desc, "imu.orientation.y", ARTL_FLOAT, 1, 1))
    artl_map_field(map, "imu.orientation.y",
                   offsetof(llapi_observation_t, imu.orientation.y), ARTL_F64);
  if (artl_check_field(desc, "imu.orientation.z", ARTL_FLOAT, 1, 1))
    artl_map_field(map, "imu.orientation.z",
                   offsetof(llapi_observation_t, imu.orientation.z), ARTL_F64);
  if (artl_check_field(desc, "imu.angular_velocity", ARTL_FLOAT, 3, 1))
    artl_map_field(map, "imu.angular_velocity",
                   offsetof(llapi_observation_t, imu.angular_velocity), ARTL_F64);
  if (artl_check_field(desc, "imu.linear_acceleration", ARTL_FLOAT, 3, 1))
    artl_map_field(map, "imu.linear_acceleration",
                   offsetof(llapi_observation_t, imu.linear_acceleration), ARTL_F64);
  if (artl_check_field(desc, "imu.magnetic_field", ARTL_FLOAT, 3, 1))
    artl_map_field(map, "imu.magnetic_field",
                   offsetof(llapi_observation_t, imu.magnetic_field), ARTL_F64);

  // Battery state of charge
  if (artl_check_field(desc, "battery.state_of_charge", ARTL_INT, 1, 1))
    artl_map_field(map, "battery.state_of_charge",
                   offsetof(llapi_observation_t, battery_charge), ARTL_I16);

  // Extract comment information. All mapping is absolute, so output is empty
  artl_map_get_array(comment_map, NULL, 1, 1, desc->comment_data,
                     desc->comment->message_size);

  return map;
}


void llapi_init(const char* pub_addr)
{
  llapi_init_custom(pub_addr, 25501, 25500);
}


void llapi_init_custom(const char* pub_addr, int listen_port, int send_port)
{
  // Create description for publisher
  artl_description_t* desc = artl_description_init();

  // Add motor fields to the description using an X macro (see above)
#define X(name, idx)                                         \
  artl_add_field(desc, #name ".torque", ARTL_F64, 1, 1);     \
  artl_add_field(desc, #name ".velocity", ARTL_F64, 1, 1);   \
  artl_add_field(desc, #name ".damping", ARTL_F64, 1, 1);
  MOTOR_NAMES
#undef X

  // Add fallback_opmode to description. This specifies the operation mode that
  // the robot will attempt to enter if the user program disconnects
  artl_enum_t* enm = artl_add_enum(desc, ARTL_ANY, ARTL_U32);
  artl_add_label(desc, enm->type, (int) Disabled, "Disabled");
  artl_add_label(desc, enm->type, (int) Damping, "Damping");
  artl_add_label(desc, enm->type, (int) Locomotion, "Locomotion");
  artl_add_field(desc, "fallback_opmode", enm->type, 1, 1);

  // Add apply_command flag to description. This must be true for
  // the commands to be applied
  artl_add_field(desc, "apply_command", ARTL_BOOL, 1, 1);

  // Create subscriber and publisher
  pub = artl_publisher_init(desc, pub_addr, send_port);
  printf("Sending commands to %s on port %d\r\n", pub_addr, send_port);
  sub = artl_subscriber_init("0.0.0.0", listen_port);
  printf("Listening for data from 0.0.0.0 on port %d\r\n", listen_port);
  sub->mapfun = llapi_map_fcn;
  sub->output_buffer_size = sizeof(llapi_observation_t);
  sub->disconnect_timeout_ms = 100;
}


int llapi_get_observation(llapi_observation_t* obs)
{
  return artl_subscriber_update(sub, obs);
}


const llapi_limits_t* llapi_get_limits()
{
  // Update observation first
  llapi_observation_t obs;
  artl_subscriber_update(sub, &obs);
  if (sub->connected)
    return &command_limits;
  else
    return NULL;
}


double llapi_get_error_shutdown_delay()
{
  return error_shutdown_delay;
}


void llapi_send_command(llapi_command_t* cmd)
{
  artl_publisher_update(pub, (void*) cmd);
}


bool llapi_connected()
{
  return sub->connected;
}
