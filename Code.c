#include <webots/robot.h>
#include <webots/light_sensor.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <stdio.h>
#include <stdbool.h>

#define TIME_STEP 64
#define MAX_SPEED 6.28
#define MAX_READINGS 10 // Max number of dead ends to record
#define THRESHOLD 100

int main(int argc, char **argv) {
  wb_robot_init();

  // Initialize motors
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  // Initialize distance sensors
  WbDeviceTag ps[8];
  for (int i = 0; i < 8; i++) {
    char sensor_name[4];
    sprintf(sensor_name, "ps%d", i);
    ps[i] = wb_robot_get_device(sensor_name);
    wb_distance_sensor_enable(ps[i], TIME_STEP);
  }

  // Initialize light sensors
  WbDeviceTag ls[8];
  for (int i = 0; i < 8; i++) {
    char sensor_name[4];
    sprintf(sensor_name, "ls%d", i);
    ls[i] = wb_robot_get_device(sensor_name);
    wb_light_sensor_enable(ls[i], TIME_STEP);
  }

  double left_motor_speed = MAX_SPEED;
  double right_motor_speed = MAX_SPEED;

  while (wb_robot_step(TIME_STEP) != -1) {
    
    // Read proximity sensor values
    double ps_values[8];
    for (int i = 0; i < 8; i++) {
      ps_values[i] = wb_distance_sensor_get_value(ps[i]);
    }

    // Read light sensor values
    double ls_values[8];
    for (int i = 0; i < 8; i++) {
      ls_values[i] = wb_light_sensor_get_value(ls[i]);
    }

// Turn around at the dead end
      wb_motor_set_velocity(left_motor, MAX_SPEED);
      wb_motor_set_velocity(right_motor, -MAX_SPEED);
      wb_robot_step(1000); // Wait to complete turn
     else if (front_wall) {
      // Turn right if there's a wall in front
      left_motor_speed = MAX_SPEED;
      right_motor_speed = -MAX_SPEED;
    } else if (left_wall) {
      // Move straight if there's a wall on the left
      left_motor_speed = MAX_SPEED;
      right_motor_speed = MAX_SPEED;
    } else {
      // Turn left if no walls are detected
      left_motor_speed = MAX_SPEED / 8;
      right_motor_speed = MAX_SPEED;
    }
    // Set motor velocities
    wb_motor_set_velocity(left_motor, left_motor_speed);
    wb_motor_set_velocity(right_motor, right_motor_speed);
  }

  wb_robot_cleanup();
  return 0;
  }
