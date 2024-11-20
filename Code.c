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

        // Check if the robot has reached 6 dead ends
    if (dead_end_index >= 7) {
      // Find the dead end with the maximum light intensity
      double maximum_light_intensity = average_light_intensity[0];
      int max_light_index = 0;
      for (int i = 1; i < dead_end_index; i++) {
        if (average_light_intensity[i] > maximum_light_intensity) {
          maximum_light_intensity = average_light_intensity[i];
          max_light_index = i;
        }
      }
            // Print the result
      printf("After reaching 7 dead ends, the dead end with the maximum light intensity is Dead End %d.\n", max_light_index + 1);
      printf("Maximum Light Intensity: %.2f\n", maximum_light_intensity);

      // Perform full 360-degree turn
      printf("Turning 180 degrees before stopping.\n");

          double turn_speed = MAX_SPEED / 4; // Slower turn
      wb_motor_set_velocity(left_motor, turn_speed);
      wb_motor_set_velocity(right_motor, -turn_speed);
      wb_robot_step(2001); // Wait to complete full turn

      // Stop the robot after reaching 6 dead ends
      printf("6 Dead ends reached. Stopping the robot.\n");
      wb_motor_set_velocity(left_motor, 0.0); // Stop left motor
      wb_motor_set_velocity(right_motor, 0.0); // Stop right motor
      break; // Exit the loop and stop the robot
    }

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

    // Detect walls using sensors 0-4 and 7
    bool left_wall = ps_values[5] > THRESHOLD; // Left wall
    bool front_wall = ps_values[7] > THRESHOLD; // Front wall

    // New dead-end condition
    bool is_dead_end = 
                       ps_values[1] > THRESHOLD && // Mid left
                       ps_values[5] > THRESHOLD && // Mid right
                       ps_values[7] > THRESHOLD;  // Front
    if (is_dead_end && dead_end_index < MAX_READINGS) {
      // Dead-end detected
      printf("Dead-end detected at index %d.\n", dead_end_index + 1);

            // Store light sensor values
      double ls_sum = 0.0;
      for (int i = 0; i < 8; i++) {
        light_readings[dead_end_index][i] = ls_values[i];
        ls_sum += ls_values[i];
      }

      // Calculate and store the average light intensity
      average_light_intensity[dead_end_index] = ls_sum / 8;
      printf("Average Light Intensity at Dead End %d: %.2f\n", dead_end_index + 1, average_light_intensity[dead_end_index]);

      // Increment the dead-end index
      dead_end_index++;

      // Find the maximum light intensity from recorded averages
      double maximum_light_intensity = average_light_intensity[0];
      for (int i = 1; i < dead_end_index; i++) {
        if (average_light_intensity[i] > maximum_light_intensity) {
          maximum_light_intensity = average_light_intensity[i];
        }
      }

      // Print the maximum light intensity found so far
      printf("Maximum Light Intensity Recorded So Far: %.2f\n", maximum_light_intensity);



// Turn around at the dead end
      wb_motor_set_velocity(left_motor, MAX_SPEED);
      wb_motor_set_velocity(right_motor, -MAX_SPEED);
      wb_robot_step(1000); // Wait to complete turn
     }else if (front_wall) {
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
