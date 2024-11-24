#include <webots/robot.h>
#include <webots/light_sensor.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/led.h> // Include header for the buzzer
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

// Define constants
#define TIME_STEP 64         // Time step for the simulation
#define MAX_SPEED 6.28       // Maximum speed of the motors
#define MAX_READINGS 7      // Maximum number of dead ends to record
#define THRESHOLD 100        // Threshold for detecting walls using proximity sensors

int main(int argc, char **argv) {
  // Initialize the Webots robot
  wb_robot_init();

  // Initialize motors
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY); // Set motor to velocity mode
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0); // Start with stopped motors
  wb_motor_set_velocity(right_motor, 0.0);

  // Initialize proximity sensors
  WbDeviceTag ps[8]; // Array to hold proximity sensor tags
  for (int i = 0; i < 8; i++) {
    char sensor_name[4];
    sprintf(sensor_name, "ps%d", i); // Generate sensor names (ps0, ps1, ...)
    ps[i] = wb_robot_get_device(sensor_name);
    wb_distance_sensor_enable(ps[i], TIME_STEP); // Enable each sensor
  }

  // Initialize light sensors
  WbDeviceTag ls[8]; // Array to hold light sensor tags
  for (int i = 0; i < 8; i++) {
    char sensor_name[4];
    sprintf(sensor_name, "ls%d", i); // Generate sensor names (ls0, ls1, ...)
    ls[i] = wb_robot_get_device(sensor_name);
    wb_light_sensor_enable(ls[i], TIME_STEP); // Enable each sensor
  }

  // Initialize the buzzer
  WbDeviceTag buzzer = wb_robot_get_device("led");
  wb_led_set(buzzer, 0); // Ensure the LED is initially off

  // Arrays to store light sensor readings and averages at dead ends
  double light_readings[MAX_READINGS][8] = {0};
  double average_light_intensity[MAX_READINGS] = {0};
  int dead_end_index = 0; // Counter for dead ends detected

  // Set initial motor speeds
  double left_motor_speed = MAX_SPEED;
  double right_motor_speed = MAX_SPEED;

  // Detect and handle dead ends, and control robot behavior
  while (dead_end_index < MAX_READINGS && wb_robot_step(TIME_STEP) != -1) {
    double ps_values[8], ls_values[8];
    for (int i = 0; i < 8; i++) {
      ps_values[i] = wb_distance_sensor_get_value(ps[i]);
      ls_values[i] = wb_light_sensor_get_value(ls[i]);
    }

    // Detect walls using proximity sensors
    bool left_wall = ps_values[5] > THRESHOLD; // Wall on the left
    bool front_wall = ps_values[7] > THRESHOLD; // Wall in front

    // Detect dead-end condition
    bool is_dead_end = ps_values[1] > THRESHOLD && // Wall mid-left
                       ps_values[5] > THRESHOLD && // Wall mid-right
                       ps_values[7] > THRESHOLD;  // Wall in front

    if (is_dead_end) {
      // Dead end detected
      printf("\nDead-end detected at index %d.\n", dead_end_index + 1);

      // Store light sensor readings and calculate average intensity
      double ls_sum = 0.0;
      for (int i = 0; i < 8; i++) {
        light_readings[dead_end_index][i] = ls_values[i];
        ls_sum += ls_values[i];
      }
      average_light_intensity[dead_end_index] = ls_sum / 8;
      printf("Average Light Intensity at Dead End %d: %.2f\n",
             dead_end_index + 1, average_light_intensity[dead_end_index]);

      // Increment dead end index
      dead_end_index++;

      // Turn around at the dead end
      wb_motor_set_velocity(left_motor, MAX_SPEED);
      wb_motor_set_velocity(right_motor, -MAX_SPEED);
      wb_robot_step(1000); // Wait for the turn to complete
    } else if (front_wall) {
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

    // Set motor speeds
    wb_motor_set_velocity(left_motor, left_motor_speed);
    wb_motor_set_velocity(right_motor, right_motor_speed);
  }

  // Main control loop to navigate to the brightest dead end
  while (wb_robot_step(TIME_STEP) != -1) {
    if (dead_end_index >= 7) {
      // Find the dead end with the maximum average light intensity
      double max_average_intensity = average_light_intensity[0];
      int target_dead_end_index = 0;
      for (int i = 1; i < dead_end_index; i++) {
        if (average_light_intensity[i] > max_average_intensity) {
          max_average_intensity = average_light_intensity[i];
          target_dead_end_index = i;
        }
      }

      // Output the result
      printf("\nAfter scanning through all the dead ends, the maximum average light intensity is %.2f at Dead End %d.\n",
             max_average_intensity, target_dead_end_index + 1);

      // Navigate to the brightest dead end
      printf("\nNavigating to Dead End %d...\n", target_dead_end_index + 1);

      bool reached_target = false;
      while (!reached_target && wb_robot_step(TIME_STEP) != -1) {
        // Read proximity and light sensor values
        double ps_values[8], ls_values[8];
        for (int i = 0; i < 8; i++) {
          ps_values[i] = wb_distance_sensor_get_value(ps[i]);
          ls_values[i] = wb_light_sensor_get_value(ls[i]);
        }

        // Detect walls using proximity sensors
        bool left_wall = ps_values[5] > THRESHOLD;
        bool front_wall = ps_values[7] > THRESHOLD;

        // Left-wall-following logic
        if (front_wall) {
          left_motor_speed = MAX_SPEED;
          right_motor_speed = -MAX_SPEED;
        } else if (left_wall) {
          left_motor_speed = MAX_SPEED;
          right_motor_speed = MAX_SPEED;
        } else {
          left_motor_speed = MAX_SPEED / 8;
          right_motor_speed = MAX_SPEED;
        }

        // Set motor speeds
        wb_motor_set_velocity(left_motor, left_motor_speed);
        wb_motor_set_velocity(right_motor, right_motor_speed);

        // Calculate current average light intensity
        double ls_sum = 0.0;
        for (int i = 0; i < 8; i++) {
          ls_sum += ls_values[i];
        }
        double current_average_intensity = ls_sum / 8;

        // Check if current intensity matches the target
        if (fabs(current_average_intensity - max_average_intensity) < 5.0) {
          reached_target = true;
          printf("\nReached Dead End %d with Maximum Average Light Intensity: %.2f\n",
                 target_dead_end_index + 1, current_average_intensity);

          // Activate the LED
          wb_led_set(buzzer, 1);
          printf("\nLED activated!\n");

          // Stop the robot
          wb_motor_set_velocity(left_motor, 0.0);
          wb_motor_set_velocity(right_motor, 0.0);
        }
      }

      // End the simulation
      wb_robot_cleanup();
      return 0;
    }
  }

  // Cleanup the Webots robot
  wb_robot_cleanup();
  return 0;
}