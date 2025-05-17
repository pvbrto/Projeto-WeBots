#include "robot_controller.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/supervisor.h>

/* Motor device */
static WbDeviceTag left_motor, right_motor;

/* Box node reference */
static WbNodeRef box_node = NULL;

/* E-puck angular speed in rad/s */
#define MAX_SPEED 6.28

/* distance sensor */
#define NUMBER_OF_DISTANCE_SENSORS 8
static WbDeviceTag distance_sensors[NUMBER_OF_DISTANCE_SENSORS];
static const char *distance_sensors_names[NUMBER_OF_DISTANCE_SENSORS] = {
  "ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"
};

#define SENSOR_VALUE_DETECTION_THRESHOLD 500
#define ROBOT_ANGULAR_SPEED_IN_DEGREES 283.588111888

static int TIME_STEP;

void robot_controller_init(int time_step)
{
  TIME_STEP = time_step;

  left_motor = wb_robot_get_device("left wheel motor");
  if (left_motor == 0) {
    printf("Error: Could not get left wheel motor device\n");
    return;
  }
  right_motor = wb_robot_get_device("right wheel motor");
  if (right_motor == 0) {
    printf("Error: Could not get right wheel motor device\n");
    return;
  }
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  // Inicializa o supervisor e pega referência da caixa
  box_node = wb_supervisor_node_get_from_def("CAIXA1");
  if (box_node == NULL) {
    printf("Erro: Não foi possível encontrar o nó CAIXA1\n");
  }

  for (int i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++) {
    distance_sensors[i] = wb_robot_get_device(distance_sensors_names[i]);
    if (distance_sensors[i] == 0) {
      printf("Error: Could not get distance sensor device %s\n", distance_sensors_names[i]);
      continue;
    }
    wb_distance_sensor_enable(distance_sensors[i], TIME_STEP);
  }
}

static float calculate_rotation_time(float degrees)
{
  return fabs(degrees) / ROBOT_ANGULAR_SPEED_IN_DEGREES;
}

void motor_stop() {
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
}

void motor_move_forward() {
  wb_motor_set_velocity(left_motor, MAX_SPEED);
  wb_motor_set_velocity(right_motor, MAX_SPEED);
}

void motor_move_backward() {
  wb_motor_set_velocity(left_motor, -MAX_SPEED);
  wb_motor_set_velocity(right_motor, -MAX_SPEED);
}

void motor_rotate_right() {
  wb_motor_set_velocity(left_motor, MAX_SPEED);
  wb_motor_set_velocity(right_motor, -MAX_SPEED);
}

void motor_rotate_left() {
  wb_motor_set_velocity(left_motor, -MAX_SPEED);
  wb_motor_set_velocity(right_motor, MAX_SPEED);
}

void motor_rotate_left_in_degrees(float degrees) {
  motor_rotate_left();
  float duration = calculate_rotation_time(degrees);
  float start_time = wb_robot_get_time();
  do {
    wb_robot_step(TIME_STEP);
  } while (wb_robot_get_time() < start_time + duration);
  motor_stop();
}

bool *get_sensors_condition()
{
  static bool sensors_condition[NUMBER_OF_DISTANCE_SENSORS] = {false};

  for (int i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++) {
    sensors_condition[i] = wb_distance_sensor_get_value(distance_sensors[i]) > SENSOR_VALUE_DETECTION_THRESHOLD;
  }

  return sensors_condition;
}

const double* get_box_position() {
  if (box_node == NULL) {
    return NULL;
  }
  return wb_supervisor_node_get_position(box_node);
}

void print_sensor_values() {
  printf("%s Sensores: ", wb_robot_get_name());

  for (int i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++) {
    printf("%d:%.3f ", i, wb_distance_sensor_get_value(distance_sensors[i]));
  }

  // Adiciona a posição da caixa ao print


  printf("\n");
}
