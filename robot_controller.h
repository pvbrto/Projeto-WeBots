#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <stdbool.h>
#include <webots/supervisor.h>

void robot_controller_init(int time_step);
void motor_stop();
void motor_move_forward();
void motor_move_backward();
void motor_rotate_right();
void motor_rotate_left();
void motor_rotate_left_in_degrees(float degrees);
bool *get_sensors_condition();
void print_sensor_values();
const double* get_box_position();

#endif