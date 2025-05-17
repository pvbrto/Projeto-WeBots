#include "robot_controller.h"
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define QtddCaixa 20
#define MOVEMENT_THRESHOLD 0.005 
#define RANDOM_TURN_INTERVAL 50  


double initial_box_positions[QtddCaixa][3];
WbNodeRef caixa[QtddCaixa];


void initialize_box_positions() {
    char nomeCaixa[10] = {0};
    for(int i = 0; i < QtddCaixa; i++) {
        sprintf(nomeCaixa, "CAIXA%d", i + 1);
        caixa[i] = wb_supervisor_node_get_from_def(nomeCaixa);
        printf("Caixa %d: %s\n", i + 1, nomeCaixa);
        if(caixa[i] != NULL) {
            const double *pos = wb_supervisor_node_get_position(caixa[i]);
            initial_box_positions[i][0] = pos[0];
            initial_box_positions[i][1] = pos[1];
            initial_box_positions[i][2] = pos[2];
        }
    }
}


bool check_box_movement() {
    for(int i = 0; i < QtddCaixa; i++) {
        if(caixa[i] != NULL) {
            const double *current_pos = wb_supervisor_node_get_position(caixa[i]);
            double dx = current_pos[0] - initial_box_positions[i][0];
            double dy = current_pos[1] - initial_box_positions[i][1];
            double dz = current_pos[2] - initial_box_positions[i][2];
            
   
            if(dx*dx + dy*dy + dz*dz > MOVEMENT_THRESHOLD * MOVEMENT_THRESHOLD) {
                return true;
            }
        }
    }
    return false;
}

int main() {
  wb_robot_init();
  srand(time(NULL));  


  if (!wb_robot_get_supervisor()) {
    printf("Este controlador precisa de permissões de supervisor\n");
    wb_robot_cleanup();
    return 1;
  }

  robot_controller_init(32); 
  initialize_box_positions(); 

  int step_counter = 0; 

  while (wb_robot_step(32) != -1) {		
		print_sensor_values();
		
		bool *is_sensors_active = get_sensors_condition();
		

		if (check_box_movement()) {
			
			motor_rotate_left();
		} else if (is_sensors_active[1] && is_sensors_active[6]) {
			motor_rotate_left_in_degrees(180);
		} else if (is_sensors_active[0] || is_sensors_active[1]) {
			motor_rotate_left();
		} else if (is_sensors_active[7] || is_sensors_active[6]) {
			motor_rotate_right();
		} else {
			
			step_counter++;
			if (step_counter >= RANDOM_TURN_INTERVAL) {
				step_counter = 0;
				int random_action = rand() % 100; 
				
				if (random_action < 20) { 
					motor_rotate_left();
					wb_robot_step(32 * 10);  
				} else if (random_action < 40) { 
					motor_rotate_right();
					wb_robot_step(32 * 10); 
				}
			
			}
			motor_move_forward();
		}
  };

  wb_robot_cleanup();
  return 0;
}