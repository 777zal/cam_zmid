/*
 * ctrl.c
 *
 *  Created on: Sep 8, 2023
 *      Author: jal
 */

#include "ctrl.h"

CAM_STATE_t cam_state = CAM1;

CAM_FLAG_t 	cam_flag = {
		.cam1_is_running = true,
		.cam2_is_running = false,
		.timer_is_running = false
};

CAM_TIMER_t cam_timer = {
		.seconds = 0,
		.minutes = 0
};
