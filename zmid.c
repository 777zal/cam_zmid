/*
 * zmid.c
 *
 *  Created on: Sep 9, 2023
 *      Author: jal
 */


#include "zmid.h"

ctrl_frame_state_t 	 		zmid_state = FRAME_IDLE;
zmid_parameter_t	 		zmid_param = {0};
zmid_package_parameter_t	zmid_package = {0};
