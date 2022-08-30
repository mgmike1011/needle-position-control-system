/*
 * A4988_Drive.c
 *
 *  Created on: March 2022
 *      Author: Miłosz Gajewski
 */
#include "A4988_Drive.h"
#include "math.h"
#include "main.h"
#include "stm32f7xx_hal.h"
void Set_Resolution_A4988(A4988_Drive* drive, int resolution){
	//
	//	Setting the resolution of the driver.
	//	@param resolution: Resolution.
	//	@param drive: Pointer to structure.
	//	@return: none
	//
	switch(resolution){
		case FULL_STEP:
			HAL_GPIO_WritePin(*&(drive->PORT_MS1), drive->PIN_MS1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(*&(drive->PORT_MS2), drive->PIN_MS2, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(*&(drive->PORT_MS3), drive->PIN_MS3, GPIO_PIN_RESET);
			drive->RESOLUTION = 1;
			break;
		case HALF_STEP:
			HAL_GPIO_WritePin(*&(drive->PORT_MS1), drive->PIN_MS1, GPIO_PIN_SET);
			HAL_GPIO_WritePin(*&(drive->PORT_MS2), drive->PIN_MS2, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(*&(drive->PORT_MS3), drive->PIN_MS3, GPIO_PIN_RESET);
			drive->RESOLUTION = 2;
			break;
		case QUARTER_STEP:
			HAL_GPIO_WritePin(*&(drive->PORT_MS1), drive->PIN_MS1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(*&(drive->PORT_MS2), drive->PIN_MS2, GPIO_PIN_SET);
			HAL_GPIO_WritePin(*&(drive->PORT_MS3), drive->PIN_MS3, GPIO_PIN_RESET);
			drive->RESOLUTION = 4;
			break;
		case ONE_EIGHTH_STEP:
			HAL_GPIO_WritePin(*&(drive->PORT_MS1), drive->PIN_MS1, GPIO_PIN_SET);
			HAL_GPIO_WritePin(*&(drive->PORT_MS2), drive->PIN_MS2, GPIO_PIN_SET);
			HAL_GPIO_WritePin(*&(drive->PORT_MS3), drive->PIN_MS3, GPIO_PIN_RESET);
			drive->RESOLUTION = 8;
			break;
		case ONE_SIXTEENTH_STEP:
			HAL_GPIO_WritePin(*&(drive->PORT_MS1), drive->PIN_MS1, GPIO_PIN_SET);
			HAL_GPIO_WritePin(*&(drive->PORT_MS2), drive->PIN_MS2, GPIO_PIN_SET);
			HAL_GPIO_WritePin(*&(drive->PORT_MS3), drive->PIN_MS3, GPIO_PIN_SET);
			drive->RESOLUTION = 16;
			break;
		default:
			HAL_GPIO_WritePin(*&(drive->PORT_MS1), drive->PIN_MS1, GPIO_PIN_SET);
			HAL_GPIO_WritePin(*&(drive->PORT_MS2), drive->PIN_MS2, GPIO_PIN_SET);
			HAL_GPIO_WritePin(*&(drive->PORT_MS3), drive->PIN_MS3, GPIO_PIN_SET);
			drive->RESOLUTION = 16;
	}
}
void Set_Direction_A4988(A4988_Drive* drive, int direction){
	//
	//	Setting the direction of rotation.
	//	@param direction: 1 - Right, 2 - Left, default - Right.
	//	@param drive: Pointer to structure.
	//	@return: none
	//
	switch(direction){
		case 1:
			HAL_GPIO_WritePin(*&(drive->PORT_DIR), drive->PIN_DIR, GPIO_PIN_SET);
			break;
		case 2:
			HAL_GPIO_WritePin(*&(drive->PORT_DIR), drive->PIN_DIR, GPIO_PIN_RESET);
			break;
		default:
			HAL_GPIO_WritePin(*&(drive->PORT_DIR), drive->PIN_DIR, GPIO_PIN_SET);
	}
}
void Enable_A4988(A4988_Drive* drive, int signal){
	//
	//	Enable controler.
	//	@param signal: ENABLE_DRIVE - 1, DISABLE_DRIVE - 2, default - disable.
	//	@param drive: Pointer to structure.
	//	@return: none
	//
	switch(signal){
		case ENABLE_DRIVE:
			HAL_GPIO_WritePin(*&(drive->PORT_ENABLE), drive->PIN_ENABLE, GPIO_PIN_RESET);
			break;
		case DISABLE_DRIVE:
			HAL_GPIO_WritePin(*&(drive->PORT_ENABLE), drive->PIN_ENABLE, GPIO_PIN_SET);
			break;
		default:
			HAL_GPIO_WritePin(*&(drive->PORT_ENABLE), drive->PIN_ENABLE, GPIO_PIN_SET);
	}
}
void Reset_A4988(A4988_Drive* drive, int reset){
	//
	//	Reset controler.
	//	@param reset: ENABLE_DRIVE - 1, DISABLE_DRIVE - 2, default - disable.
	//	@param drive: Pointer to structure.
	//	@return: none
	//
	switch(reset){
		case ENABLE_DRIVE:
			HAL_GPIO_WritePin(*&(drive->PORT_RESET), drive->PIN_RESET, GPIO_PIN_SET);
			break;
		case DISABLE_DRIVE:
			HAL_GPIO_WritePin(*&(drive->PORT_RESET), drive->PIN_RESET, GPIO_PIN_RESET);
			break;
		default:
			HAL_GPIO_WritePin(*&(drive->PORT_RESET), drive->PIN_RESET, GPIO_PIN_RESET);
	}
}
void Sleep_A4988(A4988_Drive* drive, int sleep){
	//
	//	Enable sleep mode.
	//	@param sleep: ENABLE_DRIVE - 1, DISABLE_DRIVE - 2, default - disable.
	//	@param drive: Pointer to structure.
	//	@return: none
	//
	switch(sleep){
		case ENABLE_DRIVE:
			HAL_GPIO_WritePin(*&(drive->PORT_SLEEP), drive->PIN_SLEEP, GPIO_PIN_SET);
			break;
		case DISABLE_DRIVE:
			HAL_GPIO_WritePin(*&(drive->PORT_SLEEP), drive->PIN_SLEEP, GPIO_PIN_RESET);
			break;
		default:
			HAL_GPIO_WritePin(*&(drive->PORT_SLEEP), drive->PIN_SLEEP, GPIO_PIN_RESET);
	}
}
int Calculate_Steps_A4988(A4988_Drive* drive, float angle){
	//
	// Calculate steps steps needed to rotate by a given angle.
	// @param angle: Angle.
	// @param resolution: Set resolution.
	// @param drive: Pointer to structure.
	// @return: Number of steps.
	//
	int steps = 0;
	float r_s,x_s;
	r_s = 360/((float)(drive->STEPS));
	x_s = r_s/((float)(drive->RESOLUTION));
	steps = (int)round(angle/x_s);
	return steps;
}
void Rotate_A4988(A4988_Drive* drive, float angle){
	//
	//	Rotates the motor by a given angle.
	//	@param angle: Desired angle to rotate.
	//	@param drive: Pointer to structure.
	//	@return: none
	//
	int steps = Calculate_Steps_A4988(drive, angle);
	__HAL_TIM_SET_AUTORELOAD(drive->TIM_STEP,steps);
	Sleep_A4988(drive, ENABLE_DRIVE);
	HAL_TIM_PWM_Start(drive->TIM_STEP, drive->TIM_STEP_CHANNEL);
}
void Rotate_mm_A4988(A4988_Drive* drive, float distance){
	//
	//	Rotates the motor by a given distance in mm.
	//	@param distance: Distance in mm.
	//	@param drive: Pointer to structure.
	//	@return: none
	//
	float step_res_ = 360/(float)(drive->STEPS);
	float angle = (distance*step_res_)/((float)(drive->STEP_mm_RESOLUTION));
	Rotate_A4988(drive, angle);
}
void Set_Speed(A4988_Drive* drive, int rpm){
	//
	//	Sets the speed of rotor.
	//	@param rpm: Revolutions per minute
	//	@param drive: Pointer to structure.
	//	@return: none
	//
	if(rpm > 0){
		uint16_t arr_val = TIM_CLK/(((drive->TIM_STEP->Instance->PSC+1)*rpm*drive->RESOLUTION/60))-1;
		uint16_t pulse_val = arr_val / 2;
		__HAL_TIM_SET_AUTORELOAD(drive->TIM_STEP, arr_val);
		__HAL_TIM_SET_COMPARE(drive->TIM_STEP, drive->TIM_STEP_CHANNEL, pulse_val);
	}else{
		__HAL_TIM_SET_COMPARE(drive->TIM_STEP, drive->TIM_STEP_CHANNEL,0);
	}
}
void Init_A4988(A4988_Drive* drive){
	//
	//	Initialize the drive.
	//	@param drive: Pointer to structure.
	//	@return: none
	//
	HAL_TIM_Base_Start_IT(drive->TIM_COUNTER_SLAVE);
	Set_Resolution_A4988(drive, HALF_STEP);
	Set_Direction_A4988(drive, RIGHT_DIR);
	Enable_A4988(drive, ENABLE_DRIVE);
	Reset_A4988(drive, ENABLE_DRIVE);
	Sleep_A4988(drive, DISABLE_DRIVE);
}
