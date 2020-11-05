/**
  ******************************************************************************
  * @file       example.c
  * @date       01/10/2014 12:00:00
  * @brief      Example functions for the X-NUCLEO-IHM02A1
******************************************************************************
  *
  * COPYRIGHT(c) 2014 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

#include "example.h"


/**
  * @addtogroup MicrosteppingMotor_Exampled
  * @{
  */

/**
  * @addtogroup Example
  * @{
  */

/**
  * @defgroup   ExamplePrivateFunctions
  * @brief      Example Private Functions.
  * @{
  */

/**
  * @}
  */ /* End of ExamplePrivateFunctions */

/**
  * @addtogroup ExamplePrivateFunctions
  * @brief      Example Private Functions.
  * @{
  */

/**
  * @addtogroup ExampleExportedFunctions
  * @brief      Example Exported Functions.
  * @{
  */

/**
  * @brief  Example no.1 for X-NUCLEO-IHM02A1.
  * @note	Perform a complete motor axis revolution as MPR_1 equal movements,
  *			for each L6470 mounted on all stacked X-NUCLEO-IHM02A1.
  *			At the end of each movement there is a delay of DELAY_1 ms.
  *     	After each motor has performed a complete revolution there is a
  *			delay of DELAY_2 ms.
  *			Now all motors for each X-NUCLEO-IHM02A1 will start at the same
  *			time.
  *			They are going to run at INIT_SPEED for DELAY_3 ms.
  *			After that all motors for each X-NUCLEO-IHM02A1 will get a HardStop
  *			at the same time.
  *			Perform a complete motor axis revolution as MPR_2 equal movements,
  *			for each L6470 mounted on all stacked X-NUCLEO-IHM02A1.
  *			At the end of each movement there is a delay of DELAY_1 ms.
  *			After that all motors for each X-NUCLEO-IHM02A1 will get a HardHiZ
  *			at the same time.
  */
	
#define  PERIOD_VALUE       (uint32_t)(666 - 1)  /* Period Value  */
#define  PULSE1_VALUE       (uint32_t)(PERIOD_VALUE/2)        /* Capture Compare 1 Value  */
#define  PULSE2_VALUE       (uint32_t)(PERIOD_VALUE*37.5/100) /* Capture Compare 2 Value  */
#define  PULSE3_VALUE       (uint32_t)(PERIOD_VALUE/4)        /* Capture Compare 3 Value  */
#define  PULSE4_VALUE       (uint32_t)(PERIOD_VALUE*12.5/100)
	
	
	
	#define Ulna 200 // [ mm ]
	#define Humerus 300 // [ mm ]
	#define Wrist_Link = 300 // [ mm ]
	
	#define M_PI 3.14159265358979323846
	
	#define HOME_SPEED 30000//12800
	
	#define HOME_HUM 1.431 // HOME_HUM and LIM_HUM must sum to Pi/2
	#define HOME_ULN 0.65 
	#define HOME_WST 0.8
	
	#define LIM_HUM 0.14 // HOME_HUM and LIM_HUM must sum to Pi/2
	#define LIM_ULN 0.0
	#define LIM_WST 0.1
	
	#define STEPS_PER_ROTATION 351488 
	#define GRIPPER_STEPS_PER_ROTATION 25600
	
	#define STEPS_PER_RAD_BASE 38003.0 	// 5 2/11 gearbox from 1/128th microstepping motor, 10/18 belt reduction
	#define STEPS_PER_RAD_HUM 111911.0   // 13 212/289 gearbox from 1/128th microstepping motor, 10/20 belt reduction
	#define STEPS_PER_RAD_ULN 111911.0  // same, for now
	
	#define ALPHA_CORRECTION_RATIO 1.50
	#define BETA_CORRECTION_RATIO 1.50
	#define GAMMA_CORRECTION_RATIO 1.50
	
	#define GAMMA_STARTING_ANGLE 0.5307787
	
	#define HUMERUS_MAX_PIN GPIO_PIN_8 // PB_8 - D7 on board
	#define ULNA_MIN_PIN GPIO_PIN_9			// PB_9 - D8 on board
	#define WRIST_MID_PIN GPIO_PIN_0	// PB_0 - A0 on board
	#define CLAW_PIN GPIO_PIN_6 // not used


	float calc_alpha(float x, float y);
	float calc_beta(float x, float y, float z);
	float calc_gamma(float x, float y, float z);
	float calc_L(float x, float y, float z);
	float angle_to_steps(float angle);
	float Base_Rot(float x_c, float y_c, float x_t, float y_t) ;
	float dist(float x, float y) ;
	float Phi(float dist, float z) ;
	float Beta(float dist, float z, float Phi) ;
	float Humerus_Rot(float Beta) ;
	float Ulna_Rot (float Phi) ;
	
	
	
	eL6470_DirId_t find_Dir_beta(float steps);
	eL6470_DirId_t find_Dir_gama(float steps);
	void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle);
	void user_pwm_setvalue(uint16_t value);
	static void SystemClock_Config(void);
	int gripper_angle_to_steps(float angle);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;
	TIM_HandleTypeDef htim4;
	static UART_HandleTypeDef huart2;
	
void Home_Arm()
{
	uint8_t id;
	HAL_Init();
	
	uint8_t buffer[10];
	uint8_t data[6];
	
	
	__GPIOA_CLK_ENABLE();
	GPIO_InitStructure.Pin   			= HUMERUS_MAX_PIN;
	GPIO_InitStructure.Mode  			= GPIO_MODE_INPUT;
	GPIO_InitStructure.Pull 		 	= GPIO_NOPULL;
	GPIO_InitStructure.Speed 			= GPIO_SPEED_FAST;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	__GPIOA_CLK_ENABLE();
	GPIO_InitStructure.Pin   			= ULNA_MIN_PIN;
	GPIO_InitStructure.Mode  			= GPIO_MODE_INPUT;
	GPIO_InitStructure.Pull  			= GPIO_NOPULL;
	GPIO_InitStructure.Speed 			= GPIO_SPEED_FAST;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	__GPIOA_CLK_ENABLE();
	GPIO_InitStructure.Pin   			= WRIST_MID_PIN;
	GPIO_InitStructure.Mode  			= GPIO_MODE_INPUT;
	GPIO_InitStructure.Pull  			= GPIO_NOPULL;
	GPIO_InitStructure.Speed 			= GPIO_SPEED_FAST;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
  StepperMotorBoardHandle_t *StepperMotorBoardHandle;
  MotorParameterData_t *MotorParameterDataGlobal, *MotorParameterDataSingle;
  
	uint8_t board1 = EXPBRD_ID(0);
	uint8_t board2 = EXPBRD_ID(1);
	uint8_t board3 = EXPBRD_ID(2);
	
  /* Setup each X-NUCLEO-IHM02A1 Expansion Board ******************************/
  
  /* Get the parameters for the motor connected with the 1st stepper motor driver of the 1st stepper motor expansion board */
  MotorParameterDataGlobal = GetMotorParameterInitData();
  
  for (id = 0; id < EXPBRD_MOUNTED_NR; id++)
  {
    StepperMotorBoardHandle = BSP_GetExpansionBoardHandle(EXPBRD_ID(id));
    MotorParameterDataSingle = MotorParameterDataGlobal+(id*L6470DAISYCHAINSIZE);
    StepperMotorBoardHandle->Config(MotorParameterDataSingle);
  }
	
			// Don't Home Wrist
		
			StepperMotorBoardHandle->Command->SoftStop(board1, L6470_ID(0)); // lock all motors in place using the softstop command
			StepperMotorBoardHandle->Command->SoftStop(board1, L6470_ID(1));
			StepperMotorBoardHandle->Command->SoftStop(board2, L6470_ID(0));
			StepperMotorBoardHandle->Command->SoftStop(board2, L6470_ID(1));
			StepperMotorBoardHandle->Command->SoftStop(board3, L6470_ID(0));
			StepperMotorBoardHandle->Command->SoftStop(board3, L6470_ID(1));
	
			
			// HOME GRIPPER - just run it into the hard stop
		
			StepperMotorBoardHandle->Command->Move(board3, L6470_ID(0), L6470_DIR_REV_ID, 10000);	// opens the gripper
			while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board3, L6470_ID(0), BUSY_ID)==0);
		
			StepperMotorBoardHandle->Command->SoftStop(board3, L6470_ID(0)); // lock gripper in place		
		
			//HOME ULNA
			StepperMotorBoardHandle->Command->ResetPos(board2, L6470_ID(0));
			StepperMotorBoardHandle->Command->Run(board2, L6470_ID(0), L6470_DIR_REV_ID, HOME_SPEED);
			while(!HAL_GPIO_ReadPin(GPIOA, ULNA_MIN_PIN));
			StepperMotorBoardHandle->Command->HardStop(board2, L6470_ID(0));
			StepperMotorBoardHandle->Command->Run(board2, L6470_ID(0), L6470_DIR_FWD_ID, HOME_SPEED);
			while(HAL_GPIO_ReadPin(GPIOA, ULNA_MIN_PIN));
			StepperMotorBoardHandle->Command->HardStop(board2, L6470_ID(0));
			StepperMotorBoardHandle->Command->ResetPos(board2, L6470_ID(0));
			StepperMotorBoardHandle->Command->Move(board2, L6470_ID(0), L6470_DIR_FWD_ID, angle_to_steps(HOME_ULN));
			while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board2, L6470_ID(0), BUSY_ID)==0);
			
			StepperMotorBoardHandle->Command->SoftStop(board2, L6470_ID(0)); // lock ulna in place
 			
			//HOME WRIST ANGLE
			StepperMotorBoardHandle->Command->ResetPos(board2, L6470_ID(1));
			StepperMotorBoardHandle->Command->Run(board2, L6470_ID(1), L6470_DIR_FWD_ID, HOME_SPEED);
			while(!HAL_GPIO_ReadPin(GPIOA, WRIST_MID_PIN)){};
			StepperMotorBoardHandle->Command->HardStop(board2, L6470_ID(1));
			StepperMotorBoardHandle->Command->Run(board2, L6470_ID(1), L6470_DIR_REV_ID, HOME_SPEED);
			while(HAL_GPIO_ReadPin(GPIOA, WRIST_MID_PIN)){};
			StepperMotorBoardHandle->Command->HardStop(board2, L6470_ID(1));
			StepperMotorBoardHandle->Command->ResetPos(board2, L6470_ID(1));
			StepperMotorBoardHandle->Command->Move(board2, L6470_ID(1), L6470_DIR_REV_ID, angle_to_steps(HOME_WST));
			while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board2, L6470_ID(1), BUSY_ID)==0);
			
			StepperMotorBoardHandle->Command->SoftStop(board2, L6470_ID(1));	// lock wrist arm in place
				
			//HOME HUMERUS
			while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board2, L6470_ID(0), BUSY_ID)==0); // wait for ulna
			StepperMotorBoardHandle->Command->ResetPos(board1, L6470_ID(1));
			StepperMotorBoardHandle->Command->Run(board1, L6470_ID(1), L6470_DIR_FWD_ID, HOME_SPEED);
			while(!HAL_GPIO_ReadPin(GPIOA, HUMERUS_MAX_PIN));
			StepperMotorBoardHandle->Command->HardStop(board1, L6470_ID(1));
			StepperMotorBoardHandle->Command->Run(board1, L6470_ID(1), L6470_DIR_REV_ID, HOME_SPEED);
			while(HAL_GPIO_ReadPin(GPIOA, HUMERUS_MAX_PIN));
			StepperMotorBoardHandle->Command->HardStop(board1, L6470_ID(1));
			StepperMotorBoardHandle->Command->ResetPos(board1, L6470_ID(1));
			StepperMotorBoardHandle->Command->Move(board1, L6470_ID(1), L6470_DIR_REV_ID, angle_to_steps(HOME_HUM));
			while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board1, L6470_ID(1), BUSY_ID)==0);
				
			StepperMotorBoardHandle->Command->Move(board2, L6470_ID(1), L6470_DIR_FWD_ID, angle_to_steps(HOME_WST-0.1)); // move the wrist back to level, not on the limit switch though
		
}

void Move_Arm_Relative(float x_c, float y_c, float z_c, float x_t, float y_t, float z_t, float WristAngle, int GripperState)
{
	uint8_t id;
	HAL_Init();
	
	uint8_t buffer[10];
	uint8_t data[6];
	
	
	StepperMotorBoardHandle_t *StepperMotorBoardHandle;
  MotorParameterData_t *MotorParameterDataGlobal, *MotorParameterDataSingle;
  
	uint8_t board1 = EXPBRD_ID(0);
	uint8_t board2 = EXPBRD_ID(1);
	uint8_t board3 = EXPBRD_ID(2);
	
  // Setup each X-NUCLEO-IHM02A1 Expansion Board *****************************
  
  // Get the parameters for the motor connected with the 1st stepper motor driver of the 1st stepper motor expansion board
  MotorParameterDataGlobal = GetMotorParameterInitData();
  
  for (id = 0; id < EXPBRD_MOUNTED_NR; id++)
  {
    StepperMotorBoardHandle = BSP_GetExpansionBoardHandle(EXPBRD_ID(id));
    MotorParameterDataSingle = MotorParameterDataGlobal+(id*L6470DAISYCHAINSIZE);
    StepperMotorBoardHandle->Config(MotorParameterDataSingle);
  }

	
	
		if(GripperState == 1){
					StepperMotorBoardHandle->Command->Move(board3, L6470_ID(0), L6470_DIR_FWD_ID, 10000) ;
					while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board3, L6470_ID(0), BUSY_ID)==0);
					
		}
		else if(GripperState == 2)
		{
			StepperMotorBoardHandle->Command->Move(board3, L6470_ID(0), L6470_DIR_REV_ID, 10000) ;
			while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board3, L6470_ID(0), BUSY_ID)==0);
		}
		
	
		StepperMotorBoardHandle->Command->Move(board1, L6470_ID(0), L6470_DIR_FWD_ID, Base_Rot(x_c, y_c,x_t,y_t));	 
	
		float Delta_Beta = Beta(dist(x_t,y_t),z_t,Phi(dist(x_t,y_t),z_t)) - Beta(dist(x_c,y_c),z_c,Phi(dist(x_c,y_c),z_c)) ;
		float Delta_Phi = Phi(dist(x_t,y_t),z_t) - Phi(dist(x_c,y_c),z_c) ; 
		
		
		StepperMotorBoardHandle->Command->Move(board1, L6470_ID(1), L6470_DIR_FWD_ID, fabs(Humerus_Rot(Delta_Beta)));	
		
		StepperMotorBoardHandle->Command->Move(board2, L6470_ID(0), L6470_DIR_REV_ID, fabs(Ulna_Rot(Delta_Phi)));		
		
}

float Base_Rot(float x_c, float y_c, float x_t, float y_t){
	float steps = STEPS_PER_RAD_BASE * (atan(y_t/x_t) - atan(y_c/x_c)) ; // in steps, check if atan2 works better

	return steps ;
}


float dist(float x, float y){
	float dist = pow(pow(x,2) + pow(y,2),0.5) ;
	return dist ;
}

float Phi(float dist, float z){
	float Phi = acos((pow(dist,2) + pow(z,2) - pow(Humerus,2) - pow(Ulna,2))/(2*Humerus*Ulna)) ; //   122.797 or 2.143
	return Phi ;
}

float Beta (float dist, float z, float Phi){
	float beta = atan(z / dist) + atan((Ulna * sin(Phi))/(Humerus + Ulna*cos(Phi))) ; // 36.06 + 41.265
	return beta ;
}
float Humerus_Rot(float Beta){
	float steps = Beta * STEPS_PER_RAD_HUM ;
	return steps ; // steps will be positive as Beta increases, negative as Beta decreases
}

float Ulna_Rot(float Phi){
	float ulna_rot = Phi * STEPS_PER_RAD_ULN ; // 
	return ulna_rot ;
}


float calc_alpha(float x, float y){
	float alpha = atan(y/x);
	if (x < 0) {
		alpha -= M_PI;
	}
	return alpha;
}

float calc_beta(float x, float y, float z){
	
	float temp_L = calc_L(x, y, z);
	
	float beta =  acos((pow(Ulna, 2) - pow(Humerus, 2) - pow(temp_L, 2))/(-2*Humerus*temp_L));
	
	float beta_inverse = (M_PI/2)-beta - asin(z/temp_L);
	
	return beta_inverse;
}

float calc_gamma(float x, float y, float z){
	float temp_L = calc_L(x, y, z);
	
	/*To Do: add gamma calculations, it was done incorrectly */
	float gama = acos((pow(temp_L, 2) - pow(Ulna, 2) - pow(Humerus, 2))/(-2*Ulna*Humerus));
	
	float delta_2 = gama - calc_beta(x, y, z);
	
	return  delta_2;
}

float calc_L(float x, float y, float z){
	return (float)sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
}

float angle_to_steps(float angle){
	float steps = STEPS_PER_RAD_HUM*angle;
	return steps;
}


eL6470_DirId_t find_Dir_beta(float steps){
	if(steps>=0){
		return L6470_DIR_FWD_ID;
	}
	else{
		return L6470_DIR_REV_ID;
	}
}

int gripper_angle_to_steps(float angle){
	int steps = GRIPPER_STEPS_PER_ROTATION*angle/(2*M_PI);
	return steps;
}


/**
  * @}
  */ /* End of ExamplePrivateFunctions */

/**
  * @}
  */ /* End of Example */

/**
  * @}
  */ /* End of MicrosteppingMotor_Example */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
