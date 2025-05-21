/**
 * ****************************************************************************
 * File: L6470.h
 *
 * Author: Martin Stender
 * Company: Ingenious Technology
 * Created: May 5, 2025
 * Version: 1.0
 *
 * Description:
 *   This header file provides definitions, declarations, and public function
 *   for the STM L6470 motor driver.
 *
 ******************************************************************************
 * MIT License
 *
 * Copyright (c) 2025 [Martin Stender]
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sub-license, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * *****************************************************************************
 */

#ifndef __L6470_H_
#define __L6470_H_

#ifdef __cplusplus
 extern "C" {
#endif

 /* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

 /* Definitions ---------------------------------------------------------------*/
#define SET_STEP_MODE_SYNC_EN_BIT     (0x80)

 /* Type definitions ----------------------------------------------------------*/
 typedef void (*cbL6470_SpiTxNow)(const uint8_t * const p_tx,
		 	 	 	 	 	 	 uint8_t const n_tx,
								 const uint8_t * const p_rx,
								 uint8_t const n_rx);

typedef struct
{
	uint8_t tx_buffer[4];
	uint8_t rx_buffer[4];
	cbL6470_SpiTxNow SpiTxNow;
} sL6470_t;

typedef enum
{
	ABS_POS_REG = 0x01,
	EL_POS_REG = 0x02,
	MARK_REG = 0x03,
	SPEED_REG = 0x04,
	ACC_REG = 0x05,
	DEC_REG = 0x06,
	MAX_SPEED_REG = 0x07,
	MIN_SPEED_REG = 0x08,
	KVAL_HOLD_REG = 0x09,
	KVAL_RUN_REG = 0x0A,
	KVAL_ACC_REG = 0x0B,
	KVAL_DEC_REG = 0x0C,
	INT_SPEED_REG = 0x0D,
	ST_SLP_REG = 0x0E,
	FN_SLP_ACC_REG = 0x0F,
	FN_SLP_DEC_REG = 0x10,
	K_THERM_REG = 0x11,
	ADC_OUT_REG = 0x12,
	OCD_TH_REG = 0x13,
	STALL_TH_REG = 0x14,
	FS_SPD_REG = 0x15,
	STEP_MODE_REG = 0x16,
	ALARM_EN_REG = 0x17,
	CONFIG_REG = 0x18,
	STATUS_REG = 0x19
}eL6470_Register_Address_t;

typedef enum {
	DIR_REVERSE = 0,
	DIR_FORWARD = 1
} eL6470_Direction_t;


typedef enum {
	ACT_ABS_POS_RESET = 0,
	ACT_ABS_POS_MARK = 1,
} eL6470_ACT_t;

typedef enum
{
	FULL_STEP_MODE = 0x00,
	HALF_STEP_MODE = 0x01,
	MICRO_STEP_1_4_MODE = 0x02,
	MICRO_STEP_1_8_MODE = 0x03,
	MICRO_STEP_1_16_MODE = 0x04,
	MICRO_STEP_1_32_MODE = 0x05,
	MICRO_STEP_1_64_MODE = 0x06,
	MICRO_STEP_1_128_MODE = 0x07,
}eL6470_Step_Mode_t;

typedef enum
{
	OVERCURRENT_ALARM = 0x01,
	THERMAL_SHUTDOWN_ALARM = 0x02,
	THERMAL_WARN_ALARM = 0x04,
	UNDERVOLATAGE_ALARM = 0x08,
	STALL_DETECTION_A_ALARM = 0x10,
	STALL_DETECTION_B_ALARM = 0x20,
	SWITCH_TURN_ON__ALARM = 0x40,
	WRONG_COMMAND_ALARM = 0x80,
}eL6470_Alarm_t;

typedef enum
{
	MOTOR_STOPPED = 0x00,
	MOTOR_ACCELERATION = 0x01,
	MOTOR_DECELERATION = 0x02,
	MOTOR_CONSTANT_SPEED = 0x03,
}eL6470_MotorStatus_t;

void L6470_Init(sL6470_t* const p_driver, cbL6470_SpiTxNow SpiSend);
void L6470_Config(void);

/* L6470 Application Commands */
void L6470_Nop(sL6470_t* const p_driver);
void L6470_SetParam(sL6470_t* const p_driver, const eL6470_Register_Address_t param, const uint32_t value);
uint32_t L6470_GetParam(sL6470_t* const p_driver, const eL6470_Register_Address_t param);
void L6470_Run(sL6470_t* const p_driver, const eL6470_Direction_t dir, const uint32_t speed);
void L6470_StepClock(sL6470_t* const p_driver, const eL6470_Direction_t dir);
void L6470_Move(sL6470_t* const p_driver, const eL6470_Direction_t dir, const uint32_t n_step);
void L6470_GoTo(sL6470_t* const p_driver, const uint32_t abs_pos);
void L6470_GoTo_DIR(sL6470_t* const p_driver, const eL6470_Direction_t dir, const uint32_t abs_pos);
void L6470_GoUntil(sL6470_t* const p_driver, const eL6470_ACT_t act, const eL6470_Direction_t dir, const uint32_t speed);
void L6470_ReleaseSW(sL6470_t* const p_driver, const eL6470_ACT_t act, const eL6470_Direction_t dir);
void L6470_GoHome(sL6470_t* const p_driver);
void L6470_GoMark(sL6470_t* const p_driver);
void L6470_ResetPos(sL6470_t* const p_driver);
void L6470_ResetDevice(sL6470_t* const p_driver);
void L6470_SoftStop(sL6470_t* const p_driver);
void L6470_HardStop(sL6470_t* const p_driver);
void L6470_SoftHiZ(sL6470_t* const p_driver);
void L6470_HardHiZ(sL6470_t* const p_driver);
uint16_t L6470_GetStatus(sL6470_t* const p_driver);

/* L6470 utility functions for conversions */
int32_t L6470_AbsPos_2_Position(uint32_t const abs_pos);
uint32_t L6470_Position_2_AbsPos(int32_t position);
float L6470_MaxSpeed_2_Step_s(uint16_t const max_speed);
uint16_t L6470_Step_s_2_MaxSpeed(float const step_s);
float L6470_Speed_2_Step_s(uint32_t speed);
uint32_t L6470_Step_s_2_Speed(float step_s);
float L6470_OcdTh_2_mA(uint8_t ocd_th);
uint8_t L6470_mA_2_OcdTh(const float mA);
float L6470_StallTh_2_mA(uint8_t stall_th);
uint8_t L6470_mA_2_StallTh(const float mA);
float L6470_Acc_2_Step_s2(uint16_t const acc);
uint16_t L6470_Step_s2_2_Acc(float const step_s2);
float L6470_Dec_2_Step_s2(uint16_t const dec);
uint16_t L6470_Step_s2_2_Dec(float const step_s2);
float L6470_FsSpd_2_Step_s(uint16_t const fs_spd);
uint16_t L6470_Step_s_2_FsSpd(float const step_s);
float L6470_IntSpeed_2_Step_s(uint16_t const int_speed);
uint16_t L6470_Step_s_2_IntSpeed(float const step_s);
float L6470_StSlp_2_s_Step(uint8_t const st_slp);
uint8_t L6470_s_Step_2_StSlp(float const s_step);
float L6470_FnSlpAcc_2_s_Step(uint8_t const fn_slp_acc);
uint8_t L6470_s_Step_2_FnSlpAcc(float const s_step);
float L6470_FnSlpDec_2_s_Step(uint8_t const fn_slp_dec);
uint8_t L6470_s_Step_2_FnSlpDec(float const s_step);

eL6470_MotorStatus_t L6470_GetMotorStatus(sL6470_t * const p_driver);
void L6470_DisableAlarm(sL6470_t* const p_driver, const eL6470_Alarm_t alarm);
void L6470_EnableAlarm(sL6470_t* const p_driver, const eL6470_Alarm_t alarm);
uint32_t L6470_GetResponse(sL6470_t* const p_driver, const eL6470_Register_Address_t param);

#ifdef __cplusplus
}
#endif

#endif /* __L6470_H_ */
