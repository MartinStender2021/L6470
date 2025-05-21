/**
 * ****************************************************************************
 * File: L6470.c
 *
 * Author: Martin Stender
 * Company: Ingenious Technology
 * Created: May 5, 2025
 * Version: 1.0
 *
 * Description:
 *   This source file provides contains functions for the STM L6470 motor driver.
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

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "L6470.h"

/* Definitions ---------------------------------------------------------------*/
#define L6470_MAX_POSITION          (0x1FFFFF)      // Max position
#define L6470_MIN_POSITION          (-(0x200000))   // Min position
#define L6470_POSITION_RANGE        ((uint32_t)(L6470_MAX_POSITION - L6470_MIN_POSITION))
#define L6470_MAX_SPEED             (0xFFFFF)       // max value of SPEED
#define L6470_MAX_MAX_SPEED         (0x3FF)         // max value of MAX_SPEED
#define L6470_MAX_MIN_SPEED         (0xFFF)         // max value of MIN_SPEED
#define L6470_MAX_FS_SPD            (0x3FF)         // max value of FS_SPD
#define L6470_MAX_INT_SPEED         (0x3FFF)        // max value of INT_SPEED
#define L6470_MAX_ST_SLP            (0xFF)          // max value of ST_SLP
#define L6470_MAX_FN_SLP_ACC        (0xFF)          // max value of FN_SLP_ACC
#define L6470_MAX_FN_SLP_DEC        (0xFF)          // max value of FN_SLP_DEC
#define L6470_MAX_OCD_TH            (0xF)           // max value of OCD_TH
#define L6470_MAX_STALL_TH          (0x7F)          // max value of STALL_TH
#define L6470_MAX_ACC               (0xFFF)         // max value of ACC
#define L6470_MAX_DEC               (0xFFF)         // max value of DEC

#define L6470_MAX_SPEED_VALUE   ((float)15610)      // max value for the speed in step/s
#define L6470_MAX_ACC_VALUE     ((float)59590)      // max value for the acceleration in step/s^2
#define L6470_MAX_DEC_VALUE     ((float)59590)      // max value for the acceleration in step/s^2
#define L6470_MAX_DEC_VALUE     ((float)59590)      // max value for the acceleration in step/s^2

#define L6470_ACT_BIT_POS  (3)                      // ACT bit position in all registers
#define L6470_DIR_BIT_POS  (0)                      // DIR bit position in all registers
#define MOTOR_STATUS_MASK  (0x60)                   // Bit mask for MOT_STATUS bits in STATUS register

/* Type definitions ----------------------------------------------------------*/
typedef struct {
	eL6470_Register_Address_t address;  // Register Address
	uint8_t n_bits;                     // Register Length in bits
	uint8_t n_bytes;                    // Register Length in bytes
	uint32_t reset_value;               // Register Reset Value
} sL6470_Register_t;

typedef enum {
	NOP,
	SET_PARAM,
	GET_PARAM,
	RUN,
	STEP_CLOCK,
	MOVE,
	GOTO,
	GOTO_DIR,
	GO_UNTIL,
	RELEASE_SW,
	GO_HOME,
	GO_MARK,
	RESET_POS,
	RESET_DEVICE,
	SOFT_STOP,
	HARD_STOP,
	SOFT_HIZ,
	HARD_HIZ,
	GET_STATUS
} eL6470_Application_Command_Id_t;

typedef struct {
	eL6470_Application_Command_Id_t command_id;
	uint8_t BinaryCode;
} sL6470_ApplicationCommand_t;

/* Function prototypes--------------------------------------------------------*/
static void L6470_value_2_tx_buf(uint8_t * const p_dest, const uint32_t value, const uint8_t n_bytes);

/* L6470 Register map & info */
const sL6470_Register_t L6470_Register[] = {
    {0x00,           0,   0,   0x000000},  // Dummy entry (for alignment of register ids)
    {ABS_POS_REG,   22,   3,   0x000000},  // Current position
    {EL_POS_REG,     9,   2,      0x000},  // Electrical position
    {MARK_REG,      22,   3,   0x000000},  // Mark position
    {SPEED_REG,     20,   3,     0x0000},  // Current speed
    {ACC_REG,       12,   2,      0x08A},  // Acceleration
    {DEC_REG,       12,   2,      0x08A},  // Deceleration
    {MAX_SPEED_REG, 10,   2,      0x041},  // Maximum speed
    {MIN_SPEED_REG, 13,   2,      0x000},  // Minimum speed
    {FS_SPD_REG,    10,   2,      0x027},  // Full-step speed
    {KVAL_HOLD_REG,  8,   1,       0x29},  // Holding KVAL
    {KVAL_RUN_REG,   8,   1,       0x29},  // Constant speed KVAL
    {KVAL_ACC_REG,   8,   1,       0x29},  // Acceleration starting KVAL
    {KVAL_DEC_REG,   8,   1,       0x29},  // Deceleration starting KVAL
    {INT_SPEED_REG, 14,   2,     0x0408},  // Intersect speed
    {ST_SLP_REG,     8,   1,       0x19},  // Start slope
    {FN_SLP_ACC_REG, 8,   1,       0x29},  // Acceleration final slope
    {FN_SLP_DEC_REG, 8,   1,       0x29},  // Deceleration final slope
    {K_THERM_REG,    4,   1,        0x0},  // Thermal compensation factor
    {ADC_OUT_REG,    5,   1,       0x00},  // ADC output, (the reset value is according to startup conditions)
    {OCD_TH_REG,     4,   1,        0x8},  // OCD threshold
    {STALL_TH_REG,   7,   1,       0x40},  // STALL threshold
    {STEP_MODE_REG,  8,   1,        0x7},  // Step mode
    {ALARM_EN_REG,   8,   1,       0xFF},  // Alarm enable
    {CONFIG_REG,    16,   2,     0x2E88},  // IC configuration
    {STATUS_REG,    16,   2,     0x0000}   // Status, (the reset value is according to startup conditions)
};

/* L6470 Application Commands */
const sL6470_ApplicationCommand_t L6470_ApplicationCommand[] =  {
	{NOP,          0x00},   // NOP
	{SET_PARAM,    0x00},   // SetParam
	{GET_PARAM,    0x20},   // GetParam
	{RUN,          0x50},   // Run
	{STEP_CLOCK,   0x58},   // StepClock
	{MOVE,         0x40},   // Move
	{GOTO,         0x60},   // GoTo
	{GOTO_DIR,     0x68},   // GoTo_DIR
	{GO_UNTIL,     0x82},   // GoToUntil
	{RELEASE_SW,   0x92},   // ReleaseSW
	{GO_HOME,      0x70},   // GoHome
	{GO_MARK,      0x78},   // GoMark
	{RESET_POS,    0xD8},   // ResetPos
	{RESET_DEVICE, 0xC0},   // ResetDevice
	{SOFT_STOP,    0xB0},   // SoftStop
	{HARD_STOP,    0xB8},   // HardStop
	{SOFT_HIZ,     0xA0},   // SoftHiZ
	{HARD_HIZ,     0xA8},   // HardHiZ
	{GET_STATUS,   0xD0}    // GetStatus
};

void L6470_Init(sL6470_t* const p_driver, cbL6470_SpiTxNow SpiSend)
{
	if (NULL != p_driver)
	{
		memset(p_driver->tx_buffer, 0, 4);
		memset(p_driver->rx_buffer, 0, 4);
		if(SpiSend != NULL)
		{
			p_driver->SpiTxNow = SpiSend;
		}
	}
}

/**
  * @brief  The Nop command does nothing.
  *
  * @param  p_driver      Pointer to the L6470 driver instance.
  */
void L6470_Nop(sL6470_t* const p_driver)
{
	if (NULL != p_driver)
	{
		memset(p_driver->tx_buffer, 0, 4);
		p_driver->tx_buffer[0] = L6470_ApplicationCommand[NOP].BinaryCode;

		if (NULL != p_driver->SpiTxNow)
		{
			p_driver->SpiTxNow(p_driver->tx_buffer, 1, p_driver->rx_buffer, 0);
		}
	}
}

/**
  * @brief  The SetParam command sets a new value in on of the L6470 registers.
  *
  * @param  p_driver      Pointer to the L6470 driver instance.
  * @param  param		  The L6470 register address.
  * @param  value         The register value
  */
void L6470_SetParam(sL6470_t* const p_driver, const eL6470_Register_Address_t param, const uint32_t value)
{
	if (NULL != p_driver)
	{
		memset(p_driver->tx_buffer, 0, 4);
		p_driver->tx_buffer[0] = L6470_ApplicationCommand[SET_PARAM].BinaryCode | param;
		L6470_value_2_tx_buf(p_driver->tx_buffer, value, L6470_Register[param].n_bytes);

		if (NULL != p_driver->SpiTxNow)
		{
			p_driver->SpiTxNow(p_driver->tx_buffer, 4, p_driver->rx_buffer, 0);
		}
	}
}

/**
  * @brief  The GetParam reads the current register value.
  *
  * @param  p_driver      Pointer to the L6470 driver instance.
  * @param  param		  The L6470 register address.
  */
uint32_t L6470_GetParam(sL6470_t* const p_driver, const eL6470_Register_Address_t param)
{
	uint32_t result = 0;

	if (NULL != p_driver)
	{
		memset(p_driver->tx_buffer, 0, 4);
		memset(p_driver->rx_buffer, 0, 4);
		p_driver->tx_buffer[0] = L6470_ApplicationCommand[GET_PARAM].BinaryCode | param;

		if (NULL != p_driver->SpiTxNow)
		{
			p_driver->SpiTxNow(p_driver->tx_buffer, 1, p_driver->rx_buffer, 4);
			result = L6470_GetResponse(p_driver, param);
		}
	}

	return result;
}

/**
  * @brief  The Run command produces a motion at SPD speed.
  *
  * @param  p_driver      Pointer to the L6470 driver instance.
  * @param  dir			  The L6470 motion direction.
  * @param  speed         The speed value (steps/tick)
  */
void L6470_Run(sL6470_t* const p_driver, const eL6470_Direction_t dir, const uint32_t speed)
{
	if (NULL != p_driver)
	{
		memset(p_driver->tx_buffer, 0, 4);
		p_driver->tx_buffer[0] = L6470_ApplicationCommand[RUN].BinaryCode | (dir<<L6470_DIR_BIT_POS);
		L6470_value_2_tx_buf(p_driver->tx_buffer, speed, 3);

		if (NULL != p_driver->SpiTxNow)
		{
			p_driver->SpiTxNow(p_driver->tx_buffer, 4, p_driver->rx_buffer, 0);
		}
	}
}

/**
  * @brief  The StepClock command switches the device in Step-clock mode.
  *
  * @param  p_driver      Pointer to the L6470 driver instance.
  * @param  dir			  The L6470 motion direction.
  */
void L6470_StepClock(sL6470_t* const p_driver, const eL6470_Direction_t dir)
{
	if (NULL != p_driver)
	{
		memset(p_driver->tx_buffer, 0, 4);
		p_driver->tx_buffer[0] = L6470_ApplicationCommand[STEP_CLOCK].BinaryCode | (dir<<L6470_DIR_BIT_POS);

		if (NULL != p_driver->SpiTxNow)
		{
			p_driver->SpiTxNow(p_driver->tx_buffer, 1, p_driver->rx_buffer, 0);
		}
	}
}

/**
  * @brief  The Move command produces a motion of n steps in the specified direction.
  * 		The step value is in agreement with the selected step mode
  *
  * @param  p_driver      Pointer to the L6470 driver instance.
  * @param  dir			  The L6470 motion direction.
  * @param  n_step		  The step value.
  */
void L6470_Move(sL6470_t* const p_driver, const eL6470_Direction_t dir, const uint32_t n_step)
{
	if (NULL != p_driver)
	{
		memset(p_driver->tx_buffer, 0, 4);
		p_driver->tx_buffer[0] = L6470_ApplicationCommand[MOVE].BinaryCode | (dir<<L6470_DIR_BIT_POS);
		L6470_value_2_tx_buf(p_driver->tx_buffer, n_step, 3);

		if (NULL != p_driver->SpiTxNow)
		{
			p_driver->SpiTxNow(p_driver->tx_buffer, 4, p_driver->rx_buffer, 0);
		}
	}
}

/**
  * @brief  The GoTo command produces a motion to ABS_POS absolute position through
  * 		the shortest path
  *
  * @param  p_driver      Pointer to the L6470 driver instance.
  * @param  abs_pos		  The absolute position.
  */
void L6470_GoTo(sL6470_t* const p_driver, const uint32_t abs_pos)
{
	if (NULL != p_driver && abs_pos <= L6470_POSITION_RANGE)
	{
		memset(p_driver->tx_buffer, 0, 4);
		p_driver->tx_buffer[0] = L6470_ApplicationCommand[GOTO].BinaryCode;
		L6470_value_2_tx_buf(p_driver->tx_buffer, abs_pos, 3);

		if (NULL != p_driver->SpiTxNow)
		{
			p_driver->SpiTxNow(p_driver->tx_buffer, 4, p_driver->rx_buffer, 0);
		}
	}
}

/**
  * @brief  The GoTo_DIR command produces a motion to ABS_POS absolute position imposing
  * 		a forward (DIR = '1') or a reverse (DIR = '0') rotation
  *
  * @param  p_driver      Pointer to the L6470 driver instance.
  * @param  dir			  The L6470 motion direction.
  * @param  abs_pos		  The absolute position.
  */
void L6470_GoTo_DIR(sL6470_t* const p_driver, const eL6470_Direction_t dir, const uint32_t abs_pos)
{
	if (NULL != p_driver && abs_pos <= L6470_POSITION_RANGE)
	{
		memset(p_driver->tx_buffer, 0, 4);
		p_driver->tx_buffer[0] = L6470_ApplicationCommand[GOTO_DIR].BinaryCode | (dir<<L6470_DIR_BIT_POS);
		L6470_value_2_tx_buf(p_driver->tx_buffer, abs_pos, 3);

		if (NULL != p_driver->SpiTxNow)
		{
			p_driver->SpiTxNow(p_driver->tx_buffer, 4, p_driver->rx_buffer, 0);
		}
	}
}

/**
  * @brief  The GoUntil command produces a motion at SPD speed imposing a forward (DIR = '1') or
  * 		a reverse (DIR = '0') direction
  *
  * @param  p_driver      Pointer to the L6470 driver instance.
  * @param  act           The L6470 ACT.
  * @param  dir			  The L6470 motion direction.
  * @param  speed		  The SPD speed.
  */
void L6470_GoUntil(sL6470_t* const p_driver, const eL6470_ACT_t act, const eL6470_Direction_t dir, const uint32_t speed)
{
	if (NULL != p_driver)
	{
		memset(p_driver->tx_buffer, 0, 4);
		p_driver->tx_buffer[0] = L6470_ApplicationCommand[GO_UNTIL].BinaryCode | (dir<<L6470_DIR_BIT_POS) | (act<<L6470_ACT_BIT_POS);
		L6470_value_2_tx_buf(p_driver->tx_buffer, speed, 3);

		if (NULL != p_driver->SpiTxNow)
		{
			p_driver->SpiTxNow(p_driver->tx_buffer, 4, p_driver->rx_buffer, 0);
		}
	}
}

/**
  * @brief  The ReleaseSW command produces a motion at minimum speed imposing a forward
  * 		(DIR = '1') or reverse (DIR = '0') rotation.
  *
  * @param  p_driver      Pointer to the L6470 driver instance.
  * @param  act           The L6470 ACT.
  * @param  dir			  The L6470 motion direction.
  */
void L6470_ReleaseSW(sL6470_t* const p_driver, const eL6470_ACT_t act, const eL6470_Direction_t dir)
{
	if (NULL != p_driver)
	{
		memset(p_driver->tx_buffer, 0, 4);
		p_driver->tx_buffer[0] = L6470_ApplicationCommand[RELEASE_SW].BinaryCode | (dir<<L6470_DIR_BIT_POS) | (act<<L6470_ACT_BIT_POS);

		if (NULL != p_driver->SpiTxNow)
		{
			p_driver->SpiTxNow(p_driver->tx_buffer, 1, p_driver->rx_buffer, 0);
		}
	}
}

/**
  * @brief  The GoHome command produces a motion the the HOME position (zero position)
  * 		via the shortest path.
  *
  * @param  p_driver      Pointer to the L6470 driver instance.
  */
void L6470_GoHome(sL6470_t* const p_driver)
{
	if (NULL != p_driver)
	{
		memset(p_driver->tx_buffer, 0, 4);
		p_driver->tx_buffer[0] = L6470_ApplicationCommand[GO_HOME].BinaryCode;

		if (NULL != p_driver->SpiTxNow)
		{
			p_driver->SpiTxNow(p_driver->tx_buffer, 1, p_driver->rx_buffer, 0);
		}
	}
}

/**
  * @brief  The GoMark command produces a motion to the MARK position performing the minimum path
  *
  * @param  p_driver      Pointer to the L6470 driver instance.
  */
void L6470_GoMark(sL6470_t* const p_driver)
{
	if (NULL != p_driver)
	{
		memset(p_driver->tx_buffer, 0, 4);
		p_driver->tx_buffer[0] = L6470_ApplicationCommand[GO_MARK].BinaryCode;

		if (NULL != p_driver->SpiTxNow)
		{
			p_driver->SpiTxNow(p_driver->tx_buffer, 1, p_driver->rx_buffer, 0);
		}
	}
}

/**
  * @brief  The ResetPos command resets the ABS_POS register to zero.
  *
  * @param  p_driver      Pointer to the L6470 driver instance.
  */
void L6470_ResetPos(sL6470_t* const p_driver)
{
	if (NULL != p_driver)
	{
		memset(p_driver->tx_buffer, 0, 4);
		p_driver->tx_buffer[0] = L6470_ApplicationCommand[RESET_POS].BinaryCode;

		if (NULL != p_driver->SpiTxNow)
		{
			p_driver->SpiTxNow(p_driver->tx_buffer, 1, p_driver->rx_buffer, 0);
		}
	}
}

/**
  * @brief  The ResetDevice command resets the device to power-up conditions.
  *
  * @param  p_driver      Pointer to the L6470 driver instance.
  */
void L6470_ResetDevice(sL6470_t* const p_driver)
{
	if (NULL != p_driver)
	{
		memset(p_driver->tx_buffer, 0, 4);
		p_driver->tx_buffer[0] = L6470_ApplicationCommand[RESET_DEVICE].BinaryCode;

		if (NULL != p_driver->SpiTxNow)
		{
			p_driver->SpiTxNow(p_driver->tx_buffer, 1, p_driver->rx_buffer, 0);
		}
	}
}

/**
  * @brief  The SoftStop command causes an immediate deceleration to
  * 		zero speed and a consequent motor stop.
  *
  * @param  p_driver      Pointer to the L6470 driver instance.
  */
void L6470_SoftStop(sL6470_t* const p_driver)
{
	if (NULL != p_driver)
	{
		memset(p_driver->tx_buffer, 0, 4);
		p_driver->tx_buffer[0] = L6470_ApplicationCommand[SOFT_STOP].BinaryCode;

		if (NULL != p_driver->SpiTxNow)
		{
			p_driver->SpiTxNow(p_driver->tx_buffer, 1, p_driver->rx_buffer, 0);
		}
	}
}

/**
  * @brief  The HardStop command causes an immediate motor stop with
  * 		infinite deceleration
  *
  * @param  p_driver      Pointer to the L6470 driver instance.
  */
void L6470_HardStop(sL6470_t* const p_driver)
{
	if (NULL != p_driver)
	{
		memset(p_driver->tx_buffer, 0, 4);
		p_driver->tx_buffer[0] = L6470_ApplicationCommand[HARD_STOP].BinaryCode;

		if (NULL != p_driver->SpiTxNow)
		{
			p_driver->SpiTxNow(p_driver->tx_buffer, 1, p_driver->rx_buffer, 0);
		}
	}
}

/**
  * @brief  The SoftHiZ command disables the power bridges (high impedance state)
  * 		after a	deceleration to zero.
  *
  * @param  p_driver      Pointer to the L6470 driver instance.
  */
void L6470_SoftHiZ(sL6470_t* const p_driver)
{
	if (NULL != p_driver)
	{
		memset(p_driver->tx_buffer, 0, 4);
		p_driver->tx_buffer[0] = L6470_ApplicationCommand[SOFT_HIZ].BinaryCode;

		if (NULL != p_driver->SpiTxNow)
		{
			p_driver->SpiTxNow(p_driver->tx_buffer, 1, p_driver->rx_buffer, 0);
		}
	}
}

/**
  * @brief  The HardHiZ command The HardHiZ command immediately disables the
  * 		power bridges (high impedance state) and raises the HiZ flag
  *
  * @param  p_driver      Pointer to the L6470 driver instance.
  */
void L6470_HardHiZ(sL6470_t* const p_driver)
{
	if (NULL != p_driver)
	{
		memset(p_driver->tx_buffer, 0, 4);
		p_driver->tx_buffer[0] = L6470_ApplicationCommand[HARD_HIZ].BinaryCode;

		if (NULL != p_driver->SpiTxNow)
		{
			p_driver->SpiTxNow(p_driver->tx_buffer, 1, p_driver->rx_buffer, 0);
		}
	}
}

/**
  * @brief  The GetStatus command returns the STATUS register value.
  *
  * @param  p_driver      Pointer to the L6470 driver instance.
  *
  * @retval result        The status register value.
  */
uint16_t L6470_GetStatus(sL6470_t* const p_driver)
{
	uint32_t result = 0;

	if (NULL != p_driver)
	{
		memset(p_driver->tx_buffer, 0, 4);
		memset(p_driver->rx_buffer, 0, 4);
		p_driver->tx_buffer[0] = L6470_ApplicationCommand[GET_STATUS].BinaryCode;

		if (NULL != p_driver->SpiTxNow)
		{
			p_driver->SpiTxNow(p_driver->tx_buffer, 1, p_driver->rx_buffer, 2);
			result = L6470_GetResponse(p_driver, STATUS_REG);
		}
	}

	return (uint16_t)result;
}

int32_t L6470_AbsPos_2_Position(uint32_t const abs_pos)
{
  if (abs_pos > L6470_MAX_POSITION)
    return (abs_pos - (L6470_POSITION_RANGE + 1));
  else
    return (abs_pos);
}


uint32_t L6470_Position_2_AbsPos(int32_t const position)
{
  if ((position >= 0) && (position <= L6470_MAX_POSITION))
    return (position);
  else
  {
    if ((position >= L6470_MIN_POSITION) && (position < 0))
      return (position + (L6470_POSITION_RANGE + 1));
    else
      return (L6470_POSITION_RANGE + 1);        // OVF
  }
}
float L6470_MaxSpeed_2_Step_s(uint16_t const max_speed)
{
  if (max_speed <= L6470_MAX_MAX_SPEED)
    return (max_speed * ((float)15.2588));
  else
    return (0);
}

uint16_t L6470_Step_s_2_MaxSpeed(float const step_s)
{
  if (step_s <= (L6470_MAX_MAX_SPEED * ((float)15.2588)))
    return (uint16_t)(step_s / ((float)15.2588));
  else
    return (0);
}

float L6470_Speed_2_Step_s(uint32_t speed)
{
  return (speed * ((float)14.9012e-3));
}

uint32_t L6470_Step_s_2_Speed(float step_s)
{
  if (step_s <= (L6470_MAX_SPEED * ((float)14.9012e-3)))
    return (uint32_t)(step_s / ((float)14.9012e-3));
  else
    return (0);
}

float L6470_OcdTh_2_mA(uint8_t ocd_th)
{
  if (ocd_th <= L6470_MAX_OCD_TH)
    return ((ocd_th+1) * ((float)375));
  else
    return (0);
}

uint8_t L6470_mA_2_OcdTh(const float mA)
{
  float result;
  float decimal;

  if (mA <= ((L6470_MAX_OCD_TH+1) * ((float)375)))
  {
    result = (mA / ((float)375));
    decimal = result - (uint8_t)result;

    if (decimal < (float)0.5)
      return ((uint8_t)result - 1);
    else
      return ((uint8_t)result);
  }
  else
    return (0);
}

float L6470_StallTh_2_mA(uint8_t stall_th)
{
  if (stall_th <= L6470_MAX_STALL_TH)
    return ((stall_th+1) * ((float)31.25));
  else
    return (0);
}

uint8_t L6470_mA_2_StallTh(const float mA)
{
  float result;
  float decimal;

  if (mA <= ((L6470_MAX_STALL_TH+1) * ((float)31.25)))
  {
    result = (mA / ((float)31.25));
    decimal = result - (uint8_t)result;

    if (decimal < (float)0.5)
      return ((uint8_t)result - 1);
    else
      return ((uint8_t)result);
  }
  else
    return (0);
}

float L6470_Acc_2_Step_s2(uint16_t const acc)
{
  if (acc <= L6470_MAX_ACC)
    return (acc * ((float)1.4552e1));
  else
    return (0);
}

uint16_t L6470_Step_s2_2_Acc(float const step_s2)
{
	if (step_s2 <= (L6470_MAX_ACC * ((float)1.4552e1)))
		return (uint16_t)(step_s2 / ((float)1.4552e1));
	else
		return (0);
}

float L6470_Dec_2_Step_s2(uint16_t const dec)
{
	if (dec <= L6470_MAX_DEC)
		return (dec * ((float)1.4552e1));
	else
		return (0);
}

uint16_t L6470_Step_s2_2_Dec(float const step_s2)
{
  if (step_s2 <= (L6470_MAX_DEC * ((float)1.4552e1)))
	  return (uint16_t)(step_s2 / ((float)1.4552e1));
  else
	  return (0);
}

float L6470_FsSpd_2_Step_s(uint16_t const fs_spd)
{
	if (fs_spd <= L6470_MAX_FS_SPD)
	    return ((fs_spd+0.5) * ((float)15.25));
	else
	    return (0);
}

uint16_t L6470_Step_s_2_FsSpd(float const step_s)
{
	if (step_s <= ((L6470_MAX_FS_SPD+0.5) * ((float)15.25)))
	    return (uint16_t)(((float)(step_s / ((float)15.25)) - (float)0.5));
	else
	    return (0);
}

float L6470_IntSpeed_2_Step_s(uint16_t const int_speed)
{
	if (int_speed <= L6470_MAX_INT_SPEED)
	    return (int_speed * ((float)59.6046e-3));
	else
	    return (0);
}

uint16_t L6470_Step_s_2_IntSpeed(float const step_s)
{
	if (step_s <= (L6470_MAX_INT_SPEED * ((float)59.6046e-3)))
	    return (uint16_t)(step_s / ((float)59.6046e-3));
	else
	    return (0);
}

float L6470_StSlp_2_s_Step(uint8_t const st_slp)
{
    return (st_slp * ((float)1.5686e-5));
}

uint8_t L6470_s_Step_2_StSlp(float const s_step)
{
	if (s_step <= (L6470_MAX_ST_SLP * ((float)1.5686e-5)))
		return (uint8_t)(s_step / ((float)1.5686e-5));
	else
	    return (0);
}

float L6470_FnSlpAcc_2_s_Step(uint8_t const fn_slp_acc)
{
    return (fn_slp_acc * ((float)1.5686e-5));
}

uint8_t L6470_s_Step_2_FnSlpAcc(float const s_step)
{
	if (s_step <= (L6470_MAX_FN_SLP_ACC * ((float)1.5686e-5)))
	    return (uint8_t)(s_step / ((float)1.5686e-5));
	else
	    return (0);
}

float L6470_FnSlpDec_2_s_Step(uint8_t const fn_slp_dec)
{
    return (fn_slp_dec * ((float)1.5686e-5));
}

uint8_t L6470_s_Step_2_FnSlpDec(float const s_step)
{
	if (s_step <= (L6470_MAX_FN_SLP_DEC * ((float)1.5686e-5)))
		return ((uint8_t)(s_step / ((float)1.5686e-5)));
	else
	    return (0);
}

uint32_t L6470_GetResponse(sL6470_t* const p_driver, const eL6470_Register_Address_t param) {

    uint32_t result = 0;

	if (NULL != p_driver)
	{
		for (uint8_t idx = 0; idx < L6470_Register[param].n_bytes; idx++)
		{
			result |= ((uint32_t)p_driver->rx_buffer[idx + 1]) << (8 * (L6470_Register[param].n_bytes - 1 - idx));
		}
    }

    return result;
}

eL6470_MotorStatus_t L6470_GetMotorStatus(sL6470_t * const p_driver)
{
	uint16_t status_reg = L6470_GetStatus(p_driver);

	return (MOTOR_STATUS_MASK & status_reg) >> 5;
}

void L6470_DisableAlarm(sL6470_t* const p_driver, const eL6470_Alarm_t alarm)
{
	uint32_t enabled_alarms = 0x00;

	enabled_alarms = L6470_GetParam(p_driver, ALARM_EN_REG);
	enabled_alarms ^= alarm;
	L6470_SetParam(p_driver, ALARM_EN_REG, enabled_alarms);
}

void L6470_EnableAlarm(sL6470_t* const p_driver, const eL6470_Alarm_t alarm)
{
	uint32_t enabled_alarms = 0x00;

	enabled_alarms = L6470_GetParam(p_driver, ALARM_EN_REG);
	enabled_alarms |= alarm;
	L6470_SetParam(p_driver, ALARM_EN_REG, enabled_alarms);
}

static void L6470_value_2_tx_buf(uint8_t * const p_dest, const uint32_t value, const uint8_t n_bytes)
{
	if (n_bytes == 3)
	{
		p_dest[1] = (value >> 16) & 0xFF;
		p_dest[2] = (value >> 8) & 0xFF;
		p_dest[3] = value & 0xFF;
	}
	if (n_bytes == 2)
	{
		p_dest[1] = (value >> 8) & 0xFF;
		p_dest[2] = value & 0xFF;
		p_dest[3] = 0x00;
	}
	if (n_bytes == 1)
	{
		p_dest[1] = value & 0xFF;
		p_dest[2] = 0x00;
		p_dest[3] = 0x00;
	}
}

