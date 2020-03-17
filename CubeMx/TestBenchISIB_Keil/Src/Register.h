/***************************************************************************
* Includes Directives
***************************************************************************/
#include "stdint.h"
#include "Module_Defines.h"

/***************************************************************************
* Constant declarations
***************************************************************************/

/** TM Table register ID*/
#define C_TM_TB_MODE		0
#define C_TM_CR_MOT			1
#define C_TM_U_MOT			2
#define C_TM_I_MOT			3
#define C_TM_SP_MOT			4
#define C_TM_PWM_MOT		5
#define C_TM_U_BRAKE		6
#define C_TM_I_BRAKE		7
#define C_TM_PWM_BRAKE	8

/** ADC MEASURES Table IDX*/
#define C_ADCTAB_UMOTOR		0
#define C_ADCTAB_IMOTOR		1
#define C_ADCTAB_UBRAKE		2
#define C_ADCTAB_IBRAKE		3

/** TELECOMMAND Table register ID */
#define C_TC_CMD_COUNT_ID		0
#define C_TC_CMD_ID					1
#define C_TC_PARAM_1_ID			2
#define C_TC_PARAM_2_ID			3

/** TELECOMMAND Table register ID */
#define C_TC_SET_MODE				0x00
#define C_TC_MAN_SET_U_MOT  0x01
#define C_TC_MAN_SET_I_MOT  0x02
#define C_TC_MAN_SET_SP_MOT 0x03
#define C_TC_MAN_SET_CR_MOT 0x04

/** Number of Command of each Type, HEMPT-PPU -BB-SRS-COM-012 */
#define C_LAST_P_CMD_ID		31
#define C_LAST_C_CMD_ID		2
#define C_LAST_R_CMD_ID		6


/***************************************************************************
* Variables declarations
***************************************************************************/

/** @brief TM Table register*/
extern uint16_t Table_Tm_Reg[C_TM_TABLE_SIZE];
/** @brief TELECOMMAND Table register */
extern volatile uint16_t Table_Tc_Reg[C_TC_TABLE_SIZE];

/***************************************************************************
* Type definitions
***************************************************************************/
/** @brief Enum Type to define the PPU_MODE */


/** @brief Parameter information */
typedef struct {
	uint16_t 	Register_Index;
	uint16_t	Default_Value;
} T_PARAMETER_INFO;



