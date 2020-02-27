/***************************************************************************
* Includes Directives
***************************************************************************/
#include "stdint.h"
#include "Module_Defines.h"

/***************************************************************************
* Constant declarations
***************************************************************************//** TM Table register ID*/
#define C_TM_TB_MODE		0
#define C_TM_CR_MOT			1
#define C_TM_U_MOT			2
#define C_TM_I_MOT			3
#define C_TM_SP_MOT			4
#define C_TM_PWM_MOT		5
#define C_TM_U_BRAKE		6
#define C_TM_I_BRAKE		7
#define C_TM_PWM_BRAKE	8

#define C_TM_ANODE_PROTECTIONS_VECTOR	19


#define C_ADCTAB_LOADCEL	0
#define C_ADCTAB_165REF		1
#define C_ADCTAB_UMOTOR		2
#define C_ADCTAB_IMOTOR		3
#define C_ADCTAB_UBRAKE		4
#define C_ADCTAB_IBRAKE		5
/***************************************************************************
* Variables declarations
***************************************************************************/

/** @brief TM Table register*/
extern uint16_t Table_Tm_Reg[C_TM_TABLE_SIZE];

/***************************************************************************
* Type definitions
***************************************************************************/
/** @brief Enum Type to define the PPU_MODE */


/** @brief Parameter information */
typedef struct {
	uint16_t 	Register_Index;
	uint16_t	Default_Value;
} T_PARAMETER_INFO;



