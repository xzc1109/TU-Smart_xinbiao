/*
 * stm.h
 *
 *  Created on: May 10, 2018
 *      Author: yqye
 */

#ifndef STM_H_
#define STM_H_

#include "MPC5604C.h"

void STM_Init(uint8_t system_fre,uint8_t target_fre);

#define STM__get_current_time() (STM.CNT.R)


#endif /* STM_H_ */
