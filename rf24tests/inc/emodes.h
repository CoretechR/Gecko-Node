#ifdef __cplusplus
extern "C" {
#endif
/*
 * emodes.h
 *
 *  Created on: 18.08.2017
 *      Author: Maximilian
 */

#ifndef INC_EMODES_H_
#define INC_EMODES_H_

/***************************************************************************//**
 * @file emodes.h
 * @brief Zero Gecko energy mode setups (See Data Sheet Table 3.3), header file
 * @version 5.2.2
 *******************************************************************************
 * # License
 * <b>Copyright 2016 Silicon Labs, Inc. http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/

#include "em_cmu.h"

/* Enter energy mode functions */
void em_EM0_Hfxo(void);

void em_EM0_Hfrco(CMU_HFRCOBand_TypeDef band);

void em_EM1_Hfxo(void);

void em_EM1_Hfrco(CMU_HFRCOBand_TypeDef band);

void em_EM2_LfrcoRTC(void);

void em_EM3_Ulfrco(void);

void em_EM4(void);


#endif /* INC_EMODES_H_ */

#ifdef __cplusplus
}
#endif
