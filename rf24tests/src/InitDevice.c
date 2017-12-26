// $[Library includes]
#include "em_system.h"
#include "em_emu.h"
#include "em_cmu.h"
#include "em_device.h"
#include "em_chip.h"
#include "em_assert.h"
#include "em_gpio.h"
#include "em_i2c.h"
#include "em_rtc.h"
#include "em_usart.h"
// [Library includes]$

extern void enter_DefaultMode_from_RESET(void) {
	// $[Config Calls]
	HFXO_enter_DefaultMode_from_RESET();
	LFXO_enter_DefaultMode_from_RESET();
	CMU_enter_DefaultMode_from_RESET();
	RTC_enter_DefaultMode_from_RESET();
	USART1_enter_DefaultMode_from_RESET();
	I2C0_enter_DefaultMode_from_RESET();
	PORTIO_enter_DefaultMode_from_RESET();
	// [Config Calls]$
}

extern void HFXO_enter_DefaultMode_from_RESET(void) {

	// $[HFXO]
	CMU->CTRL = (CMU->CTRL & ~_CMU_CTRL_HFXOMODE_MASK) | CMU_CTRL_HFXOMODE_XTAL;

	CMU->CTRL = (CMU->CTRL & ~_CMU_CTRL_HFXOBOOST_MASK)
			| CMU_CTRL_HFXOBOOST_50PCENT;

	SystemHFXOClockSet(32000000);
	// [HFXO]$

}

extern void LFXO_enter_DefaultMode_from_RESET(void) {

	// $[Use oscillator source]
	CMU->CTRL = (CMU->CTRL & ~_CMU_CTRL_LFXOMODE_MASK) | CMU_CTRL_LFXOMODE_XTAL;
	// [Use oscillator source]$

	// $[LFXO Boost Percent]
	CMU->CTRL = (CMU->CTRL & ~_CMU_CTRL_LFXOBOOST_MASK)
			| CMU_CTRL_LFXOBOOST_100PCENT;
	// [LFXO Boost Percent]$

	// $[REDLFXO Boost]
	// [REDLFXO Boost]$

}

extern void CMU_enter_DefaultMode_from_RESET(void) {

	// $[LFXO enable]
	CMU_OscillatorEnable(cmuOsc_LFXO, true, true);
	// [LFXO enable]$

	// $[HFXO enable]
	CMU_OscillatorEnable(cmuOsc_HFXO, true, true);
	// [HFXO enable]$

	// $[LFACLK Setup]
	/* Select LFXO as clock source for LFACLK */
	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);

	// [LFACLK Setup]$

	// $[High Frequency Clock select]
	/* Using HFXO as high frequency clock, HFCLK */
	CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

	/* Enable peripheral clock */
	CMU_ClockEnable(cmuClock_HFPER, true);

	// [High Frequency Clock select]$

	// $[LF clock tree setup]
	/* Enable LF clocks */
	CMU_ClockEnable(cmuClock_CORELE, true);
	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
	// [LF clock tree setup]$
	// $[Peripheral Clock enables]
	/* Enable clock for I2C0 */
	CMU_ClockEnable(cmuClock_I2C0, true);

	/* Enable clock for RTC */
	CMU_ClockEnable(cmuClock_RTC, true);

	/* Enable clock for USART1 */
	CMU_ClockEnable(cmuClock_USART1, true);

	/* Enable clock for GPIO by default */
	CMU_ClockEnable(cmuClock_GPIO, true);

	// [Peripheral Clock enables]$

}

extern void ADC0_enter_DefaultMode_from_RESET(void) {

	// $[ADC_Init]
	// [ADC_Init]$

	// $[ADC_InitSingle]
	// [ADC_InitSingle]$

	// $[ADC_InitScan]
	// [ADC_InitScan]$

}

extern void ACMP0_enter_DefaultMode_from_RESET(void) {

	// $[ACMP Initialization]
	// [ACMP Initialization]$

	// $[ACMP Channel config]
	// [ACMP Channel config]$

}

extern void IDAC0_enter_DefaultMode_from_RESET(void) {

	// $[IDAC Initialization]
	// [IDAC Initialization]$

	// $[IDAC optional configurations]
	// [IDAC optional configurations]$

	// $[IDAC enable]
	// [IDAC enable]$

}

extern void RTC_enter_DefaultMode_from_RESET(void) {

	// $[RTC_Init]
	RTC_Init_TypeDef init = RTC_INIT_DEFAULT;

	init.debugRun = 0;
	init.comp0Top = 0;

	RTC_Init(&init);
	// [RTC_Init]$

}

extern void USART1_enter_DefaultMode_from_RESET(void) {

	// $[USART_InitAsync]
	// [USART_InitAsync]$

	// $[USART_InitSync]
	USART_InitSync_TypeDef initsync = USART_INITSYNC_DEFAULT;

	initsync.baudrate = 1200000;
	initsync.databits = usartDatabits8;
	initsync.master = 1;
	initsync.msbf = 1;
	initsync.clockMode = usartClockMode0;
#if defined( USART_INPUT_RXPRS ) && defined( USART_TRIGCTRL_AUTOTXTEN )
	initsync.prsRxEnable = 0;
	initsync.prsRxCh = 0;
	initsync.autoTx = 0;
#endif

	USART_InitSync(USART1, &initsync);
	// [USART_InitSync]$

	// $[USART_InitPrsTrigger]
	USART_PrsTriggerInit_TypeDef initprs = USART_INITPRSTRIGGER_DEFAULT;

	initprs.rxTriggerEnable = 0;
	initprs.txTriggerEnable = 0;
	initprs.prsTriggerChannel = usartPrsTriggerCh0;

	USART_InitPrsTrigger(USART1, &initprs);
	// [USART_InitPrsTrigger]$

}

extern void LEUART0_enter_DefaultMode_from_RESET(void) {

	// $[LEUART0 initialization]
	// [LEUART0 initialization]$

}

extern void VCMP_enter_DefaultMode_from_RESET(void) {

	// $[VCMP_Init]
	// [VCMP_Init]$

}

extern void WDOG_enter_DefaultMode_from_RESET(void) {

	// $[CMU_ClockEnable]
	// [CMU_ClockEnable]$

	// $[CMU_OscillatorEnable]
	// [CMU_OscillatorEnable]$

	// $[WDOG_Init]
	// [WDOG_Init]$

}

extern void I2C0_enter_DefaultMode_from_RESET(void) {

	// $[I2C0 initialization]
	I2C_Init_TypeDef init = I2C_INIT_DEFAULT;

	init.enable = 1;
	init.master = 1;
	init.freq = I2C_FREQ_FAST_MAX;
	init.clhr = i2cClockHLRAsymetric;
	I2C_Init(I2C0, &init);
	// [I2C0 initialization]$

}

extern void TIMER0_enter_DefaultMode_from_RESET(void) {

	// $[TIMER0 initialization]
	// [TIMER0 initialization]$

	// $[TIMER0 CC0 init]
	// [TIMER0 CC0 init]$

	// $[TIMER0 CC1 init]
	// [TIMER0 CC1 init]$

	// $[TIMER0 CC2 init]
	// [TIMER0 CC2 init]$

}

extern void TIMER1_enter_DefaultMode_from_RESET(void) {

	// $[TIMER1 initialization]
	// [TIMER1 initialization]$

	// $[TIMER1 CC0 init]
	// [TIMER1 CC0 init]$

	// $[TIMER1 CC1 init]
	// [TIMER1 CC1 init]$

	// $[TIMER1 CC2 init]
	// [TIMER1 CC2 init]$

}

extern void PCNT0_enter_DefaultMode_from_RESET(void) {

	// $[PCNT0 initialization]
	// [PCNT0 initialization]$

}

extern void PRS_enter_DefaultMode_from_RESET(void) {

	// $[PRS initialization]
	// [PRS initialization]$

}

extern void PORTIO_enter_DefaultMode_from_RESET(void) {

	// $[Port A Configuration]

	/* Pin PA0 is configured to Push-pull */
	GPIO->P[0].MODEL = (GPIO->P[0].MODEL & ~_GPIO_P_MODEL_MODE0_MASK)
			| GPIO_P_MODEL_MODE0_PUSHPULL;
	// [Port A Configuration]$

	// $[Port B Configuration]
	// [Port B Configuration]$

	// $[Port C Configuration]

	/* Pin PC8 is configured to Input enabled with filter */
	GPIO->P[2].DOUT |= (1 << 8);
	GPIO->P[2].MODEH = (GPIO->P[2].MODEH & ~_GPIO_P_MODEH_MODE8_MASK)
			| GPIO_P_MODEH_MODE8_INPUT;

	/* Pin PC9 is configured to Input enabled with filter */
	GPIO->P[2].DOUT |= (1 << 9);
	GPIO->P[2].MODEH = (GPIO->P[2].MODEH & ~_GPIO_P_MODEH_MODE9_MASK)
			| GPIO_P_MODEH_MODE9_INPUT;

	/* Pin PC10 is configured to Push-pull */
	GPIO->P[2].MODEH = (GPIO->P[2].MODEH & ~_GPIO_P_MODEH_MODE10_MASK)
			| GPIO_P_MODEH_MODE10_PUSHPULL;

	/* Pin PC11 is configured to Push-pull */
	GPIO->P[2].MODEH = (GPIO->P[2].MODEH & ~_GPIO_P_MODEH_MODE11_MASK)
			| GPIO_P_MODEH_MODE11_PUSHPULL;

	/* Pin PC14 is configured to Push-pull */
	GPIO->P[2].MODEH = (GPIO->P[2].MODEH & ~_GPIO_P_MODEH_MODE14_MASK)
			| GPIO_P_MODEH_MODE14_PUSHPULL;

	/* Pin PC15 is configured to Push-pull */
	GPIO->P[2].MODEH = (GPIO->P[2].MODEH & ~_GPIO_P_MODEH_MODE15_MASK)
			| GPIO_P_MODEH_MODE15_PUSHPULL;
	// [Port C Configuration]$

	// $[Port D Configuration]

	/* Pin PD6 is configured to Input enabled */
	GPIO->P[3].MODEL = (GPIO->P[3].MODEL & ~_GPIO_P_MODEL_MODE6_MASK)
			| GPIO_P_MODEL_MODE6_INPUT;

	/* Pin PD7 is configured to Push-pull */
	GPIO->P[3].MODEL = (GPIO->P[3].MODEL & ~_GPIO_P_MODEL_MODE7_MASK)
			| GPIO_P_MODEL_MODE7_PUSHPULL;
	// [Port D Configuration]$

	// $[Port E Configuration]

	/* Pin PE12 is configured to Open-drain with pull-up and filter */
	GPIO->P[4].MODEH = (GPIO->P[4].MODEH & ~_GPIO_P_MODEH_MODE12_MASK)
			| GPIO_P_MODEH_MODE12_WIREDANDPULLUPFILTER;

	/* Pin PE13 is configured to Open-drain with pull-up and filter */
	GPIO->P[4].MODEH = (GPIO->P[4].MODEH & ~_GPIO_P_MODEH_MODE13_MASK)
			| GPIO_P_MODEH_MODE13_WIREDANDPULLUPFILTER;
	// [Port E Configuration]$

	// $[Port F Configuration]
	// [Port F Configuration]$

	// $[Route Configuration]

	/* Module I2C0 is configured to location 6 */
	I2C0->ROUTE = (I2C0->ROUTE & ~_I2C_ROUTE_LOCATION_MASK)
			| I2C_ROUTE_LOCATION_LOC6;

	/* Enable signals SCL, SDA */
	I2C0->ROUTE |= I2C_ROUTE_SCLPEN | I2C_ROUTE_SDAPEN;

	/* Module USART1 is configured to location 3 */
	USART1->ROUTE = (USART1->ROUTE & ~_USART_ROUTE_LOCATION_MASK)
			| USART_ROUTE_LOCATION_LOC3;

	/* Enable signals CLK, RX, TX */
	USART1->ROUTE |= USART_ROUTE_CLKPEN | USART_ROUTE_RXPEN | USART_ROUTE_TXPEN;
	// [Route Configuration]$

}

