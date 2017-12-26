#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_i2c.h"
#include "em_usart.h"
#include "em_rtc.h"
#include "InitDevice.h"
#include "nRF24L01.h"
#include "RF24.h"
#include "delay.h"

/* Defines*/
#define LFRCO_FREQUENCY                 32768

#define HDC1080_ADDRESS       0x40
#define CMD_ARRAY_SIZE        1
#define DATA_ARRAY_SIZE       10

const uint8_t slaveAddress[5] = {'R','x','A','A','A'};
float temp = 0;


// RTC Interrupt Handler, clears the flag.
void RTC_IRQHandler(void)
{
  RTC_IntClear(RTC_IFC_COMP0); // Clear interrupt source
}


void setupRtc(int msBetweenWakeup)
{
	/* Starting LFRCO and waiting until it is stable */
	  CMU_OscillatorEnable(cmuOsc_LFRCO, true, true);

	  /* Enabling clock to the interface of the low energy modules */
	  CMU_ClockEnable(cmuClock_CORELE, true);

	CMU_ClockSelectSet(cmuClock_LFA,cmuSelect_LFRCO);
	  CMU_ClockEnable(cmuClock_RTC, true);


  RTC_Init_TypeDef rtcInit = RTC_INIT_DEFAULT;

  /* Setting the compare value of the RTC */
  RTC_CompareSet(0, (((LFRCO_FREQUENCY * msBetweenWakeup) / 1000)-1));

  /* Enabling Interrupt from RTC */
  RTC_IntEnable(RTC_IEN_COMP0);
  NVIC_ClearPendingIRQ(RTC_IRQn);
  NVIC_EnableIRQ(RTC_IRQn);

  /* Initialize the RTC */
  RTC_Init(&rtcInit);

}

// Globals for persistent storage
uint8_t cmd_array[CMD_ARRAY_SIZE];
uint8_t data_array[DATA_ARRAY_SIZE];

// Used by the read_register and write_register functions
// data_array is read data for WRITE_READ and tx2 data for WRITE_WRITE
void i2c_transfer(uint16_t device_addr, uint8_t cmd_array[], uint8_t data_array[], uint16_t cmd_len, uint16_t data_len, uint8_t flag)
{
      // Transfer structure
      I2C_TransferSeq_TypeDef i2cTransfer;

      // Initialize I2C transfer
      I2C_TransferReturn_TypeDef result;
      i2cTransfer.addr          = device_addr << 1;   // This shift is important!;
      i2cTransfer.flags         = flag;
      i2cTransfer.buf[0].data   = cmd_array;
      i2cTransfer.buf[0].len    = cmd_len;

      // Note that WRITE_WRITE this is tx2 data
      i2cTransfer.buf[1].data   = data_array;
      i2cTransfer.buf[1].len    = data_len;

      // Set up the transfer
      result = I2C_TransferInit(I2C0, &i2cTransfer);

      // Do it until the transfer is done
      while (result != i2cTransferDone)
      {
            if (result != i2cTransferInProgress)
            {
                  break;
            }
            result = I2C_Transfer(I2C0);
      }
}

struct nodeData{
  float temp;
  float humd;
  char batt;
  char id;
};
nodeData nodeSend;

int main(void)
{
  /* Chip errata */
  CHIP_Init();
  enter_DefaultMode_from_RESET();


  RF24 radio(0, 0);

  //delay(3000);



  radio.begin();
  radio.openWritingPipe(slaveAddress);

  /* Infinite loop */
  while (1) {

	  //GPIO_PinModeSet(LED0_PORT, LED0_PIN, gpioModePushPull, GPIO_PinInGet(PB0_PORT, PB0_PIN));
	  //GPIO_PinModeSet(LED1_PORT, LED1_PIN, gpioModePushPull, GPIO_PinInGet(PB1_PORT, PB1_PIN));
	  cmd_array[0] = 0x00;
  	  data_array[0] = 0x00;
   	  i2c_transfer(HDC1080_ADDRESS, cmd_array, data_array, 1, 0, I2C_FLAG_WRITE);
   	  setupRtc(40);
   	  EMU_EnterEM2(false);
   	  i2c_transfer(HDC1080_ADDRESS, cmd_array, data_array, 4, 0, I2C_FLAG_READ);
   	  float temp = (cmd_array[0] << 8 | cmd_array[1]) * 165.0 / 65536.0 - 40.0;
   	  float humd = 100.0*((unsigned int)(cmd_array[2] << 8 | cmd_array[3]) / 65536.0);
   	  cmd_array[0] = 0x02;
   	  i2c_transfer(HDC1080_ADDRESS, cmd_array, data_array, 1, 2, I2C_FLAG_WRITE_READ);
   	  char battBit = '0';
   	  if((data_array[0] & 0b00010000) != 0) battBit = '1';

   	  nodeSend.temp = temp;
   	  nodeSend.humd = humd;
   	  nodeSend.batt = battBit;
   	  nodeSend.id = 'A';

 	  radio.powerUp();
  	  //radio.write(&temp, sizeof(float));
  	  radio.write(&nodeSend, sizeof(nodeSend));


	  radio.powerDown();
	  //i2c_read_register(0x00);


	  //GPIO_PinModeSet(RF24_CE_PORT, RF24_CE_PIN, gpioModePushPull, 0);
	  //GPIO_PinModeSet(RF24_IRQ_PORT, RF24_IRQ_PIN, gpioModeInput, 0);
	  //GPIO_PinModeSet(LED0_PORT, LED0_PIN, gpioModePushPull, 0);
	  //GPIO_PinModeSet(USART1_CLK_PORT, USART1_CLK_PIN, gpioModePushPull, 0);
	  GPIO_PinModeSet(USART1_CS_PORT, USART1_CS_PIN, gpioModePushPull, 0);
	  //GPIO_PinModeSet(USART1_RX_PORT, USART1_RX_PIN, gpioModePushPull, 0);
	  //GPIO_PinModeSet(USART1_TX_PORT, USART1_TX_PIN, gpioModePushPull, 0);
	  //GPIO_PinModeSet(I2C0_SCL_PORT, I2C0_SCL_PIN, gpioModePushPull, 1);
	  //GPIO_PinModeSet(I2C0_SDA_PORT, I2C0_SDA_PIN, gpioModePushPull, 1);

	  setupRtc(10*60000); // sleep for 10 m
	  EMU_EnterEM2(false);
	  /**************************   Wake-up happens here   *************************/



  }
}
