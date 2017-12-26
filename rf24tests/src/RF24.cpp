/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

#include "em_usart.h"
#include "em_gpio.h"
#include "RF24.h"
#include "InitDevice.h"
#include "delay.h"


/****************************************************************************/

uint8_t RF24::write_register(uint8_t reg, const uint8_t* buf, uint8_t len)
{
  uint8_t status;

  beginTransaction();
  status = USART_SpiTransfer(USART1, W_REGISTER | ( REGISTER_MASK & reg ) );
  while ( len-- )
	  USART_SpiTransfer(USART1, *buf++);
  endTransaction();

  return status;
}

/****************************************************************************/

uint8_t RF24::write_register(uint8_t reg, uint8_t value)
{
  uint8_t status;

  beginTransaction();
  status = USART_SpiTransfer(USART1, W_REGISTER | ( REGISTER_MASK & reg ) );
  USART_SpiTransfer(USART1, value);
  endTransaction();

  return status;
}

/****************************************************************************/

bool RF24::begin(void)
{
  uint8_t setup=0;

  // Make sure CS is in high state
  GPIO_PinOutSet(USART1_CS_PORT, USART1_CS_PIN);

  // Must allow the radio time to settle else configuration bits will not necessarily stick.
  // This is actually only required following power up but some settling time also appears to
  // be required after resets too. For full coverage, we'll always assume the worst.
  // Enabling 16b CRC is by far the most obvious case if the wrong timing is used - or skipped.
  // Technically we require 4.5ms + 14us as a worst case. We'll just call it 5ms for good measure.
  // WARNING: Delay is based on P-variant whereby non-P *may* require different timing.
  delay( 5 ) ;

  // Reset NRF_CONFIG and enable 16-bit CRC.
  write_register( NRF_CONFIG, 0x0C ) ;

  // Set 1500uS (minimum for 32B payload in ESB@250KBPS) timeouts, to make testing a little easier
  // WARNING: If this is ever lowered, either 250KBS mode with AA is broken or maximum packet
  // sizes must never be used. See documentation for a more complete explanation.
  setRetries(5,15);

  // Reset value is MAX
  //setPALevel( RF24_PA_MAX ) ;

  // check for connected module and if this is a p nRF24l01 variant
  //
  //if( setDataRate( RF24_250KBPS ) )
  //{
    //p_variant = true ;
  //}
  //setup = read_register(RF_SETUP);
  /*if( setup == 0b00001110 )     // register default for nRF24L01P
  {
    p_variant = true ;
  }*/

  // Then set the data rate to the slowest (and most reliable) speed supported by all
  // hardware.
  setDataRate( RF24_250KBPS ) ;

  // Initialize CRC and request 2-byte (16bit) CRC
  //setCRCLength( RF24_CRC_16 ) ;

  // Disable dynamic payloads, to match dynamic_payloads_enabled setting - Reset value is 0
  toggle_features();
  write_register(FEATURE,0 );
  write_register(DYNPD,0);
  dynamic_payloads_enabled = false;

  // Reset current status
  // Notice reset and flush is the last thing we do
  //write_register(NRF_STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

  // Set up default configuration.  Callers can always change it later.
  // This channel should be universally safe and not bleed over into adjacent
  // spectrum.
  setChannel(76);

  // Flush buffers
  flush_rx();
  flush_tx();

  powerUp(); //Power up by default when begin() is called

  // Enable PTX, do not write CE high so radio will remain in standby I mode ( 130us max to transition to RX or TX instead of 1500us from powerUp )
  // PTX should use only 22uA of power
  write_register(NRF_CONFIG, ( read_register(NRF_CONFIG) ) & ~_BV(PRIM_RX) );

  // if setup is 0 or ff then there was no response from module
  return ( setup != 0 && setup != 0xff );
}

/****************************************************************************/

void RF24::setRetries(uint8_t delay, uint8_t count)
{
 write_register(SETUP_RETR,(delay&0xf)<<ARD | (count&0xf)<<ARC);
}

/****************************************************************************/

bool RF24::setDataRate(uint8_t speed)
{
  bool result = false;
  uint8_t setup = read_register(RF_SETUP) ;

  // HIGH and LOW '00' is 1Mbs - our default
  setup &= ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH)) ;

  //16Mhz Arduino
  txDelay=85;
  if( speed == RF24_250KBPS )
  {
    // Must set the RF_DR_LOW to 1; RF_DR_HIGH (used to be RF_DR) is already 0
    // Making it '10'.
    setup |= _BV( RF_DR_LOW ) ;

    //16Mhz Arduino
	txDelay=155;
  }
  else
  {
    // Set 2Mbs, RF_DR (RF_DR_HIGH) is set 1
    // Making it '01'
    if ( speed == RF24_2MBPS )
    {
      setup |= _BV(RF_DR_HIGH);
      //16Mhz Arduino
	  txDelay=65;
    }
  }
  write_register(RF_SETUP,setup);

  // Verify our result
  if ( read_register(RF_SETUP) == setup )
  {
    result = true;
  }
  return result;
}

/****************************************************************************/

uint8_t RF24::read_register(uint8_t reg, uint8_t* buf, uint8_t len)
{
  uint8_t status;

  beginTransaction();
  status = USART_SpiTransfer(USART1,  R_REGISTER | ( REGISTER_MASK & reg ) );
  while ( len-- ){
    *buf++ = USART_SpiTransfer(USART1, 0xff);
  }
  endTransaction();

  return status;
}

/****************************************************************************/

uint8_t RF24::read_register(uint8_t reg)
{
  uint8_t result;

  beginTransaction();
  USART_SpiTransfer(USART1, R_REGISTER | ( REGISTER_MASK & reg ) );
  result = USART_SpiTransfer(USART1, 0xff);
  endTransaction();

  return result;
}

/****************************************************************************/

  inline void RF24::beginTransaction() {
	GPIO_PinOutClear(USART1_CS_PORT, USART1_CS_PIN);
  }

/****************************************************************************/

  inline void RF24::endTransaction() {
	  GPIO_PinOutSet(USART1_CS_PORT, USART1_CS_PIN);
  }

  /****************************************************************************/

  uint8_t RF24::flush_rx(void)
  {
    return spiTrans( FLUSH_RX );
  }

  /****************************************************************************/

  uint8_t RF24::flush_tx(void)
  {
    return spiTrans( FLUSH_TX );
  }

  /****************************************************************************/

  uint8_t RF24::spiTrans(uint8_t cmd){

    uint8_t status;

    beginTransaction();
    status = USART_SpiTransfer(USART1, cmd );
    endTransaction();

    return status;
  }

  /****************************************************************************/

  void RF24::toggle_features(void)
  {
    beginTransaction();
    USART_SpiTransfer(USART1, ACTIVATE );
    USART_SpiTransfer(USART1, 0x73 );
  	endTransaction();
  }

  /****************************************************************************/

  void RF24::setChannel(uint8_t channel)
  {
    const uint8_t max_channel = 125;
    write_register(RF_CH,rf24_min(channel,max_channel));
  }

  uint8_t RF24::getChannel()
  {
    return read_register(RF_CH);
  }

  /****************************************************************************/

  RF24::RF24(uint16_t _cepin, uint16_t _cspin):
    ce_pin(_cepin), csn_pin(_cspin), p_variant(false),
    payload_size(32), dynamic_payloads_enabled(false), addr_width(5),csDelay(5)//,pipe0_reading_address(0)
  {
    pipe0_reading_address[0]=0;
  }

  /****************************************************************************/

  void RF24::powerDown(void)
  {
    GPIO_PinOutSet(RF24_CE_PORT, RF24_CE_PIN); // Guarantee CE is low on powerDown
    write_register(NRF_CONFIG,read_register(NRF_CONFIG) & ~_BV(PWR_UP));
  }

  /****************************************************************************/

  //Power up now. Radio will not power down unless instructed by MCU for config changes etc.
  void RF24::powerUp(void)
  {
     uint8_t cfg = read_register(NRF_CONFIG);

     // if not powered up then power up and wait for the radio to initialize
     if (!(cfg & _BV(PWR_UP))){
        write_register(NRF_CONFIG, cfg | _BV(PWR_UP));

        // For nRF24L01+ to go from power down mode to TX or RX mode it must first pass through stand-by mode.
  	  // There must be a delay of Tpd2stby (see Table 16.) after the nRF24L01+ leaves power down mode before
  	  // the CEis set high. - Tpd2stby can be up to 5ms per the 1.0 datasheet
        delay(5);
     }
  }
  /****************************************************************************/

  void RF24::openWritingPipe(uint64_t value)
  {
    // Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
    // expects it LSB first too, so we're good.

    write_register(RX_ADDR_P0, reinterpret_cast<uint8_t*>(&value), addr_width);
    write_register(TX_ADDR, reinterpret_cast<uint8_t*>(&value), addr_width);


    //const uint8_t max_payload_size = 32;
    //write_register(RX_PW_P0,rf24_min(payload_size,max_payload_size));
    write_register(RX_PW_P0,payload_size);
  }

  /****************************************************************************/
  void RF24::openWritingPipe(const uint8_t *address)
  {
    // Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
    // expects it LSB first too, so we're good.

    write_register(RX_ADDR_P0,address, addr_width);
    write_register(TX_ADDR, address, addr_width);

    //const uint8_t max_payload_size = 32;
    //write_register(RX_PW_P0,rf24_min(payload_size,max_payload_size));
    write_register(RX_PW_P0,payload_size);
  }

  /******************************************************************/

  //Similar to the previous write, clears the interrupt flags
  bool RF24::write( const void* buf, uint8_t len, const bool multicast )
  {
  	//Start Writing
  	startFastWrite(buf,len,multicast);

  	//Wait until complete or failed
  	#if defined (FAILURE_HANDLING) || defined (RF24_LINUX)
  		uint32_t timer = millis();
  	#endif

  	while( ! ( get_status()  & ( _BV(TX_DS) | _BV(MAX_RT) ))) {
  		#if defined (FAILURE_HANDLING) || defined (RF24_LINUX)
  			if(millis() - timer > 95){
  				errNotify();
  				#if defined (FAILURE_HANDLING)
  				  return 0;
  				#else
  				  delay(100);
  				#endif
  			}
  		#endif
  	}

  	GPIO_PinOutClear(RF24_CE_PORT, RF24_CE_PIN);

  	uint8_t status = write_register(NRF_STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

    //Max retries exceeded
    if( status & _BV(MAX_RT)){
    	flush_tx(); //Only going to be 1 packet int the FIFO at a time using this method, so just flush
    	return 0;
    }
  	//TX OK 1 or 0
    return 1;
  }

  bool RF24::write( const void* buf, uint8_t len ){
  	return write(buf,len,0);
  }

  /****************************************************************************/

  //Per the documentation, we want to set PTX Mode when not listening. Then all we do is write data and set CE high
  //In this mode, if we can keep the FIFO buffers loaded, packets will transmit immediately (no 130us delay)
  //Otherwise we enter Standby-II mode, which is still faster than standby mode
  //Also, we remove the need to keep writing the config register over and over and delaying for 150 us each time if sending a stream of data

  void RF24::startFastWrite( const void* buf, uint8_t len, const bool multicast, bool startTx){ //TMRh20

  	//write_payload( buf,len);
  	write_payload( buf, len,multicast ? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD ) ;
  	if(startTx){
  		GPIO_PinOutSet(RF24_CE_PORT, RF24_CE_PIN);
  	}

  }

  /****************************************************************************/

  uint8_t RF24::write_payload(const void* buf, uint8_t data_len, const uint8_t writeType)
  {
    uint8_t status;
    const uint8_t* current = reinterpret_cast<const uint8_t*>(buf);

     data_len = rf24_min(data_len, payload_size);
     uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;

   #if defined (RF24_LINUX)
  	beginTransaction();
  	uint8_t * prx = spi_rxbuff;
  	uint8_t * ptx = spi_txbuff;
      uint8_t size;
  	size = data_len + blank_len + 1 ; // Add register value to transmit buffer

  	*ptx++ =  writeType;
      while ( data_len-- )
        *ptx++ =  *current++;
      while ( blank_len-- )
  	  *ptx++ =  0;

  	_SPI.transfernb( (char *) spi_txbuff, (char *) spi_rxbuff, size);
  	status = *prx; // status is 1st byte of receive buffer
  	endTransaction();

    #else

    beginTransaction();
    status = USART_SpiTransfer(USART1, writeType );
    while ( data_len-- ) {
    	USART_SpiTransfer(USART1, *current++);
    }
    while ( blank_len-- ) {
    	USART_SpiTransfer(USART1, 0);
    }
    endTransaction();

    #endif

    return status;
  }

  /****************************************************************************/

  uint8_t RF24::get_status(void)
  {
    return spiTrans(NOP);
  }
