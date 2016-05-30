/*
 * THE FOLLOWING FIRMWARE IS PROVIDED: (1) "AS IS" WITH NO WARRANTY; AND 
 * (2)TO ENABLE ACCESS TO CODING INFORMATION TO GUIDE AND FACILITATE CUSTOMER.
 * CONSEQUENTLY, SEMTECH SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
 * OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
 * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 * 
 * Copyright (C) SEMTECH S.A.
 */
/*! 
 * \file       sx1276-Hal.c
 * \brief      SX1276 Hardware Abstraction Layer
 *
 * \version    2.0.B2 
 * \date       Nov 21 2012
 * \author     Miguel Luis
 *
 * Last modified by Miguel Luis on Jun 19 2013
 */
#include <stdint.h>
#include <stdbool.h> 

#include "platform.h"

#if defined( USE_SX1276_RADIO )

#include <string.h>
#include "sx1276-Hal.h"
#include "nrf_gpio.h"
#include "custom_board.h"

//extern SPI_HandleTypeDef hspi1;

void SX1276InitIo( void )
{
  // Do nothing, as main initalized the GPIO SPI and NVIC
}

void SX1276SetReset( uint8_t state )
{

  if( state == RADIO_RESET_ON )
  {
   // GPIO_InitStruct.Pin = SX1276_RST_Pin;
   // GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   // GPIO_InitStruct.Pull = GPIO_NOPULL;
  //  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
   // HAL_GPIO_Init(SX1276_RST_GPIO_Port, &GPIO_InitStruct);
    // Set RESET pin to 0
   // HAL_GPIO_WritePin(SX1276_RST_GPIO_Port, SX1276_RST_Pin, GPIO_PIN_RESET);
		nrf_gpio_pin_clear(SX1276_RST);
		
  }
  else
  {  
    // Configure RESET as input
 //   GPIO_InitStruct.Pin = SX1276_RST_Pin;
   // GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
   // GPIO_InitStruct.Pull = GPIO_NOPULL;
  //  HAL_GPIO_Init(SX1276_RST_GPIO_Port, &GPIO_InitStruct);
		nrf_gpio_cfg_input(SX1276_RST,NRF_GPIO_PIN_NOPULL);
		
  }
}

void SX1276Write( uint8_t addr, uint8_t data )
{
    SX1276WriteBuffer( addr, &data, 1 );
}

void SX1276Read( uint8_t addr, uint8_t *data )
{
    SX1276ReadBuffer( addr, data, 1 );
}

void SX1276WriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
  uint8_t txBuf[130] = {0};
  uint8_t rxBuf[130] = {0};

  //NSS = 0;
  nrf_gpio_pin_clear(SX1276_NSS);
	
  
  txBuf[0] = addr | 0x80;
  memcpy(txBuf+1, buffer, size);
 // HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(&hspi1, txBuf, rxBuf, size+1, 1000);
  
  //NSS = 1;
 // HAL_GPIO_WritePin(SX1276_NSS_GPIO_Port, SX1276_NSS_Pin, GPIO_PIN_SET);
	nrf_gpio_pin_set(SX1276_NSS);
}

void SX1276ReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
  uint8_t txBuf[130] = {0};
  uint8_t rxBuf[130] = {0};

  //NSS = 0;
 // HAL_GPIO_WritePin(SX1276_NSS_GPIO_Port, SX1276_NSS_Pin, GPIO_PIN_RESET);
  nrf_gpio_pin_clear(SX1276_NSS);
	
	
  txBuf[0] = addr & 0x7F;
 // HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(&hspi1, txBuf, rxBuf, size+1, 1000);
  
  //NSS = 1;
//  HAL_GPIO_WritePin(SX1276_NSS_GPIO_Port, SX1276_NSS_Pin, GPIO_PIN_SET);
  nrf_gpio_pin_set(SX1276_NSS);
	
  memcpy(buffer, rxBuf+1, size);
}

void SX1276WriteFifo( uint8_t *buffer, uint8_t size )
{
    SX1276WriteBuffer( 0, buffer, size );
}

void SX1276ReadFifo( uint8_t *buffer, uint8_t size )
{
    SX1276ReadBuffer( 0, buffer, size );
}

inline uint8_t SX1276ReadDio0( void )
{
 // return (HAL_GPIO_ReadPin(SX1276_DIO0_GPIO_Port, SX1276_DIO0_Pin) == GPIO_PIN_SET);
	   
	return (nrf_gpio_pin_read(SX1276_DIO0));

}


inline void SX1276WriteRxTx( uint8_t txEnable )
{
  if( txEnable != 0 )
  {
    //IoePinOn( FEM_CTX_PIN );
    //GPIO_WriteBit( RXTX_IOPORT, RXTX_PIN, Bit_SET );
    //IoePinOff( FEM_CPS_PIN );
  }
  else
  {
    //IoePinOff( FEM_CTX_PIN );
    //GPIO_WriteBit( RXTX_IOPORT, RXTX_PIN, Bit_RESET );
    //IoePinOn( FEM_CPS_PIN );
  }
}

#endif // USE_SX1276_RADIO