/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
#ifndef CUSTOM_BOARD_H
#define CUSTOM_BOARD_H

// LEDs definitions for Custom Board
//#define LEDS_NUMBER    5
//#define LED_RF             29
//#define LED_PWR_4          2
//#define LED_PWR_3          0
//#define LED_PWR_1          5
//#define LED_PWR_2          30

// Buttons definitions for Custom Board
//#define BUTTONS_NUMBER    4
//#define BUTTON_PULL       NRF_GPIO_PIN_PULLDOWN

//#define BUTTON_PWR        16
//#define BQ24_PG           8
//#define BQ24_STAT         9
//#define BQ24_INT          10

// These are control output pin
//#define CTRL_PWR_HOLD           7
//#define CTRL_STM32_RST          28
//#define CTRL_STM32_BOOT0        15
//#define CTRL_PWR_TRIGGER_DAT    21
//#define CTRL_PWR_TRIGGER_CLK    25
//#define CTRL_ADC_GND            24
//#define CTRL_BQ24_PSEL          6
//#define CTRL_BQ24_OTG           11
//#define CTRL_BQ24_CE            14

// ADC input pin
//#define ADC_BAT                 1

//// UART
//#define RX_PIN_NUMBER           4
//#define TX_PIN_NUMBER           3
//#define CTS_PIN_NUMBER          13
//#define RTS_PIN_NUMBER          12
//#define HWFC                    false


#define SX1276_RST     17
#define SX1276_NSS     12 
#define SX1276_DIO0    16
#define SX1276_EXPA    11	

//static __INLINE void nrf_gpio_cfg_disconnect(uint32_t pin_number)
//{
//    /*lint -e{845} // A zero has been given as right argument to operator '|'" */
//    NRF_GPIO->PIN_CNF[pin_number] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
//                                        | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
//                                        | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
//                                        | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
//                                        | (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);
//}

#endif // CUSTOM_BOARD_H
