#ifndef ADS1115_H_DEVCUBE
#define ADS1115_H_DEVCUBE

/*
 * @file ADS1115.h
 *
 *@author Written by Mohammed Asim Merchant
 *		 Created on 14/03/2019
 *
 *@Note	PIN CONFIGURATION
 *		ADS1115 SCL				---		I2C_SCL = PINB2
 *		ADS1115 SDA				---		I2C_SDA = PINB3
 *		ADS1115 ADDR			---		GND
 *		ADS1115 ALERT/DDRY		---		any I/O = PIND3
 *
 * @License MIT License
 *
 * Copyright (c) 2019 Mohammed Asim Merchant
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.

 *
 */

#include <avr/io.h>
#include <util/delay.h>
#include "I2CLib.h"

/************************************************************************/
/* ADS1115 ADDR															*/
/************************************************************************/

#define ADS1115_ADDR_GND (0b1001000)
#define ADS1115_ADDR_VDD (0b1001001)
#define ADS1115_ADDR_SDA (0b1001010)
#define ADS1115_ADDR_SCL (0b1001011)

/************************************************************************/
/* ADS1115 REGISTER														*/
/************************************************************************/

#define ADS1115_REG_CONVERSION	0x00
#define ADS1115_REG_CONFIG		0x01
#define ADS1115_REG_LO_THRESH	0x02
#define ADS1115_REG_HI_THRESH	0x03

/************************************************************************/
/* ADS1115 CONFIG REGISTER BIT NR										*/
/************************************************************************/

#define ADS1115_OS 			15
#define ADS1115_IMUX2 		14
#define ADS1115_IMUX1 		13
#define ADS1115_IMUX0 		12
#define ADS1115_PGA2 		11
#define ADS1115_PGA1 		10
#define ADS1115_PGA0 		9
#define ADS1115_MODE 		8
#define ADS1115_DR2 		7
#define ADS1115_DR1 		6
#define ADS1115_DR0 		5
#define ADS1115_COMP_MODE 	4
#define ADS1115_COMP_POL 	3
#define ADS1115_COMP_LAT 	2
#define ADS1115_COMP_QUE1 	1
#define ADS1115_COMP_QUE0	0

/************************************************************************/
/* ADS1115 CONFIG REGISTER BITS											*/
/************************************************************************/

// Bit 0,1
#define ADS1115_COMP_QUE_CON1			(0x0000)
#define ADS1115_COMP_QUE_CON2			(0x0001)
#define ADS1115_COMP_QUE_CON4			(0x0002)
#define ADS1115_COMP_QUE_DIS			(0x0003)

// Bit 2
#define ADS1115_COMP_LAT_NonLatching	(0x0000)
#define ADS1115_COMP_LAT_Latching		(0x0004)

// Bit 3
#define ADS1115_COMP_POL_3_ACTIVELOW	(0x0000)
#define ADS1115_COMP_POL_3_ACTIVEHIGH	(0x0008)

// Bit 4
#define ADS1115_COMP_MODE_TRADITIONAL	(0x0000)
#define ADS1115_COMP_MODE_WINDOWCOMP	(0x0010)

// Bit 5,6,7
#define ADS1115_DR_8SPS					(0x0000)
#define ADS1115_DR_16SPS				(0x0020)
#define ADS1115_DR_32SPS				(0x0040)
#define ADS1115_DR_64SPS				(0x0060)
#define ADS1115_DR_128SPS				(0x0080)
#define ADS1115_DR_250SPS				(0x00A0)
#define ADS1115_DR_475SPS				(0x00C0)
#define ADS1115_DR_860SPS				(0x00E0)

// Bit 8
#define ADS1115_MODE_CONTINUOUS			(0x0000)
#define ADS1115_MODE_SINGLE				(0x0100) 

// Bit 9,10,11
#define ADS1115_PGA_6_144				(0x0000)
#define ADS1115_PGA_4_096				(0x0200)
#define ADS1115_PGA_2_048				(0x0400)
#define ADS1115_PGA_1_024				(0x0600)
#define ADS1115_PGA_0_512				(0x0800)
#define ADS1115_PGA_0_256				(0x0A00)
#define ADS1115_PGA_0_256_2				(0x0C00)
#define ADS1115_PGA_0_256_3				(0x0E00)

// Bit 12,13,14
#define ADS1115_MUX_AIN0_AIN1			(0x0000)
#define ADS1115_MUX_AIN0_AIN3			(0x1000)
#define ADS1115_MUX_AIN1_AIN3			(0x2000)
#define ADS1115_MUX_AIN2_AIN3			(0x3000)
#define ADS1115_MUX_AIN0_GND			(0x4000)
#define ADS1115_MUX_AIN1_GND			(0x5000)
#define ADS1115_MUX_AIN2_GND			(0x6000)
#define ADS1115_MUX_AIN3_GND			(0x7000)

// Bit 15
#define ADS1115_OS_OFF					(0x0000)
#define ADS1115_OS_SINGLE				(0x8000)

/************************************************************************/
/* ADS1115 ENUMS				                                        */
/************************************************************************/

typedef enum
{
	FSR_6_144		= ADS1115_PGA_6_144,
	FSR_4_096       = ADS1115_PGA_4_096,
	FSR_2_048       = ADS1115_PGA_2_048,
	FSR_1_024       = ADS1115_PGA_1_024,
	FSR_0_512       = ADS1115_PGA_0_512,
	FSR_0_256		= ADS1115_PGA_0_256,
	FSR_0_256_2     = ADS1115_PGA_0_256_2,
	FSR_0_256_3     = ADS1115_PGA_0_256_3
} ads1115_fsr_gain;

typedef enum
{
	DATARATE_8SPS		= ADS1115_DR_8SPS,
	DATARATE_16SPS		= ADS1115_DR_16SPS,
	DATARATE_32SPS		= ADS1115_DR_32SPS,
	DATARATE_64SPS		= ADS1115_DR_64SPS,
	DATARATE_128SPS		= ADS1115_DR_128SPS,
	DATARATE_250SPS		= ADS1115_DR_250SPS,
	DATARATE_475SPS		= ADS1115_DR_475SPS,
	DATARATE_860SPS		= ADS1115_DR_860SPS
} ads1115_datarate;

#define I2C_WRITE_BIT		0
#define I2C_READ_BIT		1

/************************************************************************/
/* ADS1115 FUNCTIONS			                                        */
/************************************************************************/

uint8_t ads1115_write_register(uint8_t addr, uint8_t reg, uint16_t data);
uint16_t ads1115_read_register(uint8_t addr, uint8_t reg);
uint16_t ads1115_readADC_SingleEnded(uint8_t addr, uint8_t channel, ads1115_datarate dr, ads1115_fsr_gain gain);
int16_t ads1115_readADC_Diff_A0_1(uint8_t addr, ads1115_datarate dr, ads1115_fsr_gain gain);
int16_t ads1115_readADC_Diff_A0_3(uint8_t addr, ads1115_datarate dr, ads1115_fsr_gain gain);
int16_t ads1115_readADC_Diff_A1_3(uint8_t addr, ads1115_datarate dr, ads1115_fsr_gain gain);
int16_t ads1115_readADC_Diff_A2_3(uint8_t addr, ads1115_datarate dr, ads1115_fsr_gain gain);

/**************************************************************************/
/*!
    @brief  write register of ADC
*/
/**************************************************************************/
uint8_t ads1115_write_register(uint8_t addr, uint8_t reg, uint16_t data)
{
	I2C_Start();
	I2C_Write((addr << 1) + I2C_WRITE_BIT);
	I2C_Write((uint8_t)reg);
	I2C_Write((uint8_t)(data >> 8));
	I2C_Write((uint8_t)(data & 0xFF));
	I2C_Stop();
	
	return 0;
}

/**************************************************************************/
/*!
    @brief  read register from ADC
*/
/**************************************************************************/
uint16_t ads1115_read_register(uint8_t addr, uint8_t reg)
{
	I2C_Start();
	I2C_Write((addr << 1) + I2C_WRITE_BIT);
	I2C_Write(reg);
	//I2C_Stop();
	
	I2C_Start();
	I2C_Write((addr << 1) + I2C_READ_BIT);
	uint8_t msb = I2C_Read(1);
	uint8_t lsb = I2C_Read(0);
	I2C_Stop();
	
	uint16_t data = (msb << 8 | lsb);
	return data;
}

/**************************************************************************/
/*!
    @brief  read raw data from one channel of ADC
*/
/**************************************************************************/
uint16_t ads1115_readADC_SingleEnded(uint8_t addr, uint8_t channel, ads1115_datarate dr, ads1115_fsr_gain gain)
{
	// Check channel number
	if(channel > 3)
	{
		return 0;
	}
	
	uint16_t adc_config = ADS1115_COMP_QUE_CON1	|
				 ADS1115_COMP_LAT_NonLatching |
				 ADS1115_COMP_POL_3_ACTIVELOW |	
				 ADS1115_COMP_MODE_TRADITIONAL | 
				 dr |
				 //DR_128SPS |
				 ADS1115_MODE_CONTINUOUS | 
				 gain;
				 //FSR_6_144;
	
	if(channel == 0)
	{
		adc_config |= ADS1115_MUX_AIN0_GND;
	} 
	else if(channel == 1)
	{
		adc_config |= ADS1115_MUX_AIN1_GND;
	} 
	else if(channel == 2)
	{
		adc_config |= ADS1115_MUX_AIN2_GND;
	} 
	else if(channel == 3)
	{
		adc_config |= ADS1115_MUX_AIN3_GND;
	}
	
	adc_config |= ADS1115_OS_SINGLE;	
	
	ads1115_write_register(addr, ADS1115_REG_CONFIG, adc_config);
	//_delay_ms(8);
	
	return ads1115_read_register(addr, ADS1115_REG_CONVERSION) >> 0;
}

/**************************************************************************/
/*!
    @brief  read raw data / conversion result of ADC:
			differential between P: AIN0 and N: AIN1
*/
/**************************************************************************/
int16_t ads1115_readADC_Diff_A0_1(uint8_t addr, ads1115_datarate dr, ads1115_fsr_gain gain)
{
	uint16_t adc_config = ADS1115_COMP_QUE_DIS	|
						  ADS1115_COMP_LAT_NonLatching |
						  ADS1115_COMP_POL_3_ACTIVELOW |	
						  ADS1115_COMP_MODE_TRADITIONAL | 
						  dr |
						  //ADS1115_DR_128SPS |
						  ADS1115_MODE_SINGLE | 
						  gain;
						  //FSR_6_144;
	
	adc_config |= ADS1115_MUX_AIN0_AIN1;
	
	adc_config |= ADS1115_OS_SINGLE;	
	
	ads1115_write_register(addr, ADS1115_REG_CONFIG, adc_config);
	//_delay_ms(8);
	
	return (int16_t)ads1115_read_register(addr, ADS1115_REG_CONVERSION);
}

/**************************************************************************/
/*!
    @brief  read raw data / conversion result of ADC:
			differential between P: AIN0 and N: AIN3
*/
/**************************************************************************/
int16_t ads1115_readADC_Diff_A0_3(uint8_t addr, ads1115_datarate dr, ads1115_fsr_gain gain)
{
	uint16_t adc_config = ADS1115_COMP_QUE_DIS	|
						  ADS1115_COMP_LAT_NonLatching |
						  ADS1115_COMP_POL_3_ACTIVELOW |	
						  ADS1115_COMP_MODE_TRADITIONAL | 
						  dr |
						  //ADS1115_DR_128SPS |
						  ADS1115_MODE_SINGLE | 
						  gain;
						  //FSR_6_144;
	
	adc_config |= ADS1115_MUX_AIN0_AIN3;
	
	adc_config |= ADS1115_OS_SINGLE;	
	
	ads1115_write_register(addr, ADS1115_REG_CONFIG, adc_config);
	//_delay_ms(8);
	
	return (int16_t)ads1115_read_register(addr, ADS1115_REG_CONVERSION);
}

/**************************************************************************/
/*!
    @brief  read raw data / conversion result of ADC:
			differential between P: AIN1 and N: AIN3
*/
/**************************************************************************/
int16_t ads1115_readADC_Diff_A1_3(uint8_t addr, ads1115_datarate dr, ads1115_fsr_gain gain)
{
	uint16_t adc_config = ADS1115_COMP_QUE_DIS	|
						  ADS1115_COMP_LAT_NonLatching |
						  ADS1115_COMP_POL_3_ACTIVELOW |	
						  ADS1115_COMP_MODE_TRADITIONAL | 
						  dr |
						  //ADS1115_DR_128SPS |
						  ADS1115_MODE_SINGLE | 
						  gain;
						  //FSR_6_144;
	
	adc_config |= ADS1115_MUX_AIN1_AIN3;
	
	adc_config |= ADS1115_OS_SINGLE;	
	
	ads1115_write_register(addr, ADS1115_REG_CONFIG, adc_config);
	//_delay_ms(8);
	
	return (int16_t)ads1115_read_register(addr, ADS1115_REG_CONVERSION);
}

/**************************************************************************/
/*!
    @brief  read raw data / conversion result of ADC:
			differential between P: AIN2 and N: AIN3
*/
/**************************************************************************/
int16_t ads1115_readADC_Diff_A2_3(uint8_t addr, ads1115_datarate dr, ads1115_fsr_gain gain)
{
	uint16_t adc_config = ADS1115_COMP_QUE_DIS	|
						  ADS1115_COMP_LAT_NonLatching |
						  ADS1115_COMP_POL_3_ACTIVELOW |	
						  ADS1115_COMP_MODE_TRADITIONAL | 
						  dr |
						  //ADS1115_DR_128SPS |
						  ADS1115_MODE_SINGLE | 
						  gain;
						  //FSR_6_144;
	
	adc_config |= ADS1115_MUX_AIN2_AIN3;
	
	adc_config |= ADS1115_OS_SINGLE;	
	
	ads1115_write_register(addr, ADS1115_REG_CONFIG, adc_config);
	//_delay_ms(8);
	
	return (int16_t)ads1115_read_register(addr, ADS1115_REG_CONVERSION);
}

#endif /* ADS1115_H_DEVCUBE */