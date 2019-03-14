#ifndef I2CLib
#define I2CLib

#include <avr/io.h>

void I2C_Init(uint8_t bit_PS1, uint8_t bit_PS0, uint8_t bitRate);
void I2C_Start();
void I2C_Stop(void);
void I2C_Write(uint8_t v_i2cData_u8);
uint8_t I2C_Read(uint8_t v_ackOption_u8);

void I2C_Init()
{
	TWSR=0x00; //set presca1er bits to zero
	TWBR=bitRate; //SCL frequency is 50K for 16Mhz
	TWCR=0x04; //enab1e TWI module	
}

void I2C_Start()
{
	TWCR = ((1<<TWINT) | (1<<TWSTA) | (1<<TWEN));
	while (!(TWCR & (1<<TWINT)));
}

void I2C_Stop(void)
{
	TWCR = ((1<< TWINT) | (1<<TWEN) | (1<<TWSTO));
	_delay_us(100) ; //wait for a short time
}

void I2C_Write(uint8_t v_i2cData_u8)
{
	TWDR = v_i2cData_u8 ;
	TWCR = ((1<< TWINT) | (1<<TWEN));
	while (!(TWCR & (1 <<TWINT)));
}

uint8_t I2C_Read(uint8_t v_ackOption_u8)
{
	TWCR = ((1<< TWINT) | (1<<TWEN) | (v_ackOption_u8<<TWEA));
	while ( !(TWCR & (1 <<TWINT)));
	return TWDR;
}

#endif