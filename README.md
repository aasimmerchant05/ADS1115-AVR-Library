# ADS1115-AVR-Library
ADS1115 interfacing with AVR MCU<br>

based on ADS1115 Analog Digital Converter AVR Library - https://github.com/tonysteinbock/avr-ads1115 <br>

Uses my custom I2C library

<h3>PIN CONFIGURATION</h3>
 *		ADS1115 SCL				---		I2C_SCL = PINB2<br>
 *		ADS1115 SDA				---		I2C_SDA = PINB3<br>
 *		ADS1115 ADDR			---		GND<br>
 *		ADS1115 ALERT/DDRY		---		any I/O = PIND3<br>
