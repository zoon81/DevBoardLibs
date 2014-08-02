/*
 * LedControll.h
 *
 *  Created on: 2014.03.29.
 *      Author: zoon
 */
//						This is a special file for DIY AVR Development board

#define DEVBOARDLIBS_H_
#define USART_BAUDRATE 19200
#define UBRR_ERTEK ((F_CPU / (USART_BAUDRATE * 16UL)) - 1) // UBRR
//I2C Config . This library was written by Peter Fleury. download from http://homepage.hispeed.ch/peterfleury/i2cmaster.zip
#define I2C_READ    1
#define I2C_WRITE   0
#ifndef _I2CMASTER_H
#define _I2CMASTER_H   1
#define SCL_CLOCK  50000L
#define i2c_read(ack)  (ack) ? i2c_readAck() : i2c_readNak();


#ifndef __HAS_DELAY_CYCLES
#include <util/delay.h>
#endif

#ifndef _AVR_IO_H_
#include <avr/io.h>
#endif
#include <avr/interrupt.h>
#include <inttypes.h>
#include <compat/twi.h>
#define BIT(x) (1 << (x))
#define SETBITS(x,y) ((x) |= (y))
#define CLEARBITS(x,y) ((x) &= (~(y)))
#define SETBIT(x,y) SETBITS((x), (BIT((y))))
#define CLEARBIT(x,y) CLEARBITS((x), (BIT((y))))

void delay_ms(uint16_t delay)
{
	while (delay)
	{
		_delay_ms(1);
		delay--;

	}
}
void LedBlink(uint8_t NumberOfBlinks, uint16_t delay)
{
	DDRB |= 1;
	while (NumberOfBlinks)
	{
		PORTB |= 1;
		delay_ms(delay);
		PORTB &= ~1;
		delay_ms(delay);
		NumberOfBlinks--;
	}

}
void endloop(uint16_t blinkdelay)
{
	DDRB |= 1;
		while (1)
		{
			PORTB |= 1;
			delay_ms(blinkdelay);
			PORTB &= ~1;
			delay_ms(blinkdelay);

		}
}
void UARTInit()  // UART beallitasa
{

	UBRR0L = UBRR_ERTEK; // UBRR_ERTEK also 8 bitjenek betoltese az UBRRL regiszterbe
	UBRR0H = (UBRR_ERTEK >> 8); // UBRR_ERTEK felso 8 bitjenek betoltese az UBRRH regiszterbe
	// UCSR0A |= (1<<U2X0);

	//Ado es Vevo aramkorok bekapcsolasa

	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	UCSR0C = (3 << UCSZ00);
}
char UARTReceive()
{
	while (!(UCSR0A & (1 << RXC0))){} // Varakozas amig nincs uj bejovo adat
	return UDR0;
}

void UARTSendByte(char data) // Ez a fuggveny a kuldendo adatot beirja az UDR regiszter kimeno pufferjebe
{
	while (!(UCSR0A & (1 << UDRE0))){} // Varakozas amig az Ado kesz nem lesz az adatkuldesre
	UDR0 = data;
}

void UARTSendString(char * str)
{
	while (*str)
	{
		UARTSendByte(*str);
		str++;
	}
}
char UARTSendInt(int number)
{
		//hány jegyű a szám
	int oszto=1;
	char jegy_hossz=1,i;
	while((number+1) / oszto > 9 )

	    {
	    oszto=oszto*10;
	    jegy_hossz++;
	    }

 for(i=0;i<jegy_hossz;i++)
 {
	 UARTSendByte( ((number/ oszto)+48) );
	 number-= number / oszto * oszto;
	 oszto=oszto/10;

 }
return 0;
}
void SPIInit(void)
{
	//Set SCK (PB5), MOSI (PB3) , CSN (SS & PB2)

	DDRB |= (1<<DDB5) | (1<<DDB3) | (1<<DDB2);

	// Enable SPI, Master, set clock rate fck/16
	SPCR |= (1<<SPE)|(1<<MSTR);// |(1<<SPR0) |(1<<SPR1);


}

char SPISendByte(unsigned char cData)
{
	//Load byte to Data register
	SPDR = cData;

	/* Wait for transmission complete */
	while(!(SPSR & (1<<SPIF)));

	return SPDR;
}




/*************************************************************************
 Initialization of the I2C bus interface. Need to be called only once
*************************************************************************/
void i2c_init(void)
{
  /* initialize TWI clock: 100 kHz clock, TWPS = 0 => prescaler = 1 */

  TWSR = 0;                         /* no prescaler */
  TWBR = ((F_CPU/SCL_CLOCK)-16)/2;  /* must be > 10 for stable operation */

}/* i2c_init */


/*************************************************************************
  Issues a start condition and sends address and transfer direction.
  return 0 = device accessible, 1= failed to access device
*************************************************************************/
unsigned char i2c_start(unsigned char address)
{
    uint8_t   twst;

	// send START condition
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

	// wait until transmission completed
	while(!(TWCR & (1<<TWINT)));

	// check value of TWI Status Register. Mask prescaler bits.
	twst = TW_STATUS & 0xF8;
	if ( (twst != TW_START) && (twst != TW_REP_START)) return 1;

	// send device address
	TWDR = address;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// wail until transmission completed and ACK/NACK has been received
	while(!(TWCR & (1<<TWINT)));

	// check value of TWI Status Register. Mask prescaler bits.
	twst = TW_STATUS & 0xF8;
	if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return 1;

	return 0;

}/* i2c_start */


/*************************************************************************
 Issues a start condition and sends address and transfer direction.
 If device is busy, use ack polling to wait until device is ready

 Input:   address and transfer direction of I2C device
*************************************************************************/
void i2c_start_wait(unsigned char address)
{
    uint8_t   twst;


    while ( 1 )
    {
	    // send START condition
	    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

    	// wait until transmission completed
    	while(!(TWCR & (1<<TWINT)));

    	// check value of TWI Status Register. Mask prescaler bits.
    	twst = TW_STATUS & 0xF8;
    	if ( (twst != TW_START) && (twst != TW_REP_START)) continue;

    	// send device address
    	TWDR = address;
    	TWCR = (1<<TWINT) | (1<<TWEN);

    	// wail until transmission completed
    	while(!(TWCR & (1<<TWINT)));

    	// check value of TWI Status Register. Mask prescaler bits.
    	twst = TW_STATUS & 0xF8;
    	if ( (twst == TW_MT_SLA_NACK )||(twst ==TW_MR_DATA_NACK) )
    	{
    	    /* device busy, send stop condition to terminate write operation */
	        TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);

	        // wait until stop condition is executed and bus released
	        while(TWCR & (1<<TWSTO));

    	    continue;
    	}
    	//if( twst != TW_MT_SLA_ACK) return 1;
    	break;
     }

}/* i2c_start_wait */


/*************************************************************************
 Issues a repeated start condition and sends address and transfer direction

 Input:   address and transfer direction of I2C device

 Return:  0 device accessible
          1 failed to access device
*************************************************************************/
unsigned char i2c_rep_start(unsigned char address)
{
    return i2c_start( address );

}/* i2c_rep_start */


/*************************************************************************
 Terminates the data transfer and releases the I2C bus
*************************************************************************/
void i2c_stop(void)
{
    /* send stop condition */
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);

	// wait until stop condition is executed and bus released
	while(TWCR & (1<<TWSTO));

}/* i2c_stop */


/*************************************************************************
  Send one byte to I2C device

  Input:    byte to be transfered
  Return:   0 write successful
            1 write failed
*************************************************************************/
unsigned char i2c_write( unsigned char data )
{
    uint8_t   twst;

	// send data to the previously addressed device
	TWDR = data;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// wait until transmission completed
	while(!(TWCR & (1<<TWINT)));

	// check value of TWI Status Register. Mask prescaler bits
	twst = TW_STATUS & 0xF8;
	if( twst != TW_MT_DATA_ACK) return 1;
	return 0;

}/* i2c_write */


/*************************************************************************
 Read one byte from the I2C device, request more data from device

 Return:  byte read from I2C device
*************************************************************************/
unsigned char i2c_readAck(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	while(!(TWCR & (1<<TWINT)));

    return TWDR;

}/* i2c_readAck */


/*************************************************************************
 Read one byte from the I2C device, read is followed by a stop condition

 Return:  byte read from I2C device
*************************************************************************/
unsigned char i2c_readNak(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT)));

    return TWDR;

}
#endif /* LEDCONTROLL_H_ */
