/*
 * LedControll.h
 *
 *  Created on: 2014.03.29.
 *      Author: zoon
 */
//						This is a special file for DIY AVR Development board
#ifndef DEVBOARDLIBS_H_
#define DEVBOARDLIBS_H_
#define USART_BAUDRATE 19200
#define UBRR_ERTEK ((F_CPU / (USART_BAUDRATE * 16UL)) - 1) // UBRR
#ifndef __HAS_DELAY_CYCLES
#include <util/delay.h>
#endif

#ifndef _AVR_IO_H_
#include <avr/io.h>
#endif
#include <avr/interrupt.h>
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
#endif /* LEDCONTROLL_H_ */
