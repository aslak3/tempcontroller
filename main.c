/* Temperature controller
 *
 * For the ATMEGA8 and perhaps others.
 *
 * (c) 2014 Lawrence Manning. */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/eeprom.h>

#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

#define BUFFER_SIZE 256

#define HISTORY_SIZE 10

#define MIN_TARGET_TEMP 10
#define MAX_TARGET_TEMP 180

/* Serial related */
void writechar(char c, unsigned char debug);
void writestring(char *string, unsigned char debug);

/* SPI */
unsigned char spitxrxbyte(unsigned char data);
void spitxrxbuffer(unsigned char *tx, int txcount,
	unsigned char *rx, int rxcount);

unsigned char encodesevensegs(unsigned char digit);

unsigned char debugmode = 0;
unsigned char echo = 1;

/* The message to show, shared with interrupt handler. */
char message[4 + 1] = "    ";
int keypresstimer = 0;

int main(void)
{
	/* Configure the serial port UART */
	UBRRL = BAUD_PRESCALE;
	UBRRH = (BAUD_PRESCALE >> 8);
	UCSRC = (1 << URSEL) | (3 << UCSZ0);
	UCSRB = (1 << TXEN);   /* Turn on the transmission circuitry ONLY. */

	TCCR1B |= (1 << WGM12); // CTC
	TCCR1B |= ((1 << CS10) | (1 << CS11)); // Set up timer at Fcpu/64
	OCR1A   = 625; // 200Hz
	TIMSK |= (1 << OCIE1A); // Enable CTC interrupt
	
	DDRB = (1 << 5) | (1 << 3) | (1 << 2) | (1 << 0);
	/* Setup SPI. */
	SPCR = (1 << SPE)|(1 << MSTR)|(1 << CPHA)|(1 << SPR0);

	PORTB = 0x04; /* Disable ss, turn off heater at startup! */

	DDRC = 0x0f; /* 4 digits are outputs. */
	PORTC = 0x30; /* Pullup the up/down inputs. */
	DDRD = 0xfd; /* 7 segments are outputs, leaving TxD for serial on PD1. */
	
	_delay_us(10);		/* Lets the UART fully wake up */

	sei(); /* Turn on interrupts. */
	
	int heateron = 0; /* Important: wether heater is on or not. */
	/* History of readings, current one always in element 0. */
	int temps[HISTORY_SIZE];

	memset(temps, 0, sizeof(int) * HISTORY_SIZE);

	int targettemp = 175; /* Ideal toner transfer temp. */
	int eepromtargettemp = (int) eeprom_read_word((uint16_t *) 0);

	/* Load the target temp from EEPROM, if the value looks ok. */	
	if (eepromtargettemp > MIN_TARGET_TEMP &&
		eepromtargettemp < MAX_TARGET_TEMP)
	{
		targettemp = eepromtargettemp;
	}
	
	/* Count of reads done, upto HISTORY_SIZE. */
	int readcount = 0;
	/* Message to scroll why the history buffer is fulled, must end in
	 * four spaces. */
	char startingmessage[HISTORY_SIZE + 1];

	/* Fill the message with hyphensm then add the greeting to the
	 * start. */
	memset(startingmessage, 0, HISTORY_SIZE + 1);
	memset(startingmessage, '-', HISTORY_SIZE);
	strncpy(startingmessage, "HELLO ", HISTORY_SIZE);

	for (;;)
	{
		int temp = 0; /* Current temp. */

		char serialoutput[BUFFER_SIZE];
		unsigned char tempbuffer[2]; /* SPI goes here. */

		memset(tempbuffer, 0, 2);
		
		int origtemp = 0; /* Temp read from SPI. */

		/* Chip select the MAX6675 and give it a while. */				
		PORTB &= 0xfb;
		_delay_us(10);
		
		/* Read the temperature and convert it per the datasheet. */	
		spitxrxbuffer(NULL, 0, tempbuffer, 2);
		/* We disgard the quater degree C increments for now. */
		origtemp = ((tempbuffer[0] * 256) + tempbuffer[1]) / 32;

		/* Deselect the MAX6675. */
		_delay_us(10);
		PORTB |= 0x04;

		int c;
		/* Rotate the history so the oldest eleemnt is lost. */
		for (c = HISTORY_SIZE - 2; c >= 0; c--)
			temps[c + 1] = temps[c];
		/* Save the current temp in the history. */
		temps[0] = origtemp;
		
		/* Calculate the average temp over the interval. */
		for (c = 0; c < HISTORY_SIZE; c++)
			temp += temps[c];
		temp /= HISTORY_SIZE;

		/* Disable interrupts because we need to update the shared
		 * message string. */
		cli();
		if (keypresstimer == 0)
		{
			/* If we are not manipulating the target temp.... */
			if (origtemp <= 0)
				/* If we have an error show E */
				snprintf(message, 4 + 1, "E---");
			else if (readcount >= HISTORY_SIZE)
				/* If we have enough data then show the temp. */
				snprintf(message, 4 + 1, "%03dC", temp);
			else
				/* Otherise scroll the starting mesage across. */
				strncpy(message, startingmessage + readcount, 4);
		}
		/* Enable interrupts once more. */
		sei();
		
		/* Send the raw temp, calcualated current temp, target temp,
		 * and wether the heater is on to the serial port. */
		snprintf(serialoutput, BUFFER_SIZE - 1, "%d,%d,%d,%d\r\n",
			origtemp, temp, targettemp, heateron);
		writestring(serialoutput, 0);
		
		/* Now we actually figure out if we should turn the heater on
		 * or off. If the heater is at the target temp we will do
		 * nothing and leave the heater at its current state. The
		 * idea here is that we want to avoid flicking the heater
		 * on and off too quickly. We only do this once we have
		 * a full history of previous temps. */
		if (readcount >= HISTORY_SIZE)
		{
			if (temp > targettemp + 1 && heateron == 1)
			{
				PORTB &= 0xfe;
				heateron = 0;
			}
			
			if (temp < targettemp - 1 && heateron == 0)
			{
				PORTB |= 0x01;
				heateron = 1;
			}
		}
		
		/* If we got a dodgy value, force the heater off. */
		if (origtemp <= 0)
		{
			PORTB &= 0xfe;
			heateron = 0;
		}
		
		/* This is the button press sensing code. We poll the buttons
		 * every 10ms, and detect buttons going up and down so the
		 * user can rappidly adjust the target temp. But also if
		 * they leave the button pressed it will change slowly. The
		 * buttons are active low on port C. */
		unsigned char lastpinc = 0x30;
		int delaycount;
		for (delaycount = 0; delaycount < 50; delaycount++)
		{
			unsigned char pinc = PINC & 0x30;
			if (((pinc & 0x30) != 0x30) && (keypresstimer < 350 || pinc != lastpinc))
			{
				/* If a button was pushed (changed state) or the button
				 * is pushed and its been 50 ticks since the last press
				 * then we will change the target temp. The keypresstimer
				 * value is updated in the interrupt handler. */
				/* Disable interrupts because of message. */
				cli();
				/* When this becomes 0 we revert to showing the current temp. */
				keypresstimer = 400;
				snprintf(message, 4 + 1, "%03dC", targettemp);
				sei();
				/* Adjust the target temp. */
				if (!(pinc & 0x10) && targettemp > MIN_TARGET_TEMP)
					targettemp--;
				if (!(pinc & 0x20) && targettemp < MAX_TARGET_TEMP)
					targettemp++;
					
				eeprom_write_word((uint16_t *) 0, targettemp);
			}
			
			/* Store the last state of the buttons so we can know
			 * if a button has been pushed. */
			lastpinc = pinc;
			_delay_ms(10);
		}

		/* Adjust the number of samples taken, upto size of history. */
		if (readcount < HISTORY_SIZE)
			readcount++;
	}
	
	return 0; /* Not reached. */
}

void writechar(char c, unsigned char debug)
{
	if (debug && !debugmode) return;
	
	while(!(UCSRA & (1<<UDRE)));

	UDR = c;
}

void writestring(char *string, unsigned char debug)
{
	char *p = string;
	
	while (*p)
	{
		writechar(*p, debug);
		p++;
	}
}

unsigned char spitxrxbyte(unsigned char data)
{
	SPDR = data;
	while(!(SPSR & (1 << SPIF)))
		;
	return SPDR;
}

void spitxrxbuffer(unsigned char *tx, int txcount,
	unsigned char *rx, int rxcount)
{
	int c;
	
	for (c = 0; c < txcount; c++)
		spitxrxbyte(tx[c]);
	for (c = 0; c < rxcount; c++)
		rx[c] = spitxrxbyte(0);
}

unsigned char encodesevensegs(unsigned char digit)
{
	unsigned char output;
	
	/* Order of segments is: gfedcbXa, where X is the serial transmit
	 * line, so should be always 0. */
	
	switch (digit)
	{
		case '0': output = 0b01111101; break;
		case '1': output = 0b00001100; break;
		case '2': output = 0b10110101; break;
		case '3': output = 0b10011101; break;
		case '4': output = 0b11001100; break;
		case '5': output = 0b11011001; break;
		case '6': output = 0b11111001; break;
		case '7': output = 0b00001101; break;
		case '8': output = 0b11111101; break;
		case '9': output = 0b11011101; break;
		case 'A': output = 0b11101101; break;
		case 'B': output = 0b11111000; break;
		case 'C': output = 0b01110001; break;
		case 'D': output = 0b10111100; break;
		case 'E': output = 0b11110001; break;
		case 'F': output = 0b11100001; break;
		case 'H': output = 0b11101100; break;
		case 'I': output = 0b01100000; break;
		case 'Y': output = 0b11001100; break;
		case 'L': output = 0b01110000; break;
		case 'O': output = 0b01111101; break;
		case ' ': output = 0b00000000; break;
		default:  output = 0b10000000; break;
	}
	
	return output;
}

/* The thing that makes it all work: timer interrupt. */
ISR(TIMER1_COMPA_vect)
{
	static unsigned char digit = 0;
	static unsigned char flashcounter = 0;

	/* Invert because the segment port is active low, ignoring the
	 * serial transmit line at bit 1. */
	unsigned char sevensegs = encodesevensegs(message[digit]) ^ 0xfd;
	/* Digits are ordered 3210. */
	unsigned char digitenable = (1 << (3 - digit));
	
	PORTC = 0x30; /* All off. */
	/* See if we should "modulate" the display by flashing the
	 * digits. This happens if a key has been pressed recently.
	 * Since flashcounter wraps at 255, the flash will be for
	 * 1/4 of the cycle. */
	if (keypresstimer > 0 && flashcounter > 128 + 64)
		PORTD = 0xfd;/* Off. */
	else
		PORTD = sevensegs;
	/* Or in the enabled digit, leaving the other values for pullups. */
	PORTC = digitenable | 0x30;

	digit++;
	if (digit > 3) digit = 0;
	
	flashcounter++;
	if (keypresstimer > 0) keypresstimer--;
}

