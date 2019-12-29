#define __AVR_ATmega328P__

#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/setbaud.h>

// PWM_PIN is also the OC1A which will be configured in initPWM() function.
#define PWM_PIN PORTB1
#define DIR_PIN PORTB2
#define BRK_PIN PORTD7
#define ENCODER_A PORTD2
#define ENCODER_B PORTD3

volatile int8_t state_a = 0, state_b = 0;
volatile int8_t state = 0, state_old = 0;
int enc_count = 0, enc_count_old = 0;

// Init USART Function
void Config_USART0()
{
    /*Set baud rate */
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;

    /*Set baud rate speed is 1x*/
    UCSR0A &= ~_BV(U2X0);

    // Enable receiver and transmitter, no interrupt
    UCSR0B |= (_BV(RXEN0) | _BV(TXEN0));
    UCSR0B &= ~_BV(RXCIE0);
    UCSR0B &= ~_BV(TXCIE0);

    // Set frame format: No parity check, 8 Data bits, 1 stop bit
    UCSR0C |= (_BV(UCSZ01) | _BV(UCSZ00));
}

// USART Transmitting Function
int USART0SendByte(char u8Data, FILE *stream)
{
    if (u8Data == '\n')
		USART0SendByte('\r', stream);
		
	/* Wait for empty transmit buffer */
	loop_until_bit_is_set(UCSR0A, UDRE0);
	UDR0 = u8Data;
	
	return 0;
}

// USART Receiving Function
int USART0ReceiveByte(FILE *stream)
{
    unsigned char u8Data;

    // Wait for byte to be received
    // When RXC0 is set, there is unread data in the buffer
    loop_until_bit_is_set(UCSR0A, RXC0);

    u8Data = UDR0;

    //echo input data
    // USART0SendByte(u8Data,stream);

    // Return received data
    return u8Data;
}

void initSignalPin(void)
{
    // Set the pin for output
    DDRB |= (1 << PWM_PIN | 1 << DIR_PIN);
    DDRB |= (1 << BRK_PIN);

    // Set the pin for input
    DDRD &= (~(1 << ENCODER_A) | ~(1 << ENCODER_B));
}

void initEncoderInterrupt(void)
{
    cli();
    // External Interrupt Mask Register, enable INT0 and INT1
    EIMSK |= (1 << INT0 | 1 << INT1);
    // Trigger when encoder logic change
    EICRA |= (1 << ISC00 | 1 << ISC10);

    // Init the old state of encoder
    state_a = bit_is_set(PIND, ENCODER_A) >> (ENCODER_A);
    state_b = bit_is_set(PIND, ENCODER_B) >> (ENCODER_B);
    if (state_a == 0 && state_b == 0)
        state_old = 1;
    else if (state_a == 0 && state_b == 1)
        state_old = 2;
    else if (state_a == 1 && state_b == 1)
        state_old = 3;
    else if (state_a == 1 && state_b == 0)
        state_old = 4;

    // set (global) interrupt enable bit
    sei();
}

void initPWM(void)
{
    // Timer 1 A,B, 16 bit timer
    // Fast PWM mode, counter max in ICR1, this can control the frequency of pwm
    TCCR1A |= (1 << WGM11);
    TCCR1B |= (1 << WGM12) | (1 << WGM13);

    // Configure the frequency of timer1, prescale it by F_CPU / 64, since our CPU
    // run at 16MHZ, the frequency of timer is 1/4 MHZ, a count in timer will spend
    // 4 microsecond = 0.004 ms
    TCCR1B |= (1 << CS11 | 1 << CS10);

    // Set the counter max in ICR1, which is the TOP value for timer1, about 0.048ms
    //. The frequency is about 20KHz
    ICR1 = 12;

    // Create output on OC1A which is PB1 pin
    TCCR1A |= (1 << COM1A1);

}

void encoderInterrupt(void)
{
    state_a = bit_is_set(PIND, ENCODER_A) >> (ENCODER_A);
    state_b = bit_is_set(PIND, ENCODER_B) >> (ENCODER_B);

    if (state_a == 0 && state_b == 0)
        state = 1;
    else if (state_a == 0 && state_b == 1)
        state = 2;
    else if (state_a == 1 && state_b == 1)
        state = 3;
    else if (state_a == 1 && state_b == 0)
        state = 4;

    if ((state - state_old == 1) || (state - state_old == -3))
    {
        //Backward
        enc_count--;
        state_old = state;
    }
    else if ((state - state_old == -1) || (state - state_old == 3))
    {
        //Forward
        enc_count++;
        state_old = state;
    }
}

void pidVelocityControl(void)
{}

ISR(INT0_vect)
{
    encoderInterrupt();
}

ISR(INT1_vect)
{
    encoderInterrupt();
}

ISR(TIMER0_COMPA_vect)
{
    // pidVelocityControl();
}

// Set stream pointer
FILE usart0_str = FDEV_SETUP_STREAM(USART0SendByte, USART0ReceiveByte, _FDEV_SETUP_RW);

int main(void)
{
    initSignalPin();
    initEncoderInterrupt();
    initPWM();
    Config_USART0();

    // Assign our stream to standart I/O streams
	stdin = stdout = &usart0_str;

    while (1)
    {
        printf("%d\n", enc_count);
        _delay_ms(50);
    }

    return 0;
}