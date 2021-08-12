// AVR Shopvac Controller
//
// Written for ATTiny85
// Wiring
// PB4/ADC2 - Current sensor (ACS712)
// PB3/ADC3 - Threshold selector potentiometer (10k)
// PB5 - Relay for shopvac

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define STATE_RUNNING   1
#define STATE_STOPPED   2
#define STATE_STARTING  3
#define STATE_STOPPING  4

#define TICKS_PER_SECOND   ((F_CPU / 1024) / 256)

#define DELAY_START     (TICKS_PER_SECOND * 0)
#define DELAY_STOP      (TICKS_PER_SECOND * 0)
#define SAMPLING_TICKS  (TICKS_PER_SECOND / 2)

#define MAX(a,b) ((a) > (b) ? (a) : (b))

uint8_t state = STATE_STOPPED;
uint16_t threshold;
uint16_t currentValue;
uint32_t ticksInState;
uint32_t samplingTicks = 0;

ISR (TIMER0_OVF_vect)
{
    ticksInState++;

    if(samplingTicks++ < SAMPLING_TICKS)
    {
        return;
    }

    samplingTicks = 0;

    //debug
    PORTB ^= (1 << PB0);
    if(currentValue >= 512)
    {
        PORTB |= (1 << PB1);
    }
    else
    {
        PORTB &= ~(1 << PB1);
    }

    switch(state) {
        case STATE_STOPPED:
            // Enter starting state when current is above threshold
            if(currentValue >= threshold)
            {
                state = STATE_STARTING;
                ticksInState = 0;
            }
            break;
        case STATE_RUNNING:
            // Enter stopping state when current is below threshold
            if(currentValue < threshold)
            {
                state = STATE_STOPPING;
                ticksInState = 0;
            }
            break;
        case STATE_STARTING:
            // Cancel starting when current drops below threshold
            if(currentValue < threshold)
            {
                state = STATE_STOPPED;
            }
            // Close relay once starting delay has elapsed
            else if(ticksInState >= DELAY_START)
            {
                PORTB |= (1 << PB5);
            }
            break;
        case STATE_STOPPING:
            // Cancel stopping when current raises above threshold
            if(currentValue >= threshold)
            {
                state = STATE_RUNNING;
            }
            // Open relay once stopping delay has elapsed
            else if(ticksInState >= DELAY_STOP)
            {
                PORTB &= ~(1 << PB5);
            }
            break;
    }

    // Reset measured current so next sample set can be collected
    currentValue = 0;
}

uint16_t read_adc(uint8_t mux)
{
    ADMUX = mux;

    ADCSRA |= (1 << ADSC);              // Start conversion
    while(ADCSRA & (1 << ADSC));        // Wait for conversion to complete

    return ADC;
}

int main()
{
    // Configure timer0
    TCCR0A = 0x00;                      // Normal operation mode
    TCCR0B = (1 << CS02) | (1 << CS00); // 1024 prescalar
    sei();
    TCNT0 = 0x00;                       // Clear timer counter
    TIMSK |= (1 << TOIE0);              // Enable timer0 interrupt

    // Configure pins
    DDRB =
        (1 << PB5) |                    // Set PB5 as output (relay pin), other pins are input
        (1 << PB0) |                    // Set PB0/1 as output for debugging purposes
        (1 << PB1);

    PORTB = 0x00;

    // Configure ADC
    ADCSRA =
        (1 << ADEN) |                   // Enable ADC
        (1 << ADPS2) |                  // 64 prescaler
        (1 << ADPS1);

    // Enter a readloop where threshold and the current sensor are polled continously;
    uint8_t iter = 0;
    for(;;)
    {
        // Read the threshold value every 256 iterations
        if(iter-- == 0)
        {
            threshold = read_adc((1 << MUX1) | (1 << MUX0));    // ADC3/PB3
            _delay_ms(1);
        }

        // Store max value of ADC
        uint16_t newCurrent = read_adc(1 << MUX1);  // ADC2/PB4
        currentValue = MAX(currentValue, newCurrent);
    }
}
