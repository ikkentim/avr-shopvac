// AVR Shopvac Controller
//
// Written for ATTiny85
// Wiring
// PB4/ADC2 - Current sensor (ACS712)
// PB3/ADC3 - Threshold selector potentiometer (10k)
// PB0 - Relay for shopvac

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define STATE_RUNNING       1
#define STATE_STOPPED       2
#define STATE_STARTING      3
#define STATE_STOPPING      4

#define TICKS_PER_SECOND   ((F_CPU / 1024) / 256)

#define DELAY_START         (TICKS_PER_SECOND * 1)
#define DELAY_STOP          (TICKS_PER_SECOND * 5)
#define SAMPLING_TICKS      (TICKS_PER_SECOND / 2)

#define PIN_RELAY           PB0
#define PIN_DEBUG1          PB1
#define PIN_DEBUG2          PB2
#define MUX_THRESHOLD       ((1 << MUX1) | (1 << MUX0))
#define MUX_CURRENT         ((1 << MUX1))

#define MAX(a,b)            ((a) > (b) ? (a) : (b))

#define PIN_ENABLE(pb)     do{ PORTB |= (1 << (pb)); }while(0)
#define PIN_DISABLE(pb)    do{ PORTB &= ~(1 << (pb)); }while(0)

#if defined DEBUG
#  define DEBUG_PIN_ENABLE(pb) PIN_ENABLE(pb)
#  define DEBUG_PIN_DISABLE(pb) PIN_DISABLE(pb)
#else
#  define DEBUG_PIN_ENABLE(pb)  while(0)
#  define DEBUG_PIN_DISABLE(pb)  while(0)
#endif

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

    if(currentValue >= threshold)
    {
        DEBUG_PIN_ENABLE(PIN_DEBUG2);
    }
    else
    {
        DEBUG_PIN_DISABLE(PIN_DEBUG2);
    }

    switch(state) {
        case STATE_STOPPED:
            // Enter starting state when current is above threshold
            if(currentValue >= threshold)
            {
                state = STATE_STARTING;
                DEBUG_PIN_ENABLE(PIN_DEBUG1);
                ticksInState = 0;
            }
            break;
        case STATE_RUNNING:
            // Enter stopping state when current is below threshold
            if(currentValue < threshold)
            {
                state = STATE_STOPPING;
                DEBUG_PIN_ENABLE(PIN_DEBUG1);
                ticksInState = 0;
            }
            break;
        case STATE_STARTING:
            // Cancel starting when current drops below threshold
            if(currentValue < threshold)
            {
                state = STATE_STOPPED;
                DEBUG_PIN_DISABLE(PIN_DEBUG1);
            }
            // Close relay once starting delay has elapsed
            else if(ticksInState >= DELAY_START)
            {
                state = STATE_RUNNING;
                PIN_ENABLE(PIN_RELAY);
                DEBUG_PIN_DISABLE(PIN_DEBUG1);
            }
            break;
        case STATE_STOPPING:
            // Cancel stopping when current raises above threshold
            if(currentValue >= threshold)
            {
                state = STATE_RUNNING;
                DEBUG_PIN_DISABLE(PIN_DEBUG1);
            }
            // Open relay once stopping delay has elapsed
            else if(ticksInState >= DELAY_STOP)
            {
                state = STATE_STOPPED;
                PIN_DISABLE(PIN_RELAY);
                DEBUG_PIN_DISABLE(PIN_DEBUG1);
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
    DDRB = (1 << PIN_RELAY)
#if defined DEBUG
        | (1 << PIN_DEBUG1) | (1 << PIN_DEBUG2)
#endif
        ;
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
            threshold = read_adc(MUX_THRESHOLD);
            _delay_ms(1);
        }

        // Store max value of ADC
        uint16_t newCurrent = read_adc(MUX_CURRENT);
        currentValue = MAX(currentValue, newCurrent);
    }
}
