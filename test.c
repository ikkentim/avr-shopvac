#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>

volatile struct {
  uint8_t dataByte, bitsLeft,
          pin, done;
} txData = {0, 0, 0, 0};

//---------------------------------------------------------
void sendBySerial(const uint8_t data) {

  txData.dataByte = data;
  txData.bitsLeft = 10;
  txData.done = 0;
  // Reset counter
  TCNT0 = 0;
  // Activate timer0 A match interrupt
  TIMSK = 1 << OCIE0A;

} // sendBySerial

//---------------------------------------------------------
// Blocking
void sendStrBySerial(char *p) {

  while (*p != '\0') {
    sendBySerial(*p++);
    while (!txData.done);
  } // while

} // sendStrBySerial

void sendNum(int num) {
    char n[8];

    itoa(num, n, 10);
        sendStrBySerial(n);
}

//---------------------------------------------------------
void initSerial(const uint8_t portPin) {

  txData.pin = portPin;
  // Define pin as output
  DDRB = 1 << txData.pin;
  // Set it to the default HIGH
  PORTB |= 1 << txData.pin;
  // Set top value for counter 0
  OCR0A = 26;
  // No A/B match output; just CTC mode
  TCCR0A = 1 << WGM01;
  // Set prescaler to clk/64
  TCCR0B = (1 << CS01) | 1 << (CS00);

} // initSerial;

uint16_t read_adc(uint8_t mux)
{
    ADMUX = mux;

    ADCSRA |= (1 << ADSC);              // Start conversion
    while(ADCSRA & (1 << ADSC));        // Wait for conversion to complete

    return ADC;
}
//---------------------------------------------------------
int main(void) {


    // Configure ADC
    ADCSRA =
        (1 << ADEN) |                   // Enable ADC
        (1 << ADPS2) |                  // 64 prescaler
        (1 << ADPS1);
  initSerial(PB0);

  // Enable interrupts
  sei();
  DDRB |= 1 << PB1;

  while(1) {
    PORTB ^= 1 << PB1;

    int n = read_adc((1 << MUX1) | (1 << MUX0));
    sendNum(n);
  }

} // main

//---------------------------------------------------------
// Timer 0 A-match interrupt
ISR(TIMER0_COMPA_vect) {

  uint8_t bitVal;

  switch (txData.bitsLeft) {

    case 10: bitVal = 0; break;
    case  1: bitVal = 1; break;
    case  0: TIMSK &= ~(1 << OCIE0A);
             txData.done = 1;
             return;

    default:
      bitVal = txData.dataByte & 1;
      txData.dataByte >>= 1;

  } // switch

  if (bitVal) PORTB |= (1 << txData.pin);
   else PORTB &= ~(1 << txData.pin);
  --txData.bitsLeft;

}
