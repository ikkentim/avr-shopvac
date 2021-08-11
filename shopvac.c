#include <avr/io.h>
#include <util/delay.h>

int main() {
    DDRB = 0xFF; // All pins as output

    for(;;) {
        PORTB = 0xFF; // All pins on
        _delay_ms(200);

        PORTB = 0x00; // All pins off
        _delay_ms(500);
    }
}
