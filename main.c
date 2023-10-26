// CONFIG
#pragma config FOSC = INTRCIO   // Oscillator Selection bits (INTOSC oscillator: I/O function on GP4/OSC2/CLKOUT pin, I/O function on GP5/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = ON       // Power-Up Timer Enable bit (PWRT enabled)
#pragma config MCLRE = OFF      // GP3/MCLR pin function select (GP3/MCLR pin function is digital I/O, MCLR internally tied to VDD)
#pragma config BOREN = ON       // Brown-out Detect Enable bit (BOD enabled)
#pragma config CP = OFF         // Code Protection bit (Program Memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)

#include "includes.h"

/* Pin configuration
 * 
 * GP0 = Data input
 * GP1 = LED 1
 * GP2 = LED 2
 */

#define DATA_IN GP2
#define OUT_1 GP0
#define OUT_2 GP1
#define OUT_3 GP4
#define OUT_4 GP5

#define THRESHOLD 300

volatile uint16_t t0;
volatile uint16_t t1;
volatile bit bitReady;

uint16_t t2;
uint16_t dt0;
uint16_t dt1;
uint16_t dif;

uint8_t pc; // preambule counter
uint8_t dc; // data bits counter
uint8_t bc; // buffer counter

bit waitData;
bit _0;
bit _1;

uint8_t buffer[3];
bit dataReady;

uint8_t crcByte(uint8_t crc, uint8_t data) {
    uint8_t i = 8;
    while (i--) {
        crc = ((crc ^ data) & 1) ? (crc >> 1) ^ 0x8C : (crc >> 1);
        data >>= 1;
    }
    return crc;
}

uint8_t crc8(uint8_t buffer[], uint8_t size) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < size; i++) crc = crcByte(crc, buffer[i]);
    return crc;
}

uint8_t crcXOR(uint8_t *buffer, uint8_t size) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < size; i++) crc ^= buffer[i];
    return crc;
}

void send(uint16_t data) {
    //GIE = 0;
    OUT_1 = 1;
    __delay_ms(3);
    OUT_1 = 0;
    for (uint8_t i = 0; i < 16; i++) {
        if (OUT_1 == 0) OUT_1 = 1;
        else OUT_1 = 0;
        if ((data << i)&0x8000) {
            __delay_ms(2);
        } else {
            __delay_ms(1);
        }
    }
    if (OUT_1 == 0) OUT_1 = 1;
    else OUT_1 = 0;
    __delay_ms(3);
    OUT_1 = 0;
    //GIE = 1;
}

void getTimeout(unsigned short t) {
    OUT_1 = 1;
    while (t > 0) {
        __delay_ms(1);
        t--;
    }
    OUT_1 = 0;
}

void reset(void) {
    t0 = 0;
    t1 = 0;
    t2 = 0;
    dt0 = 0;
    dt1 = 0;
    pc = 0;
    dataReady = 0;
    waitData = 0;
    dc = 0;
    bc = 0;
    bitReady = 0;
}

void interrupt globalInterrupt() {
    if (INTF) {
        INTF = 0;
        if (!bitReady) {
            t0 = TMR1;
            while (DATA_IN);
            t1 = TMR1;
            bitReady = 1;
        }
    }
}

void setup() {
    CMCON = 0x07; // Shut off the Comparator
    VRCON = 0x00; // Shut off the Voltage Reference
    //    ANSEL = 0;
    //    ADCON0 = 0;

    TRISIO = 0b00000100;
    GPIO = 0x00; // Make all pins 0
    WPU = 0b00000100;
    IOCB = 0b00000000; // interrupt for GP0
    INTEDG = 1; // 0 -> 1

    TMR1CS = 0; // TMR1 uses internal osc
    TMR1IE = 0; // disable tmr1 interrupt

    INTF = 0;
    GPIE = 0;
    INTE = 1;
    PEIE = 0;

    TMR1ON = 0; // disable TMR1
    TMR1 = 0; // reset timer
    // Timer 1 divider (1000 ticks -> 1ms)
    T1CKPS0 = 0;
    T1CKPS1 = 0;
    TMR1ON = 1; // enable TMR1

    reset();
    GIE = 1; // enable global interrupts

}

void processBit() {

    if (pc > 0) {
        if (t0 < t2) {
            dt1 = 0xFFFF - t2 + t0;
        } else {
            dt1 = t0 - t2;
        }
    }

    // check time after last bit
    if (pc == 0 || (dt1 < (dt0 + dt0 + dt0) && dt1 > dt0 / 2)) {

        t2 = t1;

        if (t1 < t0) {
            dt1 = 0xFFFF - t0 + t1;
        } else {
            dt1 = t1 - t0;
        }

        if (dt0 == 0) {
            dt0 = dt1;
            pc++;

        } else {

            _0 = 0;
            _1 = 0;

            if (dt0 > dt1) {
                dif = dt0 - dt1;
            } else {
                dif = dt1 - dt0;
            }

            if (dif <= THRESHOLD) {
                _0 = 1;
            } else { 
                if (2 * dt0 > dt1) {
                    dif = 2 * dt0 - dt1;
                } else {
                    dif = dt1 - 2 * dt0;
                }

                if (dif <= 2 * THRESHOLD) {
                    _1 = 1;
                }
            }

            if (waitData == 1) {

                bc = dc >> 3; // dc / 8 -> index of buffer array

                if (_0) {
                    // rx 0
                    buffer[bc] = buffer[bc] << 1;
                    dc++;
                } else if (_1) {
                    // rx 1
                    buffer[bc] = (buffer[bc] << 1) + 1;
                    dc++;
                } else {
                    // error
                    buffer[0] = 0;
                    buffer[1] = 0;
                    buffer[2] = 0;
                    reset();
                }

                if (dc == 24) {
                    waitData = 0;
                    dataReady = 1;
                }

            } else {
                if (pc > 10 && _1) {
                    // start bit detected
                    waitData = 1;

                } else if (_0) {
                    dt0 = (dt0 + dt1) / 2;
                    //dt0 = dt1;
                    if (pc < 200)pc++;

                } else {
                    //error
                    reset();
                }
            }
        }
    } else {
        reset();
    }
//    OUT_1 = 0;
//    OUT_2 = 0;
//    OUT_3 = 0;
    bitReady = 0;

}

void main() {

    setup();

    while (1) {

        if (!dataReady && bitReady) {
            processBit();
        }

        if (dataReady) {
            GIE = 0;
            uint8_t crc = 0;
            crc = crc8(buffer, 2);

            if (crc == buffer[2] && buffer[0] == ADDRESS) {
                // CRC and address correct
                switch (buffer[1]) {
                    case 1:
                        if (OUT_1)
                            OUT_1 = 0;
                        else
                            OUT_1 = 1;
                        break;
                    case 2:
                        if (OUT_2)
                            OUT_2 = 0;
                        else
                            OUT_2 = 1;
                        break;
                    case 3:
                        if (OUT_3)
                            OUT_3 = 0;
                        else
                            OUT_3 = 1;
                        break;
                    case 4:
                        if (OUT_4)
                            OUT_4 = 0;
                        else
                            OUT_4 = 1;
                        break;
                }
            }
            __delay_ms(2000);
            reset();
            GIE = 1;

        }
    }
}