// CONFIG
#pragma config FOSC = INTRCIO   // Oscillator Selection bits (INTOSC oscillator: I/O function on GP4/OSC2/CLKOUT pin, I/O function on GP5/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = ON       // Power-Up Timer Enable bit (PWRT enabled)
#pragma config MCLRE = OFF      // GP3/MCLR pin function select (GP3/MCLR pin function is digital I/O, MCLR internally tied to VDD)
#pragma config BOREN = ON       // Brown-out Detect Enable bit (BOD enabled)
#pragma config CP = OFF         // Code Protection bit (Program Memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)

#include "includes.h"

#define ADDR_MIN_PC 1
#define ADDR_MAX_PC 2
#define ADDR_LAST_PC 3
#define ADDR_COUNT_PC 4
#define ADDR_ERR_PC 6

__EEPROM_DATA(
        0x05, // version
        0x00, // min pc
        0x00, // max pc (best value 80)
        0x00, // last pc
        0x00, // count pc L
        0x00, // count pc H
        0x00, // errors
        0xFF
        );

/* Pin configuration */

#define DATA_IN GP2
#define OUT_1 GP5
#define OUT_2 GP4
#define PWR_ON GP1

#define DATA_LEN 3

#define M_PREAMBULE 0
#define M_START 1
#define M_DATA 2
#define M_STOP 3

uint16_t t0;
uint16_t t1;
uint16_t dt0;
uint16_t dt1;
uint16_t ticker;

uint8_t pc; // preambule counter best is 80 = (10 * 8)
uint8_t dc; // data bits counter
uint8_t bc; // buffer counter

uint8_t buffer[DATA_LEN];

uint8_t mode;
uint8_t start_byte;
uint8_t current_bit;
bit last_bit;

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

void reset(void) {
    t0 = 0;
    t1 = 0;
    dt0 = 0;
    dt1 = 0;
    dc = 0;
    bc = 0;
    pc = 0;
    mode = M_PREAMBULE;
    start_byte = 0;
    last_bit = 0;
    current_bit = 0;
}

void interrupt globalInterrupt() {
    if (T0IF) {
        ticker++;

        if (PWR_ON) {
            if (ticker > 2) {
                // about 200us
                PWR_ON = 0;
                ticker = 0;
            }
        } else {
            if (ticker > 25) {
                // about 1.7s
                PWR_ON = 1;
                ticker = 0;
            }
        }

        T0IF = 0;
    }
}

uint16_t dt(uint16_t _t0, uint16_t _t1) {
    if (_t1 < _t0) {
        return 0xFFFF - _t0 + _t1;
    }
    return _t1 - _t0;
}

void EEPROM_WriteByte(uint8_t addr, uint8_t data8) {
    uint8_t status;
    while (WR);
    EEADR = addr;
    EEDATA = data8;
    WREN = 1;
    status = GIE;
    GIE = 0;
    EECON2 = 0x55;
    EECON2 = 0xAA;
    WR = 1;
    GIE = status;
    WREN = 0;
}

uint8_t EEPROM_ReadByte(uint8_t addr) {
    while (RD || WR);
    EEADR = addr;
    RD = 1;
    return EEDATA;
}

typedef union {

    struct {
        uint8_t LOW_BYTE;
        uint8_t HIGH_BYTE;
    } refined8;

    uint16_t raw;
} count_PC;

uint8_t _min;
uint8_t _max;
count_PC countPC;

void writePC() {
    if (pc > 0) {
        _min = EEPROM_ReadByte(ADDR_MIN_PC);
        _max = EEPROM_ReadByte(ADDR_MAX_PC);
        countPC.refined8.LOW_BYTE = EEPROM_ReadByte(ADDR_COUNT_PC);
        countPC.refined8.HIGH_BYTE = EEPROM_ReadByte(ADDR_COUNT_PC + 1);

        if (_min > pc || countPC.raw == 0) {
            EEPROM_WriteByte(ADDR_MIN_PC, pc);
        }
        if (_max < pc || countPC.raw == 0) {
            EEPROM_WriteByte(ADDR_MAX_PC, pc);
        }
        if (countPC.refined8.LOW_BYTE == 0xFF && countPC.raw > 0) {
            countPC.refined8.LOW_BYTE = 0;
            countPC.refined8.HIGH_BYTE++;
            EEPROM_WriteByte(ADDR_COUNT_PC + 1, countPC.refined8.HIGH_BYTE);
        } else {
            countPC.refined8.LOW_BYTE++;
        }
        EEPROM_WriteByte(ADDR_COUNT_PC, countPC.refined8.LOW_BYTE);
        EEPROM_WriteByte(ADDR_LAST_PC, pc);
    }
}

void process_bit() {

    current_bit = dt1 > dt0;

    int div;
    if (current_bit) {
        div = dt1 / dt0;
    } else {
        div = dt0 / dt1;
    }
    if (div > 3) {
        reset();
        return;
    }

    switch (mode) {
        case M_PREAMBULE:

            if (last_bit != current_bit) {
                last_bit = current_bit;
                pc++;
            } else if (current_bit == 0) {
                mode = M_START;
                start_byte = 0;
            } else {
                reset();
            }
            break;
        case M_START:
            start_byte = (start_byte << 1) + current_bit;
            if (start_byte & 0x08) {
                if (start_byte == 0x0F) {
                    bc = 0;
                    dc = 0;
                    mode = M_DATA;
                } else {
                    reset();
                }
            }
            break;
        case M_DATA:
            bc = dc >> 3; // (dc / 8) -> index of buffer array
            buffer[bc] = (buffer[bc] << 1) + current_bit;
            dc++;
            if (dc > (8 * DATA_LEN - 1)) {
                mode = M_STOP;
                writePC();
            }
            break;
        case M_STOP:
            break;
    }
}

void read() {

    if (DATA_IN) {
        t0 = TMR1;
        if (dt0 > 0) {
            // process last bit
            dt1 = dt(t1, t0);
            process_bit();
            dt1 = 0;
        }
        while (DATA_IN);
        t1 = TMR1;
        dt0 = dt(t0, t1);
    }

}

void setup() {
#ifdef _12F675
    ANSEL = 0x00; // Set ports as digital I/O, not analog input
    ADCON0 = 0x00; // Shut off the A/D Converter
#endif     
    CMCON = 0x07; // Shut off the Comparator
    VRCON = 0x00; // Shut off the Voltage Reference

    TRISIO = 0b00000100;
    GPIO = 0x00; // Make all pins 0
    WPU = 0b00000100;
    IOCB = 0b00000000; // interrupt for GP0
    INTEDG = 1; // 0 -> 1

    TMR1CS = 0; // TMR1 uses internal osc
    TMR1IE = 0; // disable tmr1 interrupt

    // Timer 0 divider: ticker step = 1ms
    PS0 = 1;
    PS1 = 1;
    PS2 = 1;
    T0CS = 0;
    PSA = 0;
    T0IE = 1; // enable timer0 interrupts
    TMR0 = 0; // reset timer

    INTF = 0;
    GPIE = 0;
    INTE = 0; // DATA_IN interrupt
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

void main() {

    setup();

    while (1) {

        read();

        if (mode == M_STOP) {
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
                }
            } else {
                GIE = 0;
                PWR_ON = 0;
                uint8_t err = EEPROM_ReadByte(ADDR_ERR_PC);
                err++;
                EEPROM_WriteByte(ADDR_ERR_PC, err);
                for (err = 0; err < 5; err++) {
                    PWR_ON = 1;
                    __delay_ms(150);
                    PWR_ON = 0;
                    __delay_ms(400);
                }
                PWR_ON = 0;
                GIE = 1;
            }
            __delay_ms(2000);
            reset();
        }
    }
}