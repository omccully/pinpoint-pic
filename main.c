/* 
 * File:   main.c
 * Author: Pinpoint
 *
 * Created on March 22, 2017, 12:47 PM
 * Last Updated on March 28, 2017, 8:17 PM
 * 
 * Pin assignments:
 *  http://ww1.microchip.com/downloads/en/DeviceDoc/41391D.pdf
 *  RA0/C12IN0- - microphone
 *  RA1/AN1 - accelerometer Z
 *  RA2/AN2 - accelerometer Y
 *  RA3/AN3 - accelerometer X
 *  RB0 - button (normally high)
 *  RB1 - hardware RX (to GPS TX)
 *  RB2 - hardware TX (to transceiver RX)
 *  RB3 - software TX (to GPS RX)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pic16lf1827.h>
#include <htc.h>

#define _XTAL_FREQ 500000L

#define MAX_BUFFER_SIZE 80
#define ALERT_BUFFER_SIZE 16

// microphone panic voltage
#define MP_PANIC_V 2.0

// accelerometer panic voltage
#define AC_PANIC_V 2.0
#define AC_PANIC_SUM 80

#define DACR_INIT min(0b1111, (unsigned char)((MP_PANIC_V / 3.3) * 32))
#define AD_PANIC_VAL min(0xFF, (unsigned char)((AC_PANIC_V / 3.3) * 256))   

#define GPS_COMMAND_COUNT 2
const char * const gps_commands[] = { 
    "$PMTK220,5000*1B\r\n", 
    "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n" 
};
const char * const gps_acks[] = { 
    "$PMTK001,220,3*30\r\n", 
    "$PMTK001,314,3*36\r\n" 
};

char nmea_buffer[MAX_BUFFER_SIZE];
unsigned char nmea_buffer_index = 0;
unsigned char nmea_buffer_ready = 0;

char alert_buffer[ALERT_BUFFER_SIZE];
unsigned char alert_buffer_index = 0;
unsigned char alert_buffer_ready = 0; 

char send_buffer[MAX_BUFFER_SIZE];
unsigned char send_buffer_index = 0;
unsigned char send_buffer_locked = 0;

unsigned char alert_occurred = 0; // within the last 1 minute
unsigned char of_since_alert = 0; // t0 overflows since last alert
char panic_type = 0;

void configure_gps();
void start_panicing(char panic_type);
void uart_send_str(char* str);
char read_ad(unsigned char an_pin);

void sw_uart_send_bit(char b);
void sw_uart_send_char(char c);
void sw_uart_send_str(char* str);

unsigned char first = 1;

static void interrupt isr(void) {
    if(RCIF) {
        if(FERR) {
            // framing error
        }
        
        if(OERR) {
            // receive overrun error (26.1.2.5)
            CREN = 0;
            NOP();
            CREN = 1;
        }
        
        nmea_buffer_ready = 0;
        nmea_buffer[nmea_buffer_index] = RCREG;
        if(nmea_buffer[nmea_buffer_index] == '$') {
            nmea_buffer[0] = '$';
            nmea_buffer_index = 1;
        } else if(nmea_buffer[nmea_buffer_index] == '\n' || 
                nmea_buffer[nmea_buffer_index] == '\r' ||
                nmea_buffer[nmea_buffer_index] == '\0' ||
                nmea_buffer_index + 3 >= MAX_BUFFER_SIZE) {
            // nmea string ended, so it's ready to send
            // whether or not it ends with \r\n\0, it should work
            if(nmea_buffer_index < 3) {
                nmea_buffer_index = 0; // restart string
            } else {
                // must add null character at end of the string
                nmea_buffer[nmea_buffer_index + 0] = '\r'; 
                nmea_buffer[nmea_buffer_index + 1] = '\n'; 
                nmea_buffer[nmea_buffer_index + 2] = '\0'; 

                nmea_buffer_index = 0; // for receiving the next NMEA string

                if(send_buffer_locked) {
                    // there's already a string being sent by the UART, so
                    // set the flag so that the TXIF interrupt will send out
                    // this string as soon as it's able to.
                    nmea_buffer_ready = 1;
                } else {
                    // the UART isn't busy, so the NMEA string can be sent now
                    uart_send_str(nmea_buffer);
                    nmea_buffer_ready = 0;
                }    
            }  
        } else {
            nmea_buffer_index++;
        }
    }
    
    // only the interrupt is allowed to decide when to send data. 
    
    if(TXIF) {
        if(send_buffer_locked) {
            // send next character
            TXREG = send_buffer[send_buffer_index];
            send_buffer_index++;
            if(send_buffer[send_buffer_index] == '\0') {
                send_buffer_locked = 0;
                
                // if the UART is busy when requested, turn on a bool val for the buffer
                // that tells this part of the program to send the string
                
                // prioritize sending alerts over GPS data
                if(alert_buffer_ready) {
                    uart_send_str(alert_buffer);
                    alert_buffer_ready = 0;
                } else if(nmea_buffer_ready) {
                    uart_send_str(nmea_buffer);
                    nmea_buffer_ready = 0;
                } else {
                    TXIE = 0; // disable TX interrupt
                }
            }
        } else {
            TXIE = 0; // disable TX interrupt
        }
    }
    
    if(TMR0IF) {
        // TMR0IF occurs every 8uS * 2^8 * 256 = 0.524 seconds
        if(alert_occurred) {
            of_since_alert++;
            
            if(of_since_alert % 10 == 0) {
                // every 5 seconds
                unsigned int seconds = (of_since_alert * 262) / 500; // 0.524288
                
                sprintf(alert_buffer, "@PANIC-%c-%d\r\n", panic_type, seconds); // 10 = seconds since alert
                if(send_buffer_locked) {
                    // there's already a string being sent by the UART, so
                    // set the flag so that the TXIF interrupt will send out
                    // this string as soon as it's able to.
                    alert_buffer_ready = 1;
                } else {
                    // the UART isn't busy, so the alert string can be sent now
                    uart_send_str(alert_buffer);
                    alert_buffer_ready = 0;
                }
            }
            
            if(of_since_alert >= 120) { 
                // a little more than a minute has passed
                of_since_alert = 0;
                alert_occurred = 0; // forget about the last alert
            }
        }
        TMR0IF = 0;
        
    }
    
    
    if(INTF) {
        // button pressed
        NOP();
        start_panicing('B'); // button panic
        INTF = 0;
    }
    
    if(C1IF) {
        // microphone voltage level exceeded the set point
        if(first) {
            // weird bug causes C1IF to be set when program starts sometimes
            first = 0;
        } else {
            start_panicing('M'); // microphone panic
        }
         
        C1IF = 0;
    }
}

__CONFIG(WDTE_OFF);

int main() {
    OSCCONbits.IRCF = 0b0111; // 500 kHz clock 

    // configure GPS chip using software UART with T1 or something
    // before any other interrupts are configured
    TRISAbits.TRISA0 = 1; // RA0/AN0 input for microphone
    TRISAbits.TRISA1 = 1; // RA1/AN1 input for accelerometer Z
    TRISAbits.TRISA2 = 1; // RA2/AN2 input for accelerometer Y
    TRISAbits.TRISA3 = 1; // RA3/AN3 input for accelerometer Y
    ANSELA = 0b00001111; // RA0, RA1, RA2, RA3 analog
    
    TRISBbits.TRISB0 = 1; // RB0 input for interrupt button
    TRISBbits.TRISB1 = 1; // RB1 input for RX
    TRISBbits.TRISB2 = 0; // RB2 output for TX
    ANSELB = 0b00000000; 

    BAUDCON = 0x08; // 16-bit baud rate generator
    TXSTA = 0xA4; // master mode (clk generated internally from BRG)
                  // 8 bit transmission, transmit enabled
    RCSTA = 0x90; // serial port enable, enables receiver
    
    SPBRGH = 0;
    SPBRGL = 12; // baud rate 9615
    
    TXIE = 1; // enable USART transmit interrupt (from PIE1)
    RCIE = 1; // enable USART receive interrupt (from PIE1)
    
    CREN = 1; // enable USART reception
    
    TMR0IE = 1; // enable timer0 overflow interrupt
    OPTION_REG = 0x07; // TIMER0 prescaler = 256
    //OPTION_REGbits.INTEDG = 0; // INT on falling edge
    OPTION_REGbits.INTEDG = 1; // INT on rising edge
    INTE = 1; // enable INT external interrupt
    
    // CM1 
    C1SP = 1; // C1 operates in normal power, higher speed mode
    C1INTP = 1; // C1IF flag will be set upon a positive going edge of the C1OUT bit
    C1ON = 1; // C1 enabled and consumes no active power
    CM1CON1bits.C1PCH = 0b01; // C1VP connects to DAC voltage reference
    CM1CON1bits.C1NCH = 0b00; // C1VN connects to RA0/C12IN0- pin (microphone)
    C1POL = 0; // not inverted
    C1INTP = 0; // disable positive edge interrupt for comparator 1
    C1INTN = 1; // enable negative edge interrupt for comparator 1
    
    DACEN = 1; // DAC enable    DACCON0bits.DACPSS = 0b00; 
    DACCON0bits.DACPSS = 0b00; // DAC positive source = Vdd
    DACNSS = 0; // DAC negative source = Vss
    DACCON1bits.DACR = DACR_INIT; // configure the panic point
    DACEN = 1; // DAC enable
    
    ADFM = 0; // left justified, just read 8 bit val from ADRESH
    ADCON1bits.ADCS = 0b000; // A/D conversion clock = Fosc/2
    ADON = 1; // turn AD on
    
    __delay_ms(1000); // give time for the GPS chip to start up
    configure_gps();
    
    // INTCON
    GIE = 1; // enables all active interrupts
    PEIE = 1; // enables all active peripheral interrupts
    
    int x, y, z, sum;
    char za, ya, xa;
    
    while(1) {
        // assuming 255 is +16G and 0 is -16G for each axis
        // just going to add the axes because floating point operations 
        // and trigonometric functions will take too many clock cycles
        // to determine an accurate 3D vector magnitude
        
        z = (za = read_ad(1)) - 0x7F;
        y = (ya = read_ad(2)) - 0x7F;
        x = (xa = read_ad(3)) - 0x7F;
        sum = abs(x) + abs(y) + abs(z);
        if(sum > AC_PANIC_SUM) {
            GIE = 0; 
            PEIE = 0;
            start_panicing('A'); // accelerometer panic
            GIE = 1;
            PEIE = 1;
        }
        __delay_ms(1);
    }

    // TODO: write a while loop to determine max counts of a counter that
    // the program is in an interrupt for
    
    return (EXIT_SUCCESS);
}

void configure_gps() {
    // 4 MHz clock needed for 9600 baud rate software UART
    unsigned char old_ircf = OSCCONbits.IRCF;
    unsigned char old_spbrgl = SPBRGL;
    OSCCONbits.IRCF = 0b1101; 
    SPBRGL = 103; // baud rate 9615
    
    // use software serial to send commands to GPS chip
    // use timer2 for baud rates
    TRISBbits.TRISB3 = 0; // RB3 output for software TX
    PORTBbits.RB3 = 1; // UART stays normally high
    PR2 = 104; // 104 counts is 1/9600 sec
    T2CON = 0x04; // timer2 on, prescaler/postscaler = 1

    char c;
    unsigned char i, j;
    for(i = 0; i < GPS_COMMAND_COUNT; i++) {
        unsigned char acknowledged;
        // may have issues with ROM pointers
        do {
           acknowledged = 1;
           sw_uart_send_str(gps_commands[i]);
           for(j = 0; j < MAX_BUFFER_SIZE && gps_acks[i][j] != '\0'; j++) {
                if(OERR) {
                    // receive overrun error (26.1.2.5)
                    CREN = 0;
                    NOP();
                    CREN = 1;
                }
               
                while(!RCIF);
                // nmea_buffer refister 
                c = RCREG;
                if(c != gps_acks[i][j]) {
                    acknowledged = 0;
                    // wait for rest of back ACK to flood in
                    __delay_ms(100); 
                    break;
                }       
            }          
        } while(!acknowledged);
    }

    OSCCONbits.IRCF = old_ircf; // restore clock speed
    SPBRGL = old_spbrgl;
}

void sw_uart_send_bit(char b) {
    while(!TMR2IF); // wait until T2 interrupt (ready to send)
    PORTBbits.RB3 = b; // send b bit
    TMR2IF = 0;
}

void sw_uart_send_char(char c) {
    unsigned char i; 
    sw_uart_send_bit(0); // send start bit
   
    for(i = 0; i < 8; i++) {
        sw_uart_send_bit(c & 1); // send LSB of c
        c >>= 1; // rotate c right 1
    }
    sw_uart_send_bit(1); // send stop bit  
}

void sw_uart_send_str(char* str) {
    unsigned char i, j;

    TMR2IF = 0; // make sure the first bit is held for the full bit time period
    for(i = 0; str[i] != '\0'; i++) {
        sw_uart_send_char(str[i]);
    }
}

/*
 * Purpose: read digital result from A/D converter
 * inputs: none
 * outputs: digital equivalent of selected analog value
 */
char read_ad(unsigned char an_pin) {
    ADCON0bits.CHS = an_pin;
    NOP();
    ADCON0bits.GO = 1; // // start A/D conversion
    
    // wait for A/D conversion to complete
	while (ADCON0bits.GO);
    
    return ADRESH;
}

/* 
 * ONLY call this function from within an interrupt
 * or disable interrupts before checking to see if !send_buffer_locked
 */
void uart_send_str(char* str) {
    // if str == NULL, programmer already put data in send_buffer
    if(str != NULL) strcpy(send_buffer, str); // send_buffer = str
    send_buffer_index = 0;
    
    // if the TXIF flag is already set, we must put bytes into TXREG
    // until the TXIF flag clears, so the interrupt will happen
    while(TXIF && send_buffer[send_buffer_index] != '\0') {
        TXREG = send_buffer[send_buffer_index];
        send_buffer_index++;
    }
    
    send_buffer_locked = 1; // allows interrupt to start sending the data
    TXIE = 1; // enables the TX interrupt
}

/* 
 * ONLY call this function from within an interrupt
 * or disable interrupts before calling
 */
void start_panicing(char pt) {
    if(pt == panic_type && alert_occurred && of_since_alert < 10) {
        // don't repeat alerts for at least 5 seconds
        return;
    }
    
    panic_type = pt;
    sprintf(alert_buffer, "@PANIC-%c-0\r\n", panic_type); // 0 = seconds since alert
    alert_occurred = 1;
    of_since_alert = 0;
    TMR0 = 0; 

    if(send_buffer_locked) {
        // there's already a string being sent by the UART, so
        // set the flag so that the TXIF interrupt will send out
        // this string as soon as it's able to.
        alert_buffer_ready = 1;
    } else {
        // the UART isn't busy, so the alert string can be sent now
        uart_send_str(alert_buffer); 
        alert_buffer_ready = 0;
    }
}