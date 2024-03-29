#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#define DELAYTIME 12000000
#define NU32_DESIRED_BAUD 230400    // Baudrate for RS232
#include <stdio.h>
#include <math.h>
#include "ws2812b.h"



// DEVCFG0
#pragma config DEBUG = OFF // disable debugging
#pragma config JTAGEN = OFF // disable jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // disable flash write protect
#pragma config BWP = OFF // disable boot write protect
#pragma config CP = OFF // disable code protect

// DEVCFG1
#pragma config FNOSC = FRCPLL // use fast frc oscillator with pll
#pragma config FSOSCEN = OFF // disable secondary oscillator
#pragma config IESO = OFF // disable switching clocks
#pragma config POSCMOD = OFF // primary osc disabled
#pragma config OSCIOFNC = OFF // disable clock output
#pragma config FPBDIV = DIV_1 // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // disable clock switch and FSCM
#pragma config WDTPS = PS1 // use largest wdt value
#pragma config WINDIS = OFF // use non-window mode wdt
#pragma config FWDTEN = OFF // wdt disabled
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz fast rc internal oscillator
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz //4
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV //80
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz

// DEVCFG3
#pragma config USERID = 0000 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations



int main() {

    __builtin_disable_interrupts(); // disable interrupts while initializing things

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    __builtin_enable_interrupts();
    TRISBbits.TRISB4 = 1; //INITIALISE B4 as input
    TRISAbits.TRISA4 = 0; //Initialise A4 as output
    LATAbits.LATA4 = 0; //initially off


    U1MODEbits.BRGH = 0; // set baud to NU32_DESIRED_BAUD
    U1BRG = ((48000000 / NU32_DESIRED_BAUD) / 16) - 1;

    // 8 bit, no parity bit, and 1 stop bit (8N1 setup)
    U1MODEbits.PDSEL = 0;
    U1MODEbits.STSEL = 0;

    // configure TX & RX pins as output & input pins
    U1STAbits.UTXEN = 1;
    U1STAbits.URXEN = 1;
    // configure hardware flow control using RTS and CTS
    U1MODEbits.UEN = 2;  

    U1RXRbits.U1RXR = 0b0000; // Set A2 to U1RX
    RPB3Rbits.RPB3R = 0b0001; // Set B3 to U1TX


    // enable the uart
    U1MODEbits.ON = 1;
    
    ws2812b_setup();
    
    
    // create a structure that keeps track of four LEDs
    wsColor led1[4];

    
    ws2812b_setColor(led1,4);


    int k=0;
    while(1){
        
        for(int i = 1; i <= 360; i++){

            for (int j = 0; j < 4; j++){
                k=i+72*j;
                if (i+72*j   >  360){k=i+72*j-360;}
                led1[j] = HSBtoRGB(k,1,1);
            
                ws2812b_setColor(led1, 4);
            }
        }
        
        
    }


}