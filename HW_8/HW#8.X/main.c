#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#define DELAYTIME 1200000
#define NU32_DESIRED_BAUD 230400    // Baudrate for RS232
#include <stdio.h>
#include <math.h>
#include "i2c_master.h"
#include "ssd1306.h"
#include "font.h"



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

void char_draw(char character, unsigned char x, unsigned char y);
void str_draw(unsigned char x, unsigned char y, char *msg);


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

    // do your TRIS and LAT commands here
    i2c_master_setup();
    ssd1306_setup();


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


    char msg[100];
    char fps_msg[100];

    sprintf(msg, "Twinkle Twinkle");
    float fps = 0.0;
    int i = 0;
    int j = 0;
    ssd1306_clear();
    while(1){    
        _CP0_SET_COUNT(0);
        
        // LED BLINK
        // I am using rand() to randomize the duration of the LED blink.
        // This ensures the _CP0_GET_COUNT() value is varied enough to ensure the screen updates the fps value
        j = rand() % (250000 + 1 - 100000) + 100000;
        while (i < j) {
             LATAbits.LATA4 = 1;
             i+=1;
         }
        i=0;
        j = rand() % (250000 + 1 - 100000) + 100000;
        while (i < j) {
             LATAbits.LATA4 = 0;
             i+=1;
        }
        i=0;
        j = rand() % (250000 + 1 - 100000) + 100000;
        while (i<j) {
             LATAbits.LATA4 = 1;
             i+=1;
         }
        i=0;
        j = rand() % (250000 + 1 - 100000) + 100000;
         while (i<j) {
             LATAbits.LATA4 = 0;
             i+=1;
         }

        
        //Variable Draw
         str_draw(5, 5, msg);
         
         fps = (240000000.0/_CP0_GET_COUNT());
         sprintf(fps_msg, "fps = %f", fps);

         str_draw(30, 25, fps_msg);
         ssd1306_update();          
    }


}


// Draw a char
void char_draw(char character, unsigned char x, unsigned char y){
    
    for (int i = 0; i < 5; i++){
        char col = ASCII[character - 32][i];
        for (int j = 0; j < 8; j++){
            int on_or_off = (col >> j) & 0b1;
            ssd1306_drawPixel(x+i, y+j, on_or_off);        
        }
       
        
    }
  
}

// Draw a string
void str_draw(unsigned char x, unsigned char y, char *msg){
    int i = 0;
    while(msg[i] != 0){
        char_draw(msg[i], x+(i*5), y);
        i = i + 1;
    }
    


}