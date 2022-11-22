#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#define DELAYTIME 12000000
#define NU32_DESIRED_BAUD 230400    // Baudrate for RS232
#include <stdio.h>
#include <math.h>
#include "i2c_master_noint.h"




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

void gen_write(unsigned char some_register, unsigned char data);
unsigned char gen_read(unsigned char some_register2);

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
    int x = 1200000;

    
    gen_write(0b00000000, 0b00111111); //set IODIR
   
   
    while(1){
        _CP0_SET_COUNT(0);
           LATAbits.LATA4 = 1;
           while(_CP0_GET_COUNT() <= x)
           {
               ;
           }
           _CP0_SET_COUNT(0);
           LATAbits.LATA4 = 0;
           while(_CP0_GET_COUNT() <= x)
           {
               ;
           }
           _CP0_SET_COUNT(0);
           LATAbits.LATA4 = 1;
           while(_CP0_GET_COUNT() <= x)
           {
               ;
           }
           _CP0_SET_COUNT(0);
           LATAbits.LATA4 = 0;
           while(_CP0_GET_COUNT() <= x)
           {
               ;
           }
                
        
        unsigned char result_2 = gen_read(0b00001001);//gpio
        
        if((result_2 & 0b00000001) == 0b00000000)
        {
            gen_write(0b00001010, 0b11000000);  
            
        }
           
        else{
                gen_write(0b00001010, 0b00000000); 
            }
        }
        
        
        
       
    }




void gen_write(unsigned char register_number, unsigned char value){
    
    //write step
    i2c_master_start();
    i2c_master_send(0b01000000);
    i2c_master_send(register_number);
    i2c_master_send(value);
    i2c_master_stop();
    
}


unsigned char gen_read(unsigned char register_number){
    
    i2c_master_start(); //add start bit
    i2c_master_send(0b01000000); //add write address
    i2c_master_send(register_number);
    i2c_master_restart();
    i2c_master_send(0b01000001); //add read address
    unsigned char result = i2c_master_recv();
    i2c_master_ack(1);
    i2c_master_stop();
    
    return result;
    
}