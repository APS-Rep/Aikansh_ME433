#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR 

#include<stdio.h>

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
#pragma config WDTPS = PS1048576 // use largest wdt value
#pragma config WINDIS = OFF // use non-window mode wdt
#pragma config FWDTEN = OFF // wdt disabled
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz fast rc internal oscillator
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz

// DEVCFG3
#pragma config USERID = 0x1234 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations

void initSPI();
unsigned char spi_io(unsigned char o);

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
    TRISAbits.TRISA4 = 0;       //Set A4 as output
    LATAbits.LATA4 = 0;
    TRISBbits.TRISB4 = 1;       //Set B4 as input 
    
    // Used for the SPI DAC
    TRISAbits.TRISA1 = 0;       // Set A1 as output
    LATAbits.LATA1 = 0;
    TRISBbits.TRISB5 = 1;       // Set B5 as input
    
    TRISBbits.TRISB6 = 0;       // Set A0 as output
    LATBbits.LATB6 = 0;  
    
    
    
    __builtin_enable_interrupts();
    
    initSPI();
    unsigned char i=0;
    
    while (1) {
        // write one byte over SPI 
        LATBbits.LATB6 = 0; // bring CS low
        
        i++;
        if(i == 100)
        {
            i=0;
        }
        _CP0_SET_COUNT(0);
        while(_CP0_GET_COUNT() < 48000000/2)
        {
            ;
        }                  
    }
}


void initSPI() {
    // Pin B14 has to be SCK1
    // Turn of analog pins
    ANSELA = 0;
    
    // RPA0 is set for CS
    
    // Set RPA1 as SDO1
    RPA1Rbits.RPA1R = 0b0011;   
     
    // Set RPB5 as SDI1
    SDI1Rbits.SDI1R = 0b0001; 

    
    // setup SPI1
    SPI1CON = 0; // turn off the spi module and reset it
    SPI1BUF; //  clear the rx buffer by reading from it
    SPI1BRG = 1000; // 12 kHz; 
    SPI1STATbits.SPIROV = 0; // clear the overflow bit
    SPI1CONbits.CKE = 1; // data changes when clock goes from hi to lo (since CKP is 0)
    SPI1CONbits.MSTEN = 1; // master operation
    SPI1CONbits.ON = 1; // turn on spi 
}


// send a byte via spi and return the response
unsigned char spi_io(unsigned char o) {
  SPI1BUF = o;
  while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
    ;
  }
  return SPI1BUF;
}