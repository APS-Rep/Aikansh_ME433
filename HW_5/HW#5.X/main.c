#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR 
#include<stdio.h>
#define PIC32_baud 230400
#define step 0.01
#include<math.h>

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
void read_UART(char* msg, int maxL);
void write_UART(const char* str);
unsigned short gen_16bit(unsigned char choice, unsigned char signal);

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
    TRISAbits.TRISA1 = 0;       // Set A1 as output
    LATAbits.LATA1 = 0;
    TRISBbits.TRISB4 = 1;       // Set B4 as input
     
    __builtin_enable_interrupts();
    
    U1MODEbits.BRGH = 0; // set baud to PIC32_BAUD
    U1BRG = ((48000000 / PIC32_baud) / 16) - 1;
    
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
    
    // Enable UART
    U1MODEbits.ON = 1;
    
    initSPI();
    
    // declaring all variables here
    float time = 0;
    float sine;
    unsigned char s_wave, choice_sine=1, choice_tri=0;
    unsigned short signal_sine = 0;
    unsigned char sig_2, spio1, spio2;
    float x=0,y=0;
    unsigned char t_wave;
    unsigned short signal_triangle = 0;
    unsigned char sig_2_tri, spio1_tri, spio2_tri;
    
    // infinite loop to generate sine and triangular wave
    while(1)
    {
       time = time + step;
       sine = 128*(sin(2*M_PI*2*time)+255);
       
       // converting to unsigned char
       s_wave = sine;
       
       // generating a 16 bit signal 
       signal_sine = gen_16bit(choice_sine, s_wave);
       
       // sending sine wave
       LATAbits.LATA0 = 0;
       sig_2 = signal_sine >> 8;
       spio1 = spi_io(sig_2);
       spio2 = spi_io(signal_sine);
       LATAbits.LATA0 = 1;
       
       // generating the triangular wave
       x+=0.01;
       if(x < 0.5)
       {
            y = ((255.0/0.5)*x);
       }
       
       else if(x >= 0.5)
        {
            y = abs((255.0-((255.0/0.5)*x)) + 255.0);
        }
        
       else if(x>= 1)
       {
           x = 0;
       }
       
       t_wave = y;
       signal_triangle = gen_16bit(choice_tri, t_wave);
       
       // sending triangle wave
       LATAbits.LATA0 = 0;
       sig_2_tri = signal_triangle >> 8;
       spio1_tri = spi_io(sig_2_tri);
       spio2_tri = spi_io(signal_triangle);
       LATAbits.LATA0 = 1;   
    }
}


void initSPI() {
    // Pin B14 has to be SCK1
    // Turn of analog pins
    ANSELA = 0;
    
    // RPA0 is set for CS
    TRISAbits.TRISA0 = 0;
    LATAbits.LATA0 = 1;
    
    // Set RPA1 as SDO1
    RPA1Rbits.RPA1R = 0b0011;   
     
    // Setting SDI1
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

void read_UART(char* msg, int maxL)
{
    char info = 0;
    int flag=0,byteNum=0; //flag to check if message has been completed
    while(flag!=1)
    {
        if(U1STAbits.URXDA)
        {
            info = U1RXREG;
            if(info == '\n' || info == '\r')
            {
                flag++;
            }
            else
            {
                msg[byteNum] = info;
                byteNum+=1;
                if(byteNum>=maxL) //this is for rollover
                {
                    byteNum=0;
                }
            }
        }
    }
    msg[byteNum] = '\0';
}

void write_UART(const char* str)
{
    while(*str!='\0')
    {
        while(U1STAbits.UTXBF)
        {
            ;
        }
        U1TXREG = *str;
        ++str;
    }
}

unsigned short gen_16bit(unsigned char choice, unsigned char signal) {
    unsigned short s;
    s = 0;
    s = s | (choice << 15);
    s = s | (0b111 << 12);
    s = s | (signal << 4);

    return s;
}
    
