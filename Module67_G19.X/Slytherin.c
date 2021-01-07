
#include "xc.h"
#include <stdio.h>
#include <stdbool.h>
#include "configuration.h"
#include <math.h>

#define ID 0x30

volatile int millis=0;
int pulseX = 0;
int pulseY = 0;

int Command_Sethome = 0;
int Command_Write_POSX = 0;
int Command_Write_POSY = 0;

volatile int uartData = 0;

float POSX_UART = 0;
float POSY_UART = 0;


float Target_POS_X = 0;
float Target_POS_Y = 0;
float ErrorX_P = 0;
float ErrorY_P = 0;
float OutputX_P = 0;
float OutputY_P = 0;

const float Kp_px = 0.08;
const float Ki_px = 0;
const float Kd_px = 0;

const float Kp_py = 0.08;
const float Ki_py = 0;
const float Kd_py = 0;

float b,c,e,f;
int a,d,g,h;
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int package_uart(unsigned char data)
{
    int result = 0;
    static int state = 0;
    static unsigned int Highbyte_X = 0;
    static unsigned int Highbyte_Y = 0;
    static unsigned int Lowbyte_X = 0;
    static unsigned int Lowbyte_Y = 0;

    if ((state <= 1))
    {
        if (data == 0xFF)
        {
            state++;
        }
        else
        {
            result = 0xAC;
            state = 0;
        }
    }
    else if (state == 2)
    {
        if (data == ID)
        {
            state++;
        }
        else
        {
            result = 0xAC;
            state = 0;
        }
    }
    else if (state == 3)
    {
        if (data == 0xFA)
        {
            Command_Sethome = 1;
            state = 0;
        }
        if (data == 0xFB)
        {
            Command_Write_POSX = 1;
            state = 0;
        }
        if (data == 0xFD)
        {
            Command_Write_POSY = 1;
            state = 0;
        }
        if (data == 0xFC)
        {
            state++;
        }
        else
        {
            result = 0xAC;
            state = 0;
        }
    }
    else if (state == 4)
    {
        Highbyte_X = data * 100;
        state++;
    }
    else if (state == 5)
    {
        Lowbyte_X = data;
        POSX_UART = Highbyte_X + Lowbyte_X;
        state++;
    }
    else if (state == 6)
    {
        Highbyte_Y = data * 100;
        state++;
    }
    else if (state == 7)
    {
        Lowbyte_Y = data;
        POSY_UART = Highbyte_Y + Lowbyte_Y;
        state++;
    }
    else if (state == 8)
    {
        if (data == 0xFE)
        {
            Target_POS_X = (POSX_UART/60)*1024*2;
            Target_POS_Y = (POSY_UART/60)*1024*2;

            T1CONbits.TON = 1;
            printf("xmm=%d , x=%d | ymm=%d , y=%d\n",POSX_UART,Target_POS_X,POSY_UART,Target_POS_Y);
            T3CONbits.TON = 1;
            state = 0;
        }
    }
    return result;
}

void send_package(unsigned char data)
{
    U1TXREG = data;
    while (U1STAbits.TRMT == 0)
    {
    }
}

void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt(void)
{
    IFS0bits.U1TXIF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void)
{
    unsigned char data = U1RXREG;
    uartData = package_uart(data);
    IFS0bits.U1RXIF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _U1ErrInterrupt(void)
{
    U1STAbits.OERR == 1 ? U1STAbits.OERR = 0 : Nop();
    IFS4bits.U1EIF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void)
{
    //Position Control X
    ErrorX_P = (float)(Target_POS_X - POS1CNT);
    if(ErrorX_P<-35000){
        ErrorX_P += 65535;
    }
    OutputX_P = Kp_px * ErrorX_P;

    // Position Control Y
    ErrorY_P =  (float)(Target_POS_Y-POS2CNT); 
    if(ErrorY_P<-35000){
        ErrorY_P += 65535;
    }
    OutputY_P = Kp_py * ErrorY_P;
    int dutyx = 0;
    int dutyy = 0;
    
    a = POS1CNT;
    b = ErrorX_P;
    c = OutputX_P;
    d = POS2CNT;
    e = ErrorY_P;
    f = OutputY_P;
    
    if (OutputX_P >=0)
    {
        dutyx=OutputX_P;
        if(OutputX_P>=90){
            dutyx=80;
        }
        if(OutputX_P<=9){
            dutyx=0;
        }
        drive_X(dutyx,0);
    }
    if (OutputX_P < 0)
    {
        OutputX_P =  -OutputX_P;
        dutyx=OutputX_P;
        if(OutputX_P>=90){
            dutyx = 80;
        }
        if(OutputX_P<=9){
            dutyx=0;
        }
        drive_X(dutyx,1);
    }
    if (OutputY_P >=0)
    {
        dutyy = OutputY_P;
        if(OutputY_P>=90){
            dutyy=80;
        }
        if(OutputY_P<=6){
            dutyy=0;
        }
        drive_Y(dutyy,1);
    }
    if (OutputY_P < 0)
    {
        OutputY_P =  -OutputY_P;
        dutyy = OutputY_P;
        if(OutputY_P>=90){
            dutyy = 80;
        }
        if(OutputY_P<=6){
            dutyy=0;
        }
        drive_Y(dutyy,0);
    }
    g=dutyx;
    h=dutyy;

    
    _T1IF = 0;
}




void __attribute__((interrupt, no_auto_psv)) _T3Interrupt(void)
{
    printf("POS=%u | Ep=%.2f | Output=%.2f | duty=%d || POS=%u | Ep=%.2f | Output=%.2f | duty=%d\n",a,b,c,g,d,e,f,h);
    _T3IF = 0;
}
void __attribute__((interrupt, no_auto_psv)) _T4Interrupt(void)
{
    millis++;
    _T4IF = 0;
}

void INITPLL()
{
    PLLFBD = 150;           // M  = 152
    CLKDIVbits.PLLPRE = 5;  // N1 = 7
    CLKDIVbits.PLLPOST = 0; // N2 = 2
    OSCTUN = 0;             // Tune FRC oscillator, if FRC is used

    __builtin_write_OSCCONH(0x01); // Initiate Clock Switch to FRCPLL
    __builtin_write_OSCCONL(0x01); // Start clock switching

    while (OSCCONbits.COSC != 0b001)
        ; // Wait for Clock switch to occur
    while (OSCCONbits.LOCK != 1)
    {
    }; // Wait for PLL to lock
}

void INIT_UART1()
{
    __builtin_write_OSCCONL(OSCCON & 0xBF); //PPS RECONFIG UNLOCK
    RPINR18bits.U1RXR = 6;
    RPOR2bits.RP5R = 0b00011;
    __builtin_write_OSCCONL(OSCCON | 0x40); //PPS RECONFIG LOCK

    U1MODEbits.STSEL = 0;   // 1 Stop bit
    U1MODEbits.PDSEL = 0;   // No Parity, 8 data bits
    U1MODEbits.BRGH = 1;    // High Speed mode
    U1MODEbits.URXINV = 0;  // UxRX idle state is '1'
    U1BRG = 86;             // BAUD Rate Setting for 115200
    U1STAbits.UTXISEL0 = 0; // Interrupt after one TX Character is transmitted
    U1STAbits.UTXISEL1 = 0;
    IEC0bits.U1RXIE = 1;   // Enable UART RX Interrupt
    IEC0bits.U1TXIE = 1;   // Enable UART TX Interrupt
    U1MODEbits.UARTEN = 1; // Enable UART
    U1STAbits.UTXEN = 1;   // Enable UART TX
}

void INIT_QEI()
{
    __builtin_write_OSCCONL(OSCCON & 0xBF); //PPS RECONFIG UNLOCK
    RPINR18bits.U1RXR = 6;
    RPOR2bits.RP5R = 0b00011;
    RPINR14bits.QEA1R = 8;                  //remap RP8 connect to QEI1_A
    RPINR14bits.QEB1R = 9;                  //remap RP9 connect to QEI1_B
    RPINR16bits.QEA2R = 10;                 //remap RP10 connect to QEI2_A
    RPINR16bits.QEB2R = 11;                 //remap RP11 connect to QEI2_B
    __builtin_write_OSCCONL(OSCCON | 0x40); //PPS RECONFIG LOCK

    /*QEI Mode Select*/
    QEI1CONbits.QEIM = 0b000; // QEI Mode disable
    QEI1CONbits.PCDOUT = 0;   // no direction pin out
    QEI1CONbits.QEIM = 5;     // 2x ,no index
    /*digital filter config */
    DFLT1CONbits.QECK = 0b000; // clock divider Fcy/1
    DFLT1CONbits.QEOUT = 1;    // enable filter

    /*QEI Mode Select*/
    QEI2CONbits.QEIM = 0b000; // QEI Mode disable
    QEI2CONbits.PCDOUT = 0;   // no direction pin out
    QEI2CONbits.QEIM = 5;     // 2x ,no index
    /*digital filter config */
    DFLT2CONbits.QECK = 0b000; // clock divider Fcy/1
    DFLT2CONbits.QEOUT = 1;    // enable filter
}

void INIT_MOTOR()
{
    T1CONbits.TCKPS = 0b01;
    PR1 = 10000;
    _T1IE = 1; // enable Timer1 interrupt
    _T1IP = 7; // set priority 
    
    T2CONbits.TCKPS = 0b01; //set timer prescaler to 1:64
    PR2 = 8561;             //set period to 15,625 tick per cycle
    OC1RS = 0;
    OC2RS = 0;
    
    T3CONbits.TCKPS = 0b10;
    PR3 = 12500;
    _T3IE = 1; // enable Timer3 interrupt
    _T3IP = 1; // set priority 
    
    T4CONbits.TCKPS = 0b01;
    PR4 = 5000;
    _T4IE = 1; // enable Timer4 interrupt
    _T4IP = 4; // set priority to 5
    
    OC1CONbits.OCM = 0b000; //Disable Output Compare Module
    OC1CONbits.OCTSEL = 0;  //OC1 use timer2 as counter source
    OC1CONbits.OCM = 0b110; //set to pwm without fault pin mode
    OC2CONbits.OCM = 0b000; //Disable Output Compare Module
    OC2CONbits.OCTSEL = 0;  //OC1 use timer2 as counter source
    OC2CONbits.OCM = 0b110; //set to pwm without fault pin mode

    __builtin_write_OSCCONL(OSCCON & 0xBF); //PPS RECONFIG UNLOCK
    _RP14R = 0b10010;                       //remap RP14 connect to OC1
    _RP15R = 0b10011;
    __builtin_write_OSCCONL(OSCCON | 0x40); //PPS RECONFIG LOCK

    AD1PCFGL = 0xFFFF; //set analog input to digital pin
    //Output
    _TRISB14 = 0;//PWM1
    _TRISB3 = 0;//DIR1
    _TRISB15 = 0;//PWM2
    _TRISB4 = 0;//DIR2
    //Input
    _TRISB12 = 1;//Limit_X
    _TRISB13 = 1;//Limit_Y
    _TRISB8 = 1; // AX
    _TRISB9 = 1; // BX
    _TRISB10 = 1; // AY
    _TRISB11 = 1; // BY
}


void drive_X(int duty, int DIRX)
{
    int PWM = map(duty, 0, 100, 0, PR2);
    OC1RS = PWM;
    _LATB3 = DIRX;
}

void drive_Y(int duty, int DIRY)
{
    int PWM = map(duty, 0, 100, 0, PR2);
    OC2RS = PWM;
    _LATB4 = DIRY;
}

void set_home()
{
    while (_RB12 == 1) //Pull up
    {
        drive_X(50, 1);
    }
    drive_X(0, 1);

    while (_RB13 == 1) //Pull up
    {
        drive_Y(50, 0);
    }
    drive_Y(0, 0);
}

void delay_UART()
{
    unsigned int i;
    for (i = 0; i < 500; i++)
    { //347
        Nop();
    }
}
void waitmillis(int time)
{
    millis=0;
    T4CONbits.TON=1;
    while(millis<time){

    }
    T4CONbits.TON=0;
}

int main(void)
{
    /*disable global interrupt*/
    __builtin_disable_interrupts();

    INITPLL();
    INIT_QEI();
    INIT_MOTOR();
    INIT_UART1();
    delay_UART();

    /*enable global interrupt*/
    __builtin_enable_interrupts();
    T2CONbits.TON = 1;

    printf("Program Start\n");
    while (1)
    {

        if (uartData != 0)
        {
            send_package(uartData);
            uartData = 0;
        }
        if (Command_Sethome == 1)
        {
            Command_Sethome = 0;
            set_home();
            waitmillis(1000);
            POS1CNT = 0;
            POS2CNT = 0;
            printf("x=%u , y=%u\n",POS1CNT,POS2CNT);
        }
        if (Command_Write_POSX == 1)
        {
            Command_Write_POSX = 0;
            if (pulseX != POS1CNT)
            {
                printf("pulseX = %d\n", POS1CNT);
                pulseX = POS1CNT;
            }
            
        }
        if (Command_Write_POSY == 1)
        {
            Command_Write_POSY = 0;
            if (pulseY != POS2CNT)
            {
                printf("pulseY = %d\n", POS2CNT);
                pulseY = POS2CNT;
            }
        }

    }
    return 0;
}
