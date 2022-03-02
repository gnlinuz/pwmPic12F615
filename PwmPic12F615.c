/*
 * File:   main.c
 * Author: George.Nikolaidis
 *
 * Created on February 26, 2022, 5:47 PM
 */


// PIC12F615 Configuration Bit Settings

// 'C' source line config statements

// CONFIG
#pragma config FOSC = INTOSCIO  // Oscillator Selection bits (INTOSCIO oscillator: I/O function on GP4/OSC2/CLKOUT pin, I/O function on GP5/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // MCLR Pin Function Select bit (MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config IOSCFS = 8MHZ    // Internal Oscillator Frequency Select (8 MHz)
#pragma config BOREN = OFF      // Brown-out Reset Selection bits (BOR disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#define _XTAL_FREQ 8000000
#define DISABLE_PWM_Service()               (CCP1CON = 0x0)
#define ENABLE_DIGITAL_IO_Pins()            (ANSEL = 0X0)
#define GPIF_INT_InterruptFlagClear()       (INTCONbits.GPIF = 0)
#define TMR2_OFF()                          (T2CONbits.T2ON = 0)
#define TMR2_ON()                           (T2CONbits.T2ON = 1)
#define SET_PRESCALER_1()                   (T2CON = 0x0)
#define SET_PRESCALER_4()                   (T2CON = 0x1)
#define SET_PRESCALER_16()                  (T2CON = 0x2)
#define ENABLE_CCP1_Output_Drive()          (TRISAbits.TRISIO2 = 0)
#define DISABLE_CCP1_Output_Drive()         (TRISAbits.TRISIO2 = 1)
#define LED_ON()                            (GPIObits.GP5 = 1)
#define LED_OFF()                           (GPIObits.GP5 = 0)
#define SEND_HIGH_CLOCK_PULSE()             (GPIObits.GP2 = 1)
#define SEND_LOW_CLOCK_PULSE()              (GPIObits.GP2 = 0)
#define LED_Toggle()             do { GPIObits.GP5 = ~GPIObits.GP5; } while(0)
#define DEBOUNCE_Button()        do { for(int i=0;i<20000; i++); } while(0)
#define WAIT_FOR_NEW_PWM_CYCLE() do { while(PIR1bits.TMR2IF != 1); } while(0)

unsigned char lastPwmState = 1;
unsigned char lastManualPulse = 1;
unsigned char pwmSelect = 0;
unsigned char pwmFreq[36] = {0xF9,0x2,0xC,0x7D,
                            0x7C,0x2,0x2C,0x3E,
                            0x1E,0x2,0x2C,0xF,
                            0xC7,0x0,0xC,0x64,
                            0x63,0x0,0xC,0x32,
                            0x9,0x1,0xC,0x5,
                            0x13,0x0,0xC,0xA,
                            0x9,0x0,0xC,0x5,
                            0x3,0x0,0xC,0x2};

/*
 500Hz PR2=0xF9; TMR2=0x2; CCP1CON<DC1B1:DC1B0>=00 CCPR1L=01111101 0x7D;
1000Hz PR2=0x7C; TMR2=0x2; CCP1CON<DC1B1:DC1B0>=10 CCPR1L=00111110 0x3E
4.0323Khz PR2=0x1E; TMR2=0x2; CCP1CON<DC1B1:DC1B0>=10 CCPR1L=00001111 0xF;
10.4167Khz PR2=0xC7; TMR2=0x0; CCP1CON<DC1B1:DC1B0>=00 CCPR1L=01100100 0x64;
20Khz PR2=0x63; TMR2=0x0; CCP1CON<DC1B1:DC1B0>=00 CCPR1L=00110010 0x32;
50Khz PR2=0x9; TMR2=0x1; CCP1CON<DC1B1:DC1B0>=00 CCPR1L=00000101 0x5;
100Khz PR2=0x13; TMR2=0x0; CCP1CON<DC1B1:DC1B0>=00 CCPR1L=00001010 0xA;
200Khz PR2=0x9; TMR2=0x0; CCP1CON<DC1B1:DC1B0>=00 CCPR1L=00000101 0x5;
500Khz PR2=0x3; TMR2=0x0; CCP1CON<DC1B1:DC1B0>=00 CCPR1L=00000010 0x2;
 */

void SYSTEM_Initialize(){
    /* CONFIGURATION OF GPIO
     * GP2 - PWM OUTPUT P1A - PIN 5
     * GP1 - PULL UP IOC BUTTON - PIN 6
     * GP4 - PULL UP IOC BUTTON - PIN 3
     * GP5 - OUTPUT LED - PIN 2 */

    ENABLE_DIGITAL_IO_Pins();/* disable analog enable digital pins */
    TRISA = 0x1E;        /* 00011110 GP5,GP0 as OUTPUT, GP1,GP2,GP3,GP4 as INPUT
                          initially you need to disable output for PWM on GP2
                          as per documentation */
    GPIObits.GP0 = 0x0;  /*  make GP0 output low */
    GPIObits.GP5 = 0x0;  /*  make GP5 output low */

    OPTION_REG   = 0x0;  /* 00000000, 0x0
                          * GPIO pull-ups enable 0,
                          * INTEDG on rising     0,
                          * TOSC FOSC/4 TOSE     0,
                          * PSA                  0,
                          * PS                   000 1:2 */
    WPU = 0x12;          /* 00010010 0x12
                          * Weak pull-up enable on pins GP1,GP4 2 buttons */
    IOC = 0x12;          /* Interrupt on change enable IOC on GP1, GP4 */
    INTCON = 0xC8;       /* 11001000  0xC8
                          * GIE  1 Global Interrupt Enable bit
                          * PEIE 1 Enables all unmasked interrupts
                          * TOIE 0 Timer0 Overflow Interrupt Enable bit
                          * INTE 1 GP2/INT External Interrupt Enable bit
                          * GPIE 1 GPIO Change Interrupt Enable bit, IOC must EN
                          * TOIF 0 Timer0 Overflow Interrupt Flag bit(2)
                          * INTF 0 GP2/INT External Interrupt Flag bit
                          * GPIF 0 GPIO Change Interrupt Flag bit */
    LED_ON();            /* Led ON */
    DISABLE_CCP1_Output_Drive();
    PR2     = 0xF9;      /* Load value 249 decimal for PWM freq 500Hz 0xF9 */
    CCP1CON = 0xC;       /* 00001100 load duty cycle 50% */
    CCPR1L  = 0x7D;      /* 01111101 load value on CCPR1L for DC 50% 0x7D */
    PIR1    = 0x0;       /* Clear interrupt flag TMR2IF, TMR2 to PR2 Match
                          * Interrupt Flag bit(1) */
    SET_PRESCALER_16();   /* Set prescaler to 1:16 */
    TMR2_ON();           /* TMR2 ON */
    WAIT_FOR_NEW_PWM_CYCLE();   /* as per documentation you have to wait after
                                 * a new PWM cycle has started, until Timer2
                                 * overflows and TMR2IF bit is set  */
    ENABLE_CCP1_Output_Drive(); /* Enable the CCP1 pin output driver by clearing
                                 * GP2 P1A bit and make it as output */
}

void selectPwmFreq(unsigned char pos){
    PR2 = pwmFreq[pos];
    T2CON = pwmFreq[pos+1];
    CCP1CON = pwmFreq[pos+2];
    CCPR1L = pwmFreq[pos+3];
}

void pwmInitialise(unsigned char pSel) {
    DISABLE_CCP1_Output_Drive();
    selectPwmFreq(pSel);
    PIR1 = 0x0;                 /* Clear interrupt flag TMR2IF */
    TMR2_ON();                  /* TMR2 ON */
    WAIT_FOR_NEW_PWM_CYCLE();   /* as per documentation you have to wait after
                                 * a new PWM cycle has started, until Timer2
                                 * overflows and TMR2IF bit is set  */
    ENABLE_CCP1_Output_Drive(); /* Enable the CCP1 pin output driver by clearing
                                 * GP2 P1A bit and make it as output */
}

void stopPwm(){
    DISABLE_PWM_Service();      /* Stop PWM service */
    TMR2_OFF();                 /* Stop TMR2  clock */
    SEND_LOW_CLOCK_PULSE();
}

void sendManualPulse(){
    SEND_HIGH_CLOCK_PULSE();
    LED_ON();
    __delay_ms(10);
    SEND_LOW_CLOCK_PULSE();
    LED_OFF();
}

void __interrupt() ISR (void){
    if(INTCONbits.GPIF == 1 && GPIObits.GP1 == 0){
        DEBOUNCE_Button();
        if(lastPwmState == 1){
            LED_OFF();
            stopPwm();
            lastPwmState = 0;
        }
        else
       {
            LED_ON();
            pwmInitialise(pwmSelect);
            lastPwmState = 1;
        }
        GPIF_INT_InterruptFlagClear();
    }
    if(INTCONbits.GPIF == 1 && GPIObits.GP4 == 0){
        DEBOUNCE_Button();
        if(lastPwmState == 0){
            sendManualPulse();
        }
        if(lastPwmState == 1){
            stopPwm();
            pwmSelect = pwmSelect + 4;
            if(pwmSelect <= 32){
                pwmInitialise(pwmSelect);
            }
            else
            {
                pwmSelect = 0;
                pwmInitialise(pwmSelect);
            }
        }
        GPIF_INT_InterruptFlagClear();
    }
}

void main(void) {
    SYSTEM_Initialize();
    while(1);
}
