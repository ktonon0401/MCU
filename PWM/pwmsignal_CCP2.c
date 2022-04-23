// CONFIG1
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator: High-speed crystal/resonator on RA6/OSC2/CLKOUT and RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = ON       // RE3/MCLR pin function select bit (RE3/MCLR pin function is MCLR)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)
// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

#define _XTAL_FREQ 20000000
#include <xc.h>
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include "lcd.h"
#include <PIC16F887.h>

#define TMR2PRESCALE 4

int adc_value;
long PWM_freq=5000;
unsigned int pul=20;
char s[20];

void PWM_Initialize();
void PWM_Duty(unsigned int duty);
void ADC_Initialize();
unsigned int ADC_Read(unsigned char channel);

void main(void){
    ANSELbits.ANS0 = 1;
    TRISAbits.TRISA0 = 1;
    TRISCbits.TRISC1 = 0;
//    LCD_Clear();
    ADC_Initialize(); //Initialize ADC Module
    PWM_Initialize(); // This sets the PWM frequency of PWM1
    do{
        adc_value = ADC_Read(4); //Reading Analog Channel 0
        PWM_Duty(adc_value);
        __delay_ms(50);
        
//        LCD_Init();
//        sprintf(s,"FREQ: %d",adc_value);
//        LCD_Set_Cursor(1,1);
//        LCD_Write_String(s);
//        
//        sprintf(s,"     HCMUTE\0");
//        LCD_Set_Cursor(2,1);
//        LCD_Write_String(s);
    }while(1);
}

void PWM_Initialize(){
    PR2 = (_XTAL_FREQ/(PWM_freq*4*TMR2PRESCALE)) - 1;
//    PR2 = 0xFF;
    CCP2CONbits.CCP2M3 = CCP2CONbits.CCP2M2 = 1;
    T2CONbits.T2CKPS1 = 0; T2CONbits.T2CKPS0 = 1; T2CONbits.TMR2ON = 1; // Configure the Timer module
    TRISCbits.TRISC2 = 0;
}

void PWM_Duty(unsigned int duty){
    if(duty<1023){
        duty = ((float)duty/1023)*(_XTAL_FREQ/(PWM_freq*TMR2PRESCALE)); 
        CCP2CONbits.CCP2X = duty&1; // Store the 1st bit DC1B1
        CCP2CONbits.CCP2Y = duty&2; // Store the 0nd bit DC1B0
        CCPR2L = duty>>2; // Store the remaining 8 bit
    }
}

void ADC_Initialize(){
    ADCON0 = 0b01000011; //ADC ON and Fosc/8 is selected
    ADCON1 = 0b10110000; // Internal reference voltage is selected
}

unsigned int ADC_Read(unsigned char channel){
    ADCON0 &= 0x11000101; //Clearing the Channel Selection Bits
    ADCON0 |= channel<<3; //Setting the required Bits
    __delay_ms(2); //Acquisition time to charge hold capacitor
    ADCON0bits.GO_nDONE = 1; //Initializes A/D Conversion
    while(ADCON0bits.GO_nDONE); //Wait for A/D Conversion to complete
    return ((ADRESH<<8)+ADRESL); //Returns Result
}
