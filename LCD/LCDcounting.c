// CONFIG1
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator: High-speed crystal/resonator on RA6/OSC2/CLKOUT and RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = ON       // RE3/MCLR pin function select bit (RE3/MCLR pin function is MCLR)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = ON      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)
// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

#define _XTAL_FREQ 20000000
#include <xc.h>
#include <pic16f887.h>
#include <stdio.h>
#include <math.h>
#include "lcd.h"


int a;
char s[20];

void main(void) {
    ANSEL = ANSELH = 0;
    LCD_Clear();
    TRISBbits.TRISB0 = 1;
    IOCBbits.IOCB0=1;
    INTCONbits.RBIE=1;
    INTCONbits.RBIF=0;
    INTCONbits.GIE=1;
//    OPTION_REGbits.INTEDG = 1;
//    OPTION_REGbits.nRBPU = 0; 
    while(1){
//        if(!PORTBbits.RB0){a++;}
        LCD_Init();
        sprintf(s,"COUNT: ");
        LCD_Set_Cursor(1,1);  
        LCD_Write_String(s);
        sprintf(s,"      %d",a);
        LCD_Set_Cursor(2,1);
        LCD_Write_String(s);
    }
}

void __interrupt() isr(void){
    if(INTCONbits.RBIE&&INTCONbits.RBIF){
        if(!PORTBbits.RB0){a++;}
        INTCONbits.RBIF=0;  
    }
}