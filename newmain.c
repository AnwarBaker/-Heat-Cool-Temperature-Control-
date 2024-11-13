/*
 * File:   Home2.c
 * Author: HadiI
 *
 * Created on June 23, 2024, 10:35 AM
 */



// PIC18F4620 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1H
#pragma config OSC = XT         // Oscillator Selection bits (XT oscillator)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = ON         // Watchdog Timer Enable bit (WDT enabled)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdio.h>
#include "my_adc.h"
#include "my_pwm.h"
#include "lcd_x8.h"

#define _XTAL_FREQ 4000000 

typedef enum { OFF, COOL, HEAT, AUTO_COOL } Mode;
Mode CurrentMode= OFF;
int InterruptModeChange=-1;
int Hs = -1;
char Buffer[32];
short Compare_Value;
int Raw;
float AnalogInput0;
float AnalogInput1;
float AnalogInput2;
float T;
float SP;


void Heater_Off(void)
{
    PORTCbits.RC5=0;
}


void Heater_On(void)
{
    PORTCbits.RC5=1;
}

void Cooler_Off(void)
{
    PORTCbits.RC2=0;
}

void Cooler_On(void)
{
    PORTCbits.RC2=1;
}

void Compare_on(void) {
    CCP2CON = 0x09; // CCP2 in compare mode, clear output on match
    T3CON = 0x00; // Clear Timer 3 control register
    T3CONbits.TMR3CS = 0b00; // Timer 3 clock source is internal instruction cycle (Fosc/4)
    T3CONbits.T3CKPS = 0b10; // Pre-scaler is 1:4
    TMR3 = 0; // Clear Timer 3
    T3CONbits.TMR3ON = 1; // Turn on Timer 3
}

void Set_PWM(unsigned int raw_value) {
    if (raw_value > 1023) {
        raw_value = 1023; // Clamp the value to 10-bit max
    }
    CCPR1L = (raw_value >> 2) & 0x00FF; // Load the upper 8 bits in CCPR1L
    CCP1CONbits.DC1B = raw_value & 0x0003; // Load the lower 2 bits in CCP1CON
}

void AutoCoolMode(int T, int SP) {
    int CoolError = T - SP;
   
    if (CoolError > 0) {
        int pwm_value = (CoolError * 100) / 10;
        if (pwm_value < 255) {
            pwm_value = 255;  // Minimum PWM value should be 25%
        }
        
        Set_PWM(pwm_value);
        Heater_Off(); //turning heater off;
    } else if (T < (SP - Hs)) {
        Set_PWM(0);  // Cooler OFF
        
        Heater_On();
    }
}

void PWM_Disable(void) {
    CCP1CON = 0x00;        // Disable PWM
    T2CONbits.TMR2ON = 0;  // Turn off Timer2
    LATCbits.LATC2 = 0;    // Set RC2 pin to low to turn off the fan
}

void Int0ISR(void)
{
              __delay_ms(250);
        INTCONbits.INT0IF = 0; // Clear the interrupt flag
         
        InterruptModeChange++;
        if(InterruptModeChange==0)
            CurrentMode=OFF;
        else if(InterruptModeChange==1)
            CurrentMode=COOL;
        else if(InterruptModeChange==2)
            CurrentMode=HEAT;
        else if(InterruptModeChange==3)
        {
            CurrentMode=AUTO_COOL;
            InterruptModeChange=-1;
        }

}

void setupINT(void) {
    INTCON2bits.INTEDG2 = 1; // Interrupt on rising edge for INT2
    INTCON2bits.INTEDG0 = 1;  // Interrupt on rising edge for INT0
    INTCON3bits.INT2IE = 1;  // Enable INT2 interrupt
    INTCONbits.INT0IE = 1;   // Enable INT0 interrupt
    INTCON3bits.INT2IF = 0;  // Clear INT2 flag
    INTCONbits.INT0IF = 0;   // Clear INT0 flag
    INTCONbits.GIE = 1;      // Enable global interrupts
    INTCONbits.PEIE = 1;     // Enable peripheral interrupts
    RCONbits.IPEN = 0;       // Disable interrupt priority levels
    PIE2bits.TMR3IE = 1;     // Enable Timer 3 interrupt
    PIR2bits.TMR3IF = 0;     // Clear Timer 3 interrupt flag
    IPR2bits.TMR3IP = 0;     // Set Timer 3 interrupt to low priority
    PIE2bits.CCP2IE = 1;     // Enable CCP2 interrupt
    PIR2bits.CCP2IF = 0;     // Clear CCP2 interrupt flag
    IPR2bits.CCP2IP = 0;     // Set CCP2 interrupt to low priority
}

void setupPorts(void) {
    ADCON0 = 0;
    ADCON1 = 0b00001100; //3 analog channels, change this according to your application
    LATA = LATB = LATC = LATD = LATE = 0;
    TRISB = 0xFF; // all pushbuttons are inputs
    TRISC = 0x80; // RX input , others output
    TRISA = 0xFF; // All inputs
    TRISD = 0x00; // All outputs
    TRISE = 0x00; // All outputs
}

void __interrupt(high_priority) highIsr(void) {
    if (INTCON3bits.INT2IF) {
        __delay_ms(250);
        INTCON3bits.INT2IF = 0;
        Hs++;
        if (Hs == 5) {
            Hs = 0;
        }
    } else if (INTCONbits.INT0IF) {
        Int0ISR();
    } else if (PIR2bits.TMR3IF) {
        PIR2bits.TMR3IF = 0;
        Heater_On();
        Compare_Value = read_adc_raw_no_lib(1) * 64;

        CCPR2H = (Compare_Value >> 8) & 0x00FF;
        CCPR2L = Compare_Value & 0x00FF;
        TMR3 = 0;
    } else if (PIR2bits.CCP2IF) {
        PIR2bits.CCP2IF = 0;
        Heater_Off();
    }
}

void OffMode(void)
{
    lcd_gotoxy(1, 4);
    sprintf(Buffer,"Mode: OFF       ");
    lcd_puts(Buffer);
    PORTCbits.RC2 =0; //Cooler Off
    Heater_Off();
    PWM_Disable(); //Disable the PWM unit
    PIE2 = PIE1 =0; //Turning the CCPIE interrupt OFF
}

void CoolMode(void)
{
    lcd_gotoxy(1, 4);
    sprintf(Buffer,"Mode: Cool      ");
    lcd_puts(Buffer);
    init_pwm1();
    Raw = read_adc_raw_no_lib(1);
    set_pwm1_raw(Raw);
    Heater_On();
}

void Heat(void)
{
    lcd_gotoxy(1, 4);
    sprintf(Buffer,"Mode: Heat      ");
    lcd_puts(Buffer);
    
    Cooler_Off(); 
    PWM_Disable();
                
    Compare_Value = read_adc_raw_no_lib(1)*64; //setting the Compare value, the rest will be done in the interrupt.
    CCPR2H = (Compare_Value >>8) & 0x00FF;
    CCPR2L = Compare_Value & 0x00FF;
}

void AutoCool(void)
{
    lcd_gotoxy(1, 4);
    sprintf(Buffer,"Mode: Auto Cool");
    lcd_puts(Buffer);
    Cooler_Off(); 
    SP =AnalogInput0*100/5;
    T=AnalogInput2*100;
    TRISCbits.RC2 = 0;
    init_pwm1();
    AutoCoolMode(T,SP);
}

void main(void) {
    
//setting up everything
    setupPorts();
    setupINT();
    lcd_init();
    init_adc_no_lib();
    Compare_on();
    
  
    
    while(1)
    {
        CLRWDT();
        AnalogInput0=read_adc_voltage(0);
        AnalogInput1=read_adc_voltage(1);
        AnalogInput2=read_adc_voltage(2); //reading all analog input values since we use them all
        
        //SCREEN INITIALIZATION:
        lcd_gotoxy(1, 1);
        sprintf(Buffer,"RT: %4.1fC       ",AnalogInput2*100);
        lcd_puts(Buffer);
        
        lcd_gotoxy(1, 2);
        sprintf(Buffer,"SP: %4.1fC       ",AnalogInput0*100/5);
        lcd_puts(Buffer);
        
        lcd_gotoxy(1, 3);
        sprintf(Buffer,"HS: %d  HC: %2.1f%%",Hs,AnalogInput1*100/5);
        lcd_puts(Buffer);
        
        switch(CurrentMode) //mood switching based on the interrupts
        {
            case OFF: //main code of the OFF Mode
                 OffMode();
                break;
            case COOL: //main code for the COOL Mode
                CoolMode();
                break;
            case HEAT: //main code for the HEAT Mode
                Heat();
                break;
            case AUTO_COOL: //main code for the AUTO_COOL Mode
                AutoCool();
                break;      
        }
    }
    return;
}
