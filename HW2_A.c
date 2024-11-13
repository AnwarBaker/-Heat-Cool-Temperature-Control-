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
#include "my_ser.h"


#define _XTAL_FREQ 4000000 
#define WINTER_T 40
#define SUMMER_T 60

void setup_compare(void);
void Int0ISR(void);
void Int2ISR(void);
void dec(void);
void setupInterpurts(void);
void initPorts(void);
void compare_isr(void);
void tmr_isr(void);
void OffMode(void);
void CoolMode(void);
void heatMode(void);
void autoCool(void);
void autoHeat(void);
void printingOnScreen(void);
void setupAllThings(void);
void SerialHandler(void);

typedef enum { OFF, COOL, HEAT, AUTO_COOL_HEAT, AUTO_HEAT } Mode;
Mode current_mode= OFF;
int mode_counter=-1;
int Hs = -1;
char Buffer[32];
float AnalogInput0,AnalogInput1, AnalogInput2;
int raw_value;
float AI0;
float AI1=60;
float AI2;
float AI3;
float OT;
volatile int SerialFlag = 0;
float TT;  // Room temperature
float SSP;

void __interrupt(high_priority) highIsr(void) {
    if (INTCON3bits.INT2IF) {
        
        Int2ISR();
       
    } else if (INTCONbits.INT0IF) {
        Int0ISR();
    }else if (PIR2bits.CCP2IF) {
        compare_isr();
    }
    else if(PIR2bits.TMR3IF){
        tmr_isr();
    }
    else if(INTCON3bits.INT1IF){
         dec();
    }
    
}


void Int0ISR(void)
{
              __delay_ms(250);
        INTCONbits.INT0IF = 0;
         
        mode_counter++;
        if(mode_counter==0)
            current_mode=COOL;
        else if(mode_counter==1)
            current_mode=HEAT;
        else if(mode_counter==2)
            {
            current_mode=AUTO_COOL_HEAT;
            mode_counter=-1;
        }
        
        
       

}
void Int2ISR(void){
    __delay_ms(250);
    INTCON3bits.INT2IF = 0;
    if(current_mode==AUTO_COOL_HEAT){
    
    Hs++;
    if(Hs == 4){
        Hs = 3;
    }
    }
    else if(current_mode==COOL || current_mode==HEAT){
        AI1=AI1+5;
    }
}
void dec(void){
    __delay_ms(250);
    INTCON3bits.INT1IF =0;
    if(current_mode==AUTO_COOL_HEAT){
    
    Hs--;
    if(Hs <0){
        Hs = 0;
    }
    }
    else if(current_mode==COOL || current_mode==HEAT){
        AI1=AI1-5;
    }
}
void setupInterpurts(void) {
    INTCON2 = 0b01010001; 
    INTCON3 = 0b00011000;  
    INTCON = 0b11010000;  
    RCON = 0b00000000;    
    PIE2 = 0b00001001;     
    PIR2 = 0b00000000;     
    IPR2 = 0b00000000;

}
void setup_compare(void){
    CCP2CON = 9;
    T3CON = 0x00; 
    T3CONbits.TMR3CS = 0; 
    T3CONbits.T3CKPS = 0b10; 
    T3CONbits.T3CCP2 = 0;
    T3CONbits.T3CCP1 = 1;
    TMR3 = 0; 
    T3CONbits.TMR3ON = 1; 
}
void initPorts(void) {
    ADCON0 = 0;
    ADCON1 = 0b00001100; 
    LATA = LATB = LATC = LATD = LATE = 0;
     TRISA = 0xFF; 
    TRISB = 0xFF;
    TRISC = 0x80; 
    TRISD = 0; 
    TRISE = 0;   
}

void setupAllThings(void){
     initPorts();
    setupInterpurts();
    lcd_init();
    init_adc_no_lib();
    setup_compare();
}
void compare_isr(void){
    PIR2bits.CCP2IF = 0;
    PORTCbits.RC5=0;
    PORTDbits.RD2 ^= 1;
}
void tmr_isr(void){
    PIR2bits.TMR3IF = 0;
    PORTCbits.RC5=1;
    PORTDbits.RD1 ^= 1;
      
    TMR3 = 0;
}


void OffMode(void)
{
    AI3 = read_adc_raw_no_lib(1);
     AI0 = read_adc_raw_no_lib(0);
    lcd_gotoxy(1, 2);
    sprintf(Buffer, "SP: %4.1fC    Y Y",(AI0*5.0*100.0)/1023.0/5.0);
    lcd_puts(Buffer);
    lcd_gotoxy(1, 4);
    sprintf(Buffer,"MD:OFF          ");
    lcd_puts(Buffer);
    lcd_gotoxy(1, 3);
    sprintf(Buffer, "OT: %4.1fC R: 0.0", (AI3*5.0*100.0)/1023.0/5.0);
    lcd_puts(Buffer);

    lcd_puts(Buffer);
    PORTCbits.RC2 =0; 
    PORTCbits.RC5=0;
    PIE2 = PIE1 = 0;
}

void CoolMode(void)
{
    AI3 = read_adc_raw_no_lib(1);
     AI0 = read_adc_raw_no_lib(0);
    lcd_gotoxy(1, 2);
    sprintf(Buffer, "SP: %4.1fC    Y N",(AI0*5.0*100.0)/1023.0/5.0);
    lcd_puts(Buffer);
    lcd_gotoxy(1, 4);
    sprintf(Buffer,"MD:Cool         ");
    lcd_puts(Buffer);
    lcd_gotoxy(1, 3);
    sprintf(Buffer, "OT: %4.1fC C: %2.1f%%", (AI3*5.0*100.0)/1023.0/5.0,AI1);
    lcd_puts(Buffer);
    init_pwm1();
    raw_value = read_adc_raw_no_lib(1);
    set_pwm1_raw(raw_value);
    PORTCbits.RC5=0; // off the heater !
    
}

void heatMode(void){
    AI3 = read_adc_raw_no_lib(1);
     AI0 = read_adc_raw_no_lib(0);
    lcd_gotoxy(1, 2);
    sprintf(Buffer, "SP: %4.1fC    N Y",(AI0*5.0*100.0)/1023.0/5.0);
    lcd_puts(Buffer);
    lcd_gotoxy(1, 4);
    sprintf(Buffer,"MD:Heat         ");
    lcd_puts(Buffer);
    lcd_gotoxy(1, 3);
    sprintf(Buffer, "OT: %4.1fC H: %2.1f%%", (AI3*5.0*100.0)/1023.0/5.0,AI1);
    lcd_puts(Buffer);
    
   PIE2bits.TMR3IE = 1; 
    PIE2bits.CCP2IE = 1; 
    
     
       
        PIE2bits.TMR3IE = 1; 
         PIE2bits.CCP2IE = 1; 
        int compare_value = (int) (100 * 65535 / 100.0);
        CCPR2H = (compare_value >>8) & 0x00FF;
        CCPR2L = compare_value & 0x00FF;
}
void autoCool(void){
    AI3 = read_adc_raw_no_lib(1);
     AI0 = read_adc_raw_no_lib(0);
    lcd_gotoxy(1, 2);
    sprintf(Buffer, "SP: %4.1fC    N N",(AI0*5.0*100.0)/1023.0/5.0);
    lcd_puts(Buffer);
    lcd_gotoxy(1, 4);
    sprintf(Buffer,"MD:Auto HC HS: %d",Hs);
    lcd_puts(Buffer);
    lcd_gotoxy(1, 3);
    sprintf(Buffer, "OT: %4.1fC R: 0.0", (AI3*5.0*100.0)/1023.0/5.0);
    lcd_puts(Buffer);
    init_pwm1();
    
    float SP =(AI0*5.0/1023.0)*100/5.0;
    float T=(AI2*5/1023.0)*100;
    
    PIE2bits.TMR3IE = 0; 
    PIE2bits.CCP2IE = 0; 
    
    float coolError = T - SP;
    
    if(coolError > 0){
        float percent_value = coolError * 10;
        if(percent_value < 25.0) percent_value = 25;
        if(percent_value > 100.0) percent_value = 100;
        
        set_pwm1_percent(percent_value);
        PORTCbits.RC5=0;
        PIE2bits.TMR3IE = 0; 
        PIE2bits.CCP2IE = 0; 
    }

    if (T < (SP - Hs)) {
        set_pwm1_percent(0.0);
        PIE2bits.TMR3IE = 1; 
        PIE2bits.CCP2IE = 1; 
        
        raw_value = 512 * 64; 
        CCPR2H = (raw_value >>8) & 0x00FF;
        CCPR2L = raw_value & 0x00FF;
    }
    float SP =(AI0*5.0/1023.0)*100/5.0;
    float T=(AI2*5/1023.0)*100;
    PORTCbits.RC2 =0;
    CCP1CON = 0x00;
    
    PIE2bits.TMR3IE = 1; 
    PIE2bits.CCP2IE = 1; 
    
    float heatError = SP - T;
    
    if(heatError > 0){
        float percent_heat = heatError * 10;
        if(percent_heat < 50.0) percent_heat = 50.0;
        if(percent_heat > 100.0) percent_heat = 100.0;
        PIE2bits.TMR3IE = 1; 
         PIE2bits.CCP2IE = 1; 
        int compare_value = (int) (percent_heat * 65535 / 100.0);
        CCPR2H = (compare_value >>8) & 0x00FF;
        CCPR2L = compare_value & 0x00FF;
  
        
    }
    if(T > (SP+Hs)){
        PORTCbits.RC5=0;
        PIE2bits.TMR3IE = 0; 
        PIE2bits.CCP2IE = 0; 
    }
    
    
    
}
void autoHeat(void){
    lcd_gotoxy(1, 4);
    sprintf(Buffer,"MD: Auto Heat      ");
    lcd_puts(Buffer);
    
    float SP =(AI0*5.0/1023.0)*100/5.0;
    float T=(AI2*5/1023.0)*100;
    PORTCbits.RC2 =0;
    CCP1CON = 0x00;
    
    PIE2bits.TMR3IE = 1; 
    PIE2bits.CCP2IE = 1; 
    
    float heatError = SP - T;
    
    if(heatError > 0){
        float percent_heat = heatError * 10;
        if(percent_heat < 50.0) percent_heat = 50.0;
        if(percent_heat > 100.0) percent_heat = 100.0;
        PIE2bits.TMR3IE = 1; 
         PIE2bits.CCP2IE = 1; 
        int compare_value = (int) (percent_heat * 65535 / 100.0);
        CCPR2H = (compare_value >>8) & 0x00FF;
        CCPR2L = compare_value & 0x00FF;
  
        
    }
    if(T > (SP+Hs)){
        PORTCbits.RC5=0;
        PIE2bits.TMR3IE = 0; 
        PIE2bits.CCP2IE = 0; 
    }
    
    
}
void printingOnScreen(void){
    
   
    
    AI2 = read_adc_raw_no_lib(2);

    lcd_gotoxy(1, 1);
    sprintf(Buffer, "RT: %4.1fC    H C",(AI2*100.0*5.0)/1023.0);
    lcd_puts(Buffer);

    

    

        
}
void main(void) {
    
   setupSerial();
    setupAllThings();
    current_mode=OFF;
    while(1)
    {
        AI3 = read_adc_raw_no_lib(1);
        OT=(AI3*5.0*100.0)/1023.0/5.0;
        //////////////////////////////////////////
        AnalogInput1 = read_adc_voltage(1);
        AnalogInput2 = read_adc_voltage(2);
        AnalogInput0 = read_adc_voltage(0);
        
        
          if (INTCONbits.INT0IF) Int0ISR();
    if (PIR1bits.RCIF) SerialFlag = 1;
        
        
        TT = AnalogInput2 * 100;
        SSP = AnalogInput0 * 100 / 5;
        
        /////////////////////////////////////
        if(OT>SUMMER_T || OT<WINTER_T){
            current_mode = AUTO_COOL_HEAT;
        }
       
        if(RB3==0){
            
            current_mode = OFF;
        }
        CLRWDT();
        
        printingOnScreen();
       
        if (current_mode == OFF) {
                OffMode();
        } 
        else if (current_mode == COOL) {
                CoolMode();
        } 
        else if (current_mode == HEAT) {
            heatMode();
        } 
        else if (current_mode == AUTO_COOL_HEAT) {
            autoCool();
        } 
        
        if (SerialFlag) {
            SerialHandler();
            SerialFlag = 0;
        }
    }
    return;
}


void SerialHandler(void) {
    unsigned char received = read_byte_no_lib();
    if (received == 'M') {
        sprintf(Buffer, "\nRT:%4.1f OT:%4.1f SP:%4.1f", TT, AnalogInput1 * 100 / 5, SSP);
        send_string_no_lib(Buffer);
    } else if (received == 'S') {
        current_mode = OFF;
    }
}
