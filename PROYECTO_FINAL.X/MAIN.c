//******************************************************************************
//   UNIVERSIDAD DEL VALLE DE GUATEMALA
//   IE2023 PROGRAAMACIÓN DE MICROCONTROLADORES 
//   AUTOR: JORGE SILVA
//   COMPILADOR: XC8 (v1.41), MPLAB X IDE (v6.00)
//   PROYECTO: PROYECTO FINAL, BRAZO ANIMATRÓNICO
//   HARDWARE: PIC16F887
//   CREADO: 18/11/2022
//   ÚLTIMA MODIFCACIÓN: 18/11/2022
//******************************************************************************
// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT
#pragma config WDTE = OFF       
#pragma config PWRTE = ON      
#pragma config MCLRE = OFF      
#pragma config CP = OFF        
#pragma config CPD = OFF        
#pragma config BOREN = OFF      
#pragma config IESO = OFF       
#pragma config FCMEN = OFF     
#pragma config LVP = OFF       

// CONFIG2
#pragma config BOR4V = BOR40V   
#pragma config WRT = OFF
//******************************************************************************

#include <xc.h>
#include <stdint.h>

#define _XTAL_FREQ 500000
#define tmr0_value 240      //TMR0 a 2ms

//******************************************************************************
//FUNCIONES
//******************************************************************************

void setup(void);
void paso(int voltaje);
void setupADC(void);
void setupPWM(void);

//******************************************************************************
//VARIABLES
//******************************************************************************

int volt; 
int SERVO1;
int SERVO2; 
int LED;
uint8_t cont;
int VAL3;
int VAL4;

//******************************************************************************
// INTERRUPCIONES
//******************************************************************************

void __interrupt() isr (void){
     
    if (INTCONbits.T0IF){ //TMR0 PARA SEÑAL DE LOS OTROS 2 SERVOS
        cont++;
        if (cont <= VAL3){
            PORTCbits.RC3 = 1;
            __delay_ms(20);
        }
        
        else{
            PORTCbits.RC3 = 0; 
        }
        
        TMR0 = tmr0_value; 
        INTCONbits.T0IF = 0;
    }
}

//******************************************************************************
//CÓDIGO
//******************************************************************************

void main(void) {
    
    setup();
    setupADC();
    setupPWM();
    cont = 0;
    
    while(1){
        
        while(PORTBbits.RB0 == 1 && PORTBbits.RB1 == 0){ //MODO MANUAL
            
            //LEDS QUE INDICAN MODO
            PORTAbits.RA5 = 1;
            PORTAbits.RA4 = 0;
            PORTBbits.RB6 = 0;
            PORTBbits.RB7 = 0;
            
            //SERVO1
            ADCON0bits.CHS = 0b0001;
            __delay_us(100);
            ADCON0bits.GO = 1;
            while (ADCON0bits.GO == 1){
                ;
            }
            SERVO1 = ADRESH;
            paso(SERVO1);
            CCPR1L = volt; 
            __delay_us(100);
        
            //SERVO2
            ADCON0bits.CHS = 0b0010;  
            __delay_us(100);
            ADCON0bits.GO = 1; 
            while (ADCON0bits.GO == 1){
             ;
            }
            SERVO2 = ADRESH; 
            paso(SERVO2);
            CCPR2L = volt; 
            __delay_us(100);
        
            //SERVO3
            ADCON0bits.GO = 0b0011;
            __delay_us(100);
            ADCON0bits.GO = 1; 
            while (ADCON0bits.GO == 1){
              ;
            }
            VAL3 = ADRESH;  
            __delay_us(100);
        }
        
        while (PORTBbits.RB0 == 0 && PORTBbits.RB1 == 1){ //MODO EPROM
            PORTAbits.RA5 = 0;
            PORTAbits.RA4 = 1;
            PORTBbits.RB6 = 0;
            PORTBbits.RB7 = 0;
        }
        
        while (PORTBbits.RB0 == 1 && PORTBbits.RB1 == 1){ //MODO SERIAL
            PORTAbits.RA5 = 0;
            PORTAbits.RA4 = 0;
            PORTBbits.RB6 = 1;
            PORTBbits.RB7 = 0;
        }
        
        while (PORTBbits.RB0 == 0 && PORTBbits.RB1 == 0){ //ERROR
            PORTAbits.RA5 = 0;
            PORTAbits.RA4 = 0;
            PORTBbits.RB6 = 0;
            PORTBbits.RB7 = 1;
        }
        
  
    }
}

void setup(void){
    //CONF PUERTOS
    ANSEL = 0B00001111
    ANSELH = 0;
    
    TRISA = 0b00001111; 
    TRISC = 0b10000000;
    TRISB = 0b00000011;
    
    PORTA = 0; 
    PORTD = 0;
    PORTC = 0;
    PORTD = 0;
   
    //CONF OSCILADOR
    OSCCONbits.IRCF = 0b011;       // 500 KHz
    OSCCONbits.SCS = 1;
    
    //CONF INTERRUPCIONES
    INTCONbits.GIE = 1;            //GLOBALES
    
    PIE1bits.ADIE = 1;              //ADC
    PIR1bits.ADIF = 0;              
    
    INTCONbits.TMR0IE = 1;          //TMR0
    INTCONbits.T0IF = 0;
    OPTION_REGbits.T0CS = 0;     
    OPTION_REGbits.PSA = 0;     
    OPTION_REGbits.PS = 0b011;      //PS 1:16
    TMR0 = tmr0_value;              
    
}

void paso(int voltaje){ //CONVERSIÓN DE VOLTAJE
    volt = (unsigned short)(7+((float)(9)/(255))*(voltaje-0));
}

void setupADC(void){
    
    // Paso 1 Seleccionar puertoS de entrada
    TRISAbits.TRISA0 = 1;
    ANSELbits.ANS0 = 1;
    
    TRISAbits.TRISA1 = 1;
    ANSELbits.ANS1 = 1; 
    
    TRISAbits.TRISA2 = 1;
    ANSELbits.ANS2 = 1;
    
    TRISAbits.TRISA3 = 1;
    ANSELbits.ANS3 = 1;
  
    
    // Paso 2 Configurar módulo ADC
    
    ADCON0bits.ADCS1 = 0;
    ADCON0bits.ADCS0 = 1;       
    
    ADCON1bits.VCFG1 = 0;       
    ADCON1bits.VCFG0 = 0;       
    
    ADCON1bits.ADFM = 0;        //JUSTIFICADO IZQUIERDA
    
    
    ADCON0bits.CHS = 0b0010;
   
    
    ADCON0bits.ADON = 1;
    __delay_us(100);
    
}

void setupPWM(void){
    // Paso 1. Definir pines de donde saldrá la señal del PWM
    TRISCbits.TRISC1 = 1;
    TRISCbits.TRISC2 = 1;
    
    // Paso 2. Definimos periodo
    PR2 = 255;      // 20mS
    
    // Paso 3. Configuramos los bits de salida/entrada del PWM
    
    CCP1CONbits.P1M = 0b00;     
    
    CCP1CONbits.CCP1M = 0b1100;  
    
    CCP2CONbits.CCP2M = 0b1111;    
            
   // Paso 4. Ancho de pulso a 1.5ms
    CCP1CONbits.DC1B = 0b11;       
    CCPR1L = 11;        
    
    CCP2CONbits.DC2B1 = 0b1;       
    CCP2CONbits.DC2B0 = 0b1;
    CCPR1L = 11;   
                   
    // Paso 5. Configurar TMR2
    TMR2IF = 0;
    T2CONbits.T2CKPS = 0b11;      //PS1:16
    TMR2ON = 1;
    while(!TMR2IF);
    TRISCbits.TRISC1 = 0;  
    TRISCbits.TRISC2 = 0; 
    
}