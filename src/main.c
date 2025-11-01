/*
    Actividad Experimental N°2 - Sistemas Embebidos
    Tema: Manejo de Temporizadores e Interrupciones en Microcontroladores 
    ATmega2560 y ATmega328p.
    Autor: Veron Gonzalo Manuel
*/

// --------------------- Definiciones --------------------------
#define F_CPU 16000000UL

// --------------------- Inclusiones --------------------------
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define tbi(p,b) p ^= _BV(b)
#define is_low(p,b) (p & _BV(b)) == 0

// Para Z1, Z2 y Z3, utilizo polling.
// Para P1, utilizo interrupción externa (PD2).
// Pulsadores P1, P2, P3
#define PD4_PIN PIND4 // Z1
#define PD5_PIN PIND5 // Z2
#define PD6_PIN PIND6 // Z3
// Defino los pines PB0, PB1, PB2 y PB3 para los datos BCD
// Defino los pines PC0, PC1 y PC2 para controlar los transistores Q1, Q2 y Q3
#define BCD_PORTB PORTB
#define BCD_DDRB  DDRB

#define TRANSISTOR_ALARMA_PORTC PORTC
#define TRANSISTOR_ALARMA_DDRC DDRC
#define Q1_PIN PC0 //Transistor del display 1 LSB
#define Q2_PIN PC1 //Transistor del display 2 MSB
#define Q3_PIN PC2 //Transistor del display 3 (Para sistema apagado)
#define ALARMA PC3 //Salida para la alarma

// --------------------- Declaración de variables -------------------------- 


// Display
volatile uint8_t display_1 = 0; // Display LSB
volatile uint8_t display_2 = 0; // Display MSB
volatile uint8_t display_3 = 0; // (Para sistema apagado)

// variables para timer 1
volatile uint8_t estado_alarma = 0;         // 0:DESACTIVADA, 1:ACTIVADA, 2:SONANDO, 3:CONTANDO 4:SALIDA 5:POST-ALARMA
volatile uint8_t segundos_restantes = 0;    // este valor puede ser iniciar en 10 o 15
volatile uint8_t zona_disparada = 0;

// Pulling
uint8_t Flag_ant_PD4 = 1;
uint8_t Flag_ant_PD5 = 1;
uint8_t Flag_ant_PD6 = 1;
uint8_t Flag_interrup_PD4 = 0;
uint8_t Flag_interrup_PD5 = 0;
uint8_t Flag_interrup_PD6 = 0;

// --------------------- Servicio de rutina a la interrupción (RSI) -------------------------- 
ISR (INT0_vect){// Interrupción externa para P1
    if (estado_alarma == 0){
        estado_alarma = 4;
        segundos_restantes = 16;

    }
    else if(estado_alarma == 1 || estado_alarma == 2 || estado_alarma == 3 || estado_alarma == 4){
        estado_alarma = 0;
        TRANSISTOR_ALARMA_PORTC &= ~(1 << ALARMA);
        display_1 = 0;
        display_2 = 0;
        display_3 = 0;
    }
}

ISR (TIMER0_OVF_vect){
    // Lo que hago con esta variable es recordar que display fue
    // encendido cada vez que se ejecuta la interrupción
    // 244 veces por segundo.
    static uint8_t display_actual = 0;

    // Siguiendo la secuencia que esta en el pdf de multiplexado
    // apago todos los display, esto equivale al punto 3 y 6
    // Con esto, pogo en la base de los transistores NPN un 0
    TRANSISTOR_ALARMA_PORTC &= ~((1 << Q1_PIN) | (1 << Q2_PIN) | (1 << Q3_PIN));

    switch (display_actual)
    {
        case 0:
            if(estado_alarma == 2 || estado_alarma == 3 || estado_alarma == 4 || estado_alarma == 5){
                // Escribo el valor BCD en los pines PB0...PB3
                // Con la primera mascara me aseguro de no tocar los transistores
                // y con la segunda mascara coloco el valor del display 1 (LSB)
                //BCD_PORTB = (BCD_PORTB & 0b11110000) | (display_1 & 0b00001111);
                BCD_PORTB = (display_1 & 0b00001111);

                // Ahora solo enciendo el transistor Q1
                TRANSISTOR_ALARMA_PORTC |= (1 << Q1_PIN);
                break;
            }
        
        case 1:
            if(estado_alarma == 2 || estado_alarma == 3 || estado_alarma == 4){
                // Escribo el valor BCD en los pines PB0...PB3
                // Con la primera mascara me aseguro de no tocar los transistores
                // y con la segunda mascara coloco el valor del display 2 (Decena)
                //BCD_PORTB = (BCD_PORTB & 0b11110000) | (display_2 & 0b00001111);
                BCD_PORTB = (display_2 & 0b00001111);

                // Ahora solo enciendo el transistor Q1
                TRANSISTOR_ALARMA_PORTC |= (1 << Q2_PIN);
                break;
            }
            

        case 2:
            if (estado_alarma == 0){
                // Escribo el valor BCD en los pines PB0...PB3
                // Con la primera mascara me aseguro de no tocar los transistores
                // y con la segunda mascara coloco el valor del display 2 (Decena)
                //BCD_PORTB = (BCD_PORTB & 0b11110000) | (display_3 & 0b00001111);
                BCD_PORTB = (display_3 & 0b00001111);

                // Ahora solo enciendo el transistor Q1
                TRANSISTOR_ALARMA_PORTC |= (1 << Q3_PIN);
                break;
            }
    }


    // Ahora avanzo al siguiente display
    display_actual+=1;
    if (display_actual == 3){
    //reseteo el display actual
        display_actual = 0;
    }
    
}

ISR (TIMER1_COMPA_vect){

    if(estado_alarma == 2 || estado_alarma == 3 || estado_alarma == 4 || estado_alarma == 5){
        segundos_restantes -= 1;
        if(segundos_restantes == 0){
            if(estado_alarma == 3){
                estado_alarma = 2;
                segundos_restantes = 10;
                zona_disparada = 1;
            }
            else if (estado_alarma == 2)
            {
                estado_alarma = 5; //Termino de sonar la alarma, muestro la zona
                segundos_restantes = 4;
                display_1 = zona_disparada;
            }

            else if (estado_alarma == 4 || estado_alarma == 5)
            {
                estado_alarma = 1;
            }
            
        }
        if(estado_alarma == 2){
            TRANSISTOR_ALARMA_PORTC |= (1 << ALARMA);
        }

        if(estado_alarma == 2 || estado_alarma == 3 || estado_alarma == 4){
            display_2 = segundos_restantes / 10;
            display_1 = segundos_restantes % 10;
        }

    }
    else if(estado_alarma == 1){
        TRANSISTOR_ALARMA_PORTC &= ~(1 << ALARMA);
    }
}

// --------------------- Declaración de funciones -------------------------- 

void conf_PUERTOS_INT0(void); // Configuración de puertos e interrupción externa P1
void conf_TIMER0(void); // Configuración del TIMER0 para multiplexación
void conf_TIMER1(void); // Para que nos de una escalña de tiempo de 1 seg.
void polling_pd4(void);
void polling_pd5(void);
void polling_pd6(void);

int main(void)
{
    conf_PUERTOS_INT0();
    conf_TIMER0();
    conf_TIMER1();
    while (1){
        polling_pd4();
        polling_pd5();
        polling_pd6();
        Flag_interrup_PD4 = 0;
        Flag_interrup_PD5 = 0;
        Flag_interrup_PD6 = 0;
    }   
}


// --------------------- Definición de funciones -------------------------- 
void conf_PUERTOS_INT0(void){

    BCD_DDRB = 0b00001111; // Configuramos PB0...PB3 como salida
    TRANSISTOR_ALARMA_DDRC = 0b00001111; // Configuramos PC0...PC2 como salida

    // PD4, PD5 y PD6 como entradas con pull-up
    DDRD &= ~((1 << PD4_PIN) | (1 << PD5_PIN) | (1 << PD6_PIN));
    PORTD &= ~((1 << PD4_PIN) |  (1 << PD5_PIN) | (1 << PD6_PIN));

    //Configuración de interrupción externa INT0 para P1 (PD2)
    EICRA = (1 << ISC00) | (1 << ISC01);
    EIMSK = (1 << INT0);
    EIFR = (1 << INTF0);
    sei();
}

void conf_TIMER0(void){

    // Modo Normal (WGM00 = 0, WGM01 = 0)
    // COM0A1, COM0A0, COM0B1, COM0B0 también igual a 0 
    TCCR0A = 0x00; 

    // Prescaler N=256 (CS02=1, CS01=0, CS00=0)
    // FOC0A, FOC0B y WGM02 igual a cero
    TCCR0B = (1 << CS02); // 0b00000100

    // Habilito la interrupción por desbordamiento
    TIMSK0 = (1 << TOIE0); // 0b00000001

    // Como explique antes, no voy a utilizar el VPC.
}

void conf_TIMER1(void){

    // Modo CTC (WGM11=0, WGM10=0)
    TCCR1A = 0b00000000;

    // Modo CTC (WGM12=1) y Prescaler N=256 (CS12=1)
    TCCR1B = (1 << WGM12) | (1 << CS12); // 0b00001100 o 0x0C

    // Establezco el valor de comparación para 1 seg
    OCR1A = 62499;

    // Habilito la interrupción por comparación A
    TIMSK1 = (1 << OCIE1A);
}

void polling_pd4(void){
    if(Flag_ant_PD4==1 && is_low(PIND,PD4_PIN)) {       
        _delay_ms(1);
        if(Flag_ant_PD4==1 && is_low(PIND,PD4_PIN)){
            if (estado_alarma == 1){
                segundos_restantes = 16;
                estado_alarma = 3;
            }
            else if(estado_alarma == 0){
                while (is_low(PIND,PD4_PIN))
                {
                    display_3 = 1;
                }
                
                display_3 = 0;
            }
            
            Flag_interrup_PD4 = 1;
            Flag_ant_PD4 = 0;
        }
    }
    if(!is_low(PIND,PD4_PIN)){
        _delay_ms(1);
        if(!is_low(PIND,PD4_PIN)){
            Flag_ant_PD4 = 1;
        }
    }
}

void polling_pd5(void){
    if(Flag_ant_PD5==1 && is_low(PIND,PD5_PIN)) {       
        _delay_ms(1);
        if(Flag_ant_PD5==1 && is_low(PIND,PD5_PIN)){
            if (estado_alarma == 1 || estado_alarma == 4 || estado_alarma == 3){
                segundos_restantes = 11;
                estado_alarma = 2;
                zona_disparada = 2;
            }
            else if(estado_alarma == 0){
                while (is_low(PIND,PD5_PIN))
                {
                    display_3 = 2;
                }
                
                display_3 = 0;
            }
            Flag_interrup_PD5 = 1;
            Flag_ant_PD5 = 0;
        }
    }
    if(!is_low(PIND,PD5_PIN)){
        _delay_ms(1);
        if(!is_low(PIND,PD5_PIN)){
            Flag_ant_PD5 = 1;
        }
    }
}

void polling_pd6(void){
    if(Flag_ant_PD6==1 && is_low(PIND,PD6_PIN)) {       
        _delay_ms(1);
        if(Flag_ant_PD6==1 && is_low(PIND,PD6_PIN)){
            if (estado_alarma == 1 || estado_alarma == 4 || estado_alarma == 3){
                segundos_restantes = 11;
                estado_alarma = 2;
                zona_disparada = 3;
            }
            else if(estado_alarma == 0){
                while (is_low(PIND,PD6_PIN))
                {
                    display_3 = 3;
                }
                
                display_3 = 0;
            }
            Flag_interrup_PD6 = 1;
            Flag_ant_PD6 = 0;
        }
    }
    if(!is_low(PIND,PD6_PIN)){
        _delay_ms(1);
        if(!is_low(PIND,PD6_PIN)){
            Flag_ant_PD6 = 1;
        }
    }
}


