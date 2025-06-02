/******************************
 *
 * Practica_04_PAE UART
 * UB, 05/2025.
 *
 * Autors: Jiajun Wang i Alicia Torres Andrés
 *
 *****************************/
#include "msp.h"
#include "robot.h"
//typedef unsigned char byte;
#include "lib_PAE.h"
#include "lib_PAE2.h"
#include <stdbool.h>

/**
 * main.c
 */
void main(void)
 {

    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer

    init_ucs_24MHz();
    Init_UART();
    Init_LCD();
    init_boton();
    int paretTrobada = 0;

    // Configurar instrucción y parámetros para encender el LED
    byte bInstruction = 3;  // Instrucción de escritura (por ejemplo, 3 para Write Instruction)
    byte bParameterLength = 2;  // Longitud de los parámetros (por ejemplo, 4 para ID + dirección + datos + checksum)
    byte Parametros[16] = {0x19, 0x01};  // Parámetros para encender el LED


    byte bPacketLength = TxPacket(ID_DRETA, bParameterLength, bInstruction, Parametros);
    RxPacket();

    config_angle_limit(ID_DRETA);
    config_angle_limit(ID_ESQUERRA);

    noMoure(); // per borrar l'estat anterior de les rodes (per si s'estaven movent-se)
    initVariables();

    halLcdClearScreenBkg();
    pantallaNoms();
    esperaTemps(5000); // 5 segons
    halLcdClearScreen(0);

    while(1){
        if (!programaInicio){ // tenía un "!"
            comprovar_comencament();
        }
        else{
            if (!canviarSentit){
                if (!paretTrobada){
                    buscarParet();
                    paretTrobada = 1;
                }
                else{
                    if (!gir180){
                        giraMitjaVolta();
                        gir180 = 1;
                    }
                    halLcdPrintLine("Go LEFT: ", 5, 0);
                    seguirParetEsquerra();
                }
            }
            else{
                if (!gir180){
                    giraMitjaVolta();
                    gir180 = 1;
                }
                halLcdPrintLine("Go RIGHT:", 5, 0);
                seguirParetDreta();
            }
        }
    }
}

