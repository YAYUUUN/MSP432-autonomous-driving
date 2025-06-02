/******************************
 * robot.h
 * Cabecera para control de robot con Dynamixel
 * UB, 05/2025.
 * Autors: Jiajun Wang i Alicia Torres Andrés
 *****************************/

#ifndef ROBOT_H
#define ROBOT_H

#include <stdint.h>
#include <stdbool.h>
#include "lib_PAE.h"

// Definiciones de constantes
#define ROBOT 1
#define P_GOAL_POSITION_L (30)
#define P_GOAL_POSITION_H (31)
#define P_GOAL_SPEED_L (32)
#define P_GOAL_SPEED_H (33)
#define P_CW_ANGLE_LIMIT_L 6
#define P_CW_ANGLE_LIMIT_H 7
#define P_CCW_ANGLE_LIMIT_L 8
#define P_CCW_ANGLE_LIMIT_H 9
#define INST_WRITE 0x03
#define ID_DRETA 2
#define ID_ESQUERRA 1
#define TXD0_READY (UCA2IFG & UCTXIFG)

// Tipos de datos
typedef uint8_t byte;

typedef struct RxReturn {
    uint8_t StatusPacket[32];
    bool timeOut;
} RxReturn;

// Variables globales (extern)
extern volatile uint16_t cnt, espera;
extern volatile bool Byte_Recibido;
extern volatile uint8_t DatoLeido_UART;
extern volatile bool principi;
volatile byte num_aplaudiments_anterior;
volatile byte nombreAplaudiments;
volatile bool programaInicio;

volatile byte num_aplaudiments_anteriorSentit;
volatile byte nombreAplaudimentsSentit;
volatile bool canviarSentit;
volatile bool gir180;

// Prototipos de funciones
void Init_UART(void);
void Sentit_Dades_Rx(void);
void Sentit_Dades_Tx(void);
void TxUACx(uint8_t bTxdData);
byte TxPacket(byte bID, byte bParameterLength, byte bInstruction, byte Parametros[16]);
void delay_t(uint32_t temps);
void EUSCIA2_IRQHandler(void);
void Activa_TimerA1_TimeOut(void);
void Desactiva_TimerA1_TimeOut(void);
void TA1_0_IRQHandler(void);
byte TimeOut(unsigned int tiempo_espera);
void Reset_Timeout(void);
struct RxReturn RxPacket(void);
void config_angle_limit(byte bID);
void moureMotor(byte id, int direction, int speed);
void moureEnrere(byte id_1, byte id_2, int speed);
void moureEndevant(byte id_1, byte id_2, int speed);
void pivotarSobreSiMateix(byte id_1, byte id_2, int speed);
void pivotarSobreSiMateixDreta(byte id_1, byte id_2, int speed);
void moureDreta(byte id_1, byte id_2, int speed_drt, int speed_esq);
void moureEsquerra(byte id_1, byte id_2, int speed_drt, int speed_esq);
struct RxReturn sensor(byte id_sensor);
int sensorDreta(void);
int sensorEsquerra(void);
int sensorCentre(void);
void TA0_0_IRQHandler(void);
void Activa_TimerA0_TimeOut(void);
void Desactiva_TimerA0_TimeOut(void);
void esperaTemps(uint32_t temps);
void esperar(uint32_t temps);
void buscarParet();
void seguirParetEsquerra();
void seguirParetDreta();
void encerrado();
void Init_LCD(void);
void pantallaNoms();
struct RxReturn sensorSoroll(byte id_sensor);
void comprovar_comencament();
void final();
byte llegir_aplaudiments();
void noMoure();
void giraMitjaVolta();
void init_boton();
void PORT1_IRQnHandler(void);
void initVariables();

#endif // ROBOT_H
