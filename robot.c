/*
 * robot.c
 *
 *  Created on: 5 may. 2025
 *      Author: alici
 */

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
#define ROBOT 1
#define P_GOAL_POSITION_L (30)
#define P_GOAL_POSITION_H (31)
#define P_GOAL_SPEED_L (32)
#define P_GOAL_SPEED_H (33)
//No s'utilitza, pero es informació treta del manual del sensor
#define P_CW_ANGLE_LIMIT_L 6
#define P_CW_ANGLE_LIMIT_H 7
#define P_CCW_ANGLE_LIMIT_L 8
#define P_CCW_ANGLE_LIMIT_H 9
#define INST_WRITE 0x03
#define ID_DRETA 2
#define ID_ESQUERRA 3

#define VELOCITAT_45_GRAUS 900
#define TEMPS_45_GRAUS 280
#define DIST_SEGURETAT 30
#define MARGEN_ERROR 5

//typedef unsigned char byte;
#include "lib_PAE.h"
#include "lib_PAE2.h"
#include <stdbool.h>

//typedef struct RxReturn{
//    uint8_t StatusPacket[32];
//    bool timeOut;
//} RxReturn;
typedef uint8_t byte;
volatile uint16_t cnt, espera;
#define TXD0_READY (UCA2IFG & UCTXIFG)
volatile bool Byte_Recibido ;
volatile uint8_t DatoLeido_UART;
volatile bool principi;
char cadenaEsquerra[16];
char cadenaCentre[16];
char cadenaDreta[16];
volatile uint8_t lineaCentre = 0;
volatile uint8_t lineaEsquerra = 1;
volatile uint8_t lineaDreta = 2;


void Init_UART(void)
{
    P3SEL0 &= ~BIT0;
    P3SEL1 &= ~BIT0;
    P3DIR |= BIT0;
    P3OUT &= ~BIT0;
    UCA2CTLW0 |= UCSWRST; //Fem un reset de la USCI, desactiva la USCI
    UCA2CTLW0 |= UCSSEL__SMCLK; //UCSYNC=0 mode asíncron
    //UCMODEx=0; //seleccionem mode UART
    //UCSPB=0; //nomes 1 stop bit
    //UC7BIT=0; //8 bits de dades
    //UCMSB=0; //bit de menys pes primer
    //UCPAR=x; //ja que no es fa servir bit de paritat
    //UCPEN=0;  //sense bit de paritat
    //Triem SMCLK (24MHz) com a font del clock BRCLK
    UCA2MCTLW = UCOS16; // Necessitem sobre-mostreig => bit 0 = UCOS16 = 1
    UCA2BRW = 3; //Prescaler de BRCLK fixat a 3. Com SMCLK va a24MHz,
    //volem un baud rate de 500 kbps i fem sobre-mostreig de 16
    //el rellotge de la UART ha de ser de ~8MHz (24MHz/3).
    //UCA0MCTLW |= (0x25 << 8); UCBRSx, part fractional del baud rate

    if(ROBOT == 1){ // Si es fa ús de la versió real
        //Configurem els pins de la UART corresponents amb la UCA2
        P3SEL0 |= BIT2 | BIT3; //I/O funció: P3.3 = UART0TX, P3.2 = UART0RX
        P3SEL1 &= ~ (BIT2 | BIT3);
    }else{
        //Configurem els pins de la UART corresponents amb la UCA0
        P1SEL0 |= BIT2 | BIT3; //I/O funció: P1.3 = UART0TX, P1.2 = UART0RX
        P1SEL1 &= ~ (BIT2 | BIT3);
    }

    UCA2CTLW0 &= ~UCSWRST; //Reactivem la línia de comunicacions sèrie
    EUSCI_A2->IFG &= ~EUSCI_A_IFG_RXIFG; // Clear eUSCI RX interrupt flag
    EUSCI_A2->IE |= EUSCI_A_IE_RXIE; // Enable USCI_A2 RX interrupt, nomes quan tinguem la recepcio
    NVIC->ICPR[0] |= 1 << ((EUSCIA2_IRQn) & 31);
    NVIC->ISER[0] = 1 << ((EUSCIA2_IRQn) & 31);
    NVIC->ICPR[0] |= 1 << (TA1_0_IRQn & 31);
    NVIC->ISER[0] |= 1 << (TA1_0_IRQn & 31);
    NVIC->ICPR[0]= 1 << (TA0_0_IRQn & 31); //Los timers están en la primera parte de los registros
    NVIC->ISER[0]= 1 << (TA0_0_IRQn & 31);
    TIMER_A1->CTL = TIMER_A_CTL_ID_1 | TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_CLR
                | TIMER_A_CTL_MC__UP;
    TIMER_A1->CCR[0] = 240 - 1;     // 1 Hz
    TIMER_A1->CCTL[0] |= TIMER_A_CCTLN_CCIE; //Interrupciones activadas en CCR0
    TIMER_A0->CTL  = TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_ID__1
                   | TIMER_A_CTL_MC__UP | TIMER_A_CTL_CLR;
    TIMER_A0->CCR[0] = 24000 - 1;           // 24 MHz / 24 000 = 1 kHz → 1 ms
    TIMER_A0->CCTL[0] |= TIMER_A_CCTLN_CCIE;

}


/* funcions per canviar el sentit de les comunicacions */
void Sentit_Dades_Rx(void)
{

    //Configuració del Half Duplex dels motors: Recepció
    P3OUT &= ~BIT0; //El pin P3.0 (DIRECTION_PORT) el posem a 0 (Rx)
}
void Sentit_Dades_Tx(void)
{

    //Configuració del Half Duplex dels motors: Transmissió
    P3OUT |= BIT0; //El pin P3.0 (DIRECTION_PORT) el posem a 1 (Tx)
}

/* funció TxUACx(byte): envia un byte de dades per la UART 0 */
void TxUACx(uint8_t bTxdData)
{
    while(!TXD0_READY); // Espera a que estigui preparat el buffer de transmissió
    UCA2TXBUF = bTxdData;
}

//TxPacket() 3 paràmetres: ID del Dynamixel, Mida dels paràmetres, Instruction byte. torna la mida del "Return packet"
byte TxPacket(byte bID, byte bParameterLength, byte bInstruction , byte Parametros[16])
{
    byte bCount,bCheckSum,bPacketLength;
    byte TxBuffer[32];
    Sentit_Dades_Tx(); //El pin P3.0 (DIRECTION_PORT) el posem a 1 (Transmetre)
    TxBuffer[0] = 0xff; //Primers 2 bytes que indiquen inici de trama FF, FF.
    TxBuffer[1] = 0xff;
    TxBuffer[2] = bID; //ID del mòdul al que volem enviar el missatge
    TxBuffer[3] = bParameterLength+2; //Length(Parameter,Instruction,Checksum)
    TxBuffer[4] = bInstruction; //Instrucció que enviem al Mòdul
    char error[] = "adr. no permitida";
    if ((Parametros[0] < 6) && (bInstruction == 3)){//si se intenta escribir en una direccion <= 0x05,
        //emitir mensaje de error de direccion prohibida:
        //halLcdPrintLine(error, 8, INVERT_TEXT);
        //y salir de la funcion sin mas:
        return 0;
    }
    for(bCount = 0; bCount < bParameterLength; bCount++) //Comencem a generar la trama que hem d’enviar
    {
        TxBuffer[bCount+5] = Parametros[bCount];
    }
    bCheckSum = 0;
    bPacketLength = bParameterLength+4+2;
    for(bCount = 2; bCount < bPacketLength-1; bCount++) //Càlcul del checksum
    {
        bCheckSum += TxBuffer[bCount];
    }
    TxBuffer[bCount] = ~bCheckSum; //Escriu el Checksum (complement a 1)
    for(bCount = 0; bCount < bPacketLength; bCount++) //Aquest bucle és el que envia la trama al Mòdul Robot
    {
        TxUACx(TxBuffer[bCount]);
    }
    while( (UCA2STATW & UCBUSY)); //Espera fins que s’ha transmès el últim byte
    Sentit_Dades_Rx(); //Posem la línia de dades en Rx perquè el mòdul Dynamixel envia resposta
    return(bPacketLength);
}

void delay_t(uint32_t temps)
//Aquesta funció pren un paràmetre temps, que indica la durada del retard en unitats de cicles de
//rellotge del microcontrolador. Aquest paràmetre és de tipus uint32_t, que és un enter sense signe de 32 bits.
{
    volatile uint32_t i;
    //Declara una variable anomenada i de 32 bits sense signe que es marca com volatile

    /**
     * TODO PER PART DEL ALUMNE AMB UN BUCLE FOR
     * Un cop implementat, comenteu o elimineu la seguent funcio
     **/

    //__delay_cycles(RETRASO);
    //Solucio
    for (i = 0; i < temps; i++);
    //El bucle en si mateix s'executarà temps vegades, com no hi ha cap instrucció dins del bucle, simplement s'executarà
    //sense fer res i es detindrà quan i sigui igual a temps.
}

void EUSCIA2_IRQHandler(void)
{ //interrupcion de recepcion en la UART A2
    EUSCI_A2->IFG &=~ EUSCI_A_IFG_RXIFG; // Clear interrupt
    UCA2IE &= ~UCRXIE; //Interrupciones desactivadas en RX
    DatoLeido_UART = UCA2RXBUF;
    Byte_Recibido=true;
    UCA2IE |= UCRXIE; //Interrupciones reactivadas en RX
}

void Activa_TimerA1_TimeOut() {
    TA1CCTL0 &= ~TIMER_A_CCTLN_CCIFG; //Clear interrupt flag
    TA1CCR0 = 240 - 1;
    cnt = 0;
}

void Desactiva_TimerA1_TimeOut() {
    TA1CCR0 = 0;
    //TIMER_A1->CCTL[0] &= ~TIMER_A_CCTLN_CCIE; // Deshabilita la interrupción en CCR0
}

void TA1_0_IRQHandler(void)
{
    TA1CCTL0 &= ~TIMER_A_CCTLN_CCIFG; //Clear interrupt flag
    cnt++;
}

byte TimeOut(unsigned int tiempo_espera) {
    if(cnt >= tiempo_espera){
        return 1;
    }else{
        return 0;
    }
}

void Reset_Timeout() {
    cnt = 0;
}

struct RxReturn RxPacket(void)
{
    struct RxReturn respuesta;
    byte bCount, bLenght, bChecksum;
    byte Rx_time_out=0;
    Sentit_Dades_Rx(); //Ponemos la linea half duplex en Rx
    Activa_TimerA1_TimeOut();
    for(bCount = 0; bCount < 4; bCount++) //bRxPacketLength; bCount++)
    {
        Reset_Timeout();
        Byte_Recibido=false; //No_se_ha_recibido_Byte();
        while (!Byte_Recibido) //Se_ha_recibido_Byte())
        {
            Rx_time_out=TimeOut(1000); // tiempo en decenas de microsegundos
            if (Rx_time_out)break;//sale del while
         }
        if (Rx_time_out)break; //sale del for si ha habido Timeout
        //Si no, es que todo ha ido bien, y leemos un dato:
        respuesta.StatusPacket[bCount] = DatoLeido_UART; //Get_Byte_Leido_UART();
    }//fin del for
    if (!Rx_time_out)
    // Continua llegint la resta de bytes del Status Packet
    {
        // El cuarto byte contiene la longitud de los datos restantes
        bLenght = respuesta.StatusPacket[3];

        // Leer los bytes restantes según la longitud indicada
        for (bCount; bCount < bLenght + 4; bCount++)  // Ajustar el inicio del for según los bytes ya leídos
        {
            Reset_Timeout();
            Byte_Recibido = false;
            while (!Byte_Recibido)
            {
                Rx_time_out = TimeOut(1000);
                if (Rx_time_out) break;
            }
            if (Rx_time_out) break;

            respuesta.StatusPacket[bCount] = DatoLeido_UART;
         }
     }
    Desactiva_TimerA1_TimeOut();
    respuesta.timeOut = (bool)Rx_time_out;
    return respuesta;
}

//Els angles han d'estar set a 0 per funcionar correctament, tret del Manual del motor
void config_angle_limit(byte bID){
    byte bInstruction = 3;
    byte gbpParameter[5];

    // Configurar CW y CCW Angle Limit a 0 para que funcione en modo continuo
    gbpParameter[0] = P_CW_ANGLE_LIMIT_L;  // Dirección baja de CW Angle Limit
    gbpParameter[1] = 0x00;                // CW Angle Limit(L) a 0
    gbpParameter[2] = 0x00;                // CW Angle Limit(H) a 0
    gbpParameter[3] = 0x00;                // CCW Angle Limit(L) a 0
    gbpParameter[4] = 0x00;                // CCW Angle Limit(H) a 0
    TxPacket(bID, 5, bInstruction, gbpParameter);
    RxPacket();
}

void moureMotor(byte id, int direction, int speed){
    byte gbpParameter[3];
    gbpParameter[0] = P_GOAL_SPEED_L;
    //Calcular Range
    gbpParameter[1] = speed & 0xFF; //LOW, coje los 8 bits menos significativos de la velocidad
    gbpParameter[2] = ((speed >> 8) & 0x03 | (direction << 2) & 0x04); //HIGH, los 2 bits más altos
    TxPacket(id, 3, 3, gbpParameter);
    RxPacket();
}

void moureEnrere(byte id_1, byte id_2, int speed){
    moureMotor(id_1, 0, speed); //Los motores estan en espejo
    moureMotor(id_2, 1, speed);
}

void moureEndevant(byte id_1, byte id_2, int speed){
    moureMotor(id_1, 1, speed);
    moureMotor(id_2, 0, speed);
}

void pivotarSobreSiMateix(byte id_1, byte id_2, int speed){
    moureMotor(id_1, 1, speed);
    moureMotor(id_2, 1, speed);
}

void pivotarSobreSiMateixDreta(byte id_1, byte id_2, int speed){
    moureMotor(id_1, 0, speed);
    moureMotor(id_2, 0, speed);
}

void moureDreta(byte id_1, byte id_2, int speed_drt, int speed_esq){
    moureMotor(id_1, 1, speed_drt);
    moureMotor(id_2, 0, speed_esq);
}

void moureEsquerra(byte id_1, byte id_2, int speed_drt, int speed_esq){
    moureMotor(id_1, 1, speed_drt);
    moureMotor(id_2, 0, speed_esq);
}

struct RxReturn sensor(byte id_sensor){
    byte gbpParameter[2];
    gbpParameter[0] = 0x1A; //Dirección de memoria del sensor
    gbpParameter[1] = 3;
    TxPacket(id_sensor, 2, 0x02, gbpParameter);
    return RxPacket();
}

int sensorDreta(){
    struct RxReturn respuesta2 = sensor(100); //100 és el id del sensor
    return respuesta2.StatusPacket[7];
}

int sensorEsquerra(){
    struct RxReturn respuesta2 = sensor(100);
    return respuesta2.StatusPacket[5];
}

int sensorCentre(){
    struct RxReturn respuesta2 = sensor(100);
    return respuesta2.StatusPacket[6];
}

void TA0_0_IRQHandler(void) {
    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;
    espera++;
}

void Activa_TimerA0_TimeOut() {
    TA0CCTL0 &= ~TIMER_A_CCTLN_CCIFG; //Clear interrupt flag
    TA0CCR0 = (1 << 15) - 1;
    espera = 0;
}

void Desactiva_TimerA0_TimeOut() {
    TA0CCR0 = 0;
    //TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIE; // Deshabilita la interrupción en CCR0
}

void esperaTemps(uint32_t temps){
    Activa_TimerA0_TimeOut();
    while(espera < temps);
    Desactiva_TimerA0_TimeOut();
}

void esperar(uint32_t temps){
    Reset_Timeout();
    while (!TimeOut(temps)){}
}


void buscarParet(){

    int distCentre = sensorCentre();
    int distancias[8];
    int min_dist = 0;
    int pos_min = 0;
    int i;

    // Escanear los 360 grados, yatusabe
    for(i = 0; i < 8; i++) {
        pivotarSobreSiMateixDreta(ID_DRETA, ID_ESQUERRA, VELOCITAT_45_GRAUS);
        esperaTemps(TEMPS_45_GRAUS);

        noMoure();
        esperaTemps(500);

        // Leer sensor centro
        distancias[i] = sensorCentre();
        if(distancias[i] > min_dist) {
            min_dist = distancias[i];
            pos_min = i;
        }
    }

    // orientar hacia paret m'as cercana
    for(i = 0; i < pos_min; i++) {
        pivotarSobreSiMateixDreta(ID_DRETA, ID_ESQUERRA, VELOCITAT_45_GRAUS);
        esperaTemps(TEMPS_45_GRAUS);
    }

    while (distCentre < (DIST_SEGURETAT)){
        distCentre = sensorCentre();
        moureEndevant(ID_DRETA, ID_ESQUERRA, 700);
    }

    pivotarSobreSiMateixDreta(ID_DRETA, ID_ESQUERRA, VELOCITAT_45_GRAUS); // *2 el temps d'espera per fer 90 graus aprox
    esperaTemps(TEMPS_45_GRAUS*2);
}

void seguirParetEsquerra(){
    int distEsq = sensorEsquerra();
    int distCent = sensorCentre();
    int distDre = sensorDreta();

    int error = distEsq - DIST_SEGURETAT;

    if (distEsq > 88 && distCent > 88 && distDre > 88){
        encerrado();
        return;
    }


    if (distCent > (DIST_SEGURETAT+7)){ // bajar A DIST_SEGURETAT+0?
        pivotarSobreSiMateixDreta(ID_DRETA, ID_ESQUERRA, 700);
        esperaTemps(600);
        return;
    }

    if (abs(error) > MARGEN_ERROR){
        if (error > 0){ // cuando el error es positivo significa que distEsq es grande (por ende, cerca)
            moureDreta(ID_DRETA, ID_ESQUERRA, 0, 900);
            esperaTemps(1);
        }
        else {
            if (distEsq < 2){
                pivotarSobreSiMateix(ID_DRETA, ID_ESQUERRA, 900); // 700 maybe, antes 500
                esperaTemps(1);
            }
            else{
                moureEsquerra(ID_DRETA, ID_ESQUERRA, 500, 0);
                esperaTemps(1);
            }
        }
    }

    moureEndevant(ID_DRETA, ID_ESQUERRA, 500); // antes 500

}

void seguirParetDreta(){
    int distEsq = sensorEsquerra();
    int distCent = sensorCentre();
    int distDre = sensorDreta();

    int error = distDre - DIST_SEGURETAT;

    if (distEsq > 88 && distCent > 88 && distDre > 88){
        encerrado();
        return;
    }

    if (distCent > (DIST_SEGURETAT+7)){ // bajar A DIST_SEGURETAT+0?
        pivotarSobreSiMateix(ID_DRETA, ID_ESQUERRA, 700);
        esperaTemps(600);
        return;
    }

    if (abs(error) > MARGEN_ERROR){
        if (error > 0){ // cuando el error es positivo significa que distEsq es grande (por ende, cerca)
            moureEsquerra(ID_DRETA, ID_ESQUERRA, 900, 0);
            esperaTemps(1);
        }
        else {
            if (distDre < 2){
                pivotarSobreSiMateixDreta(ID_DRETA, ID_ESQUERRA, 900);
                esperaTemps(1);
            }
            else{
                moureDreta(ID_DRETA, ID_ESQUERRA, 0, 500);
                esperaTemps(1);
            }
        }
    }

    moureEndevant(ID_DRETA, ID_ESQUERRA, 500);

}

void encerrado(){
    pivotarSobreSiMateixDreta(ID_DRETA, ID_ESQUERRA, 700);
    esperaTemps(1200);
}

void Init_LCD(void) {
    halLcdInit(); // Funció d'inicialització de la pantalla
    halLcdClearScreen(0); // Neteja la pantalla (0 per blanc, 1 per negre)
}

void pantallaNoms(){
    halLcdClearScreen(0);
    halLcdPrintLine("Membres:", 0, 0); // Mostra el títol a la primera línia
    halLcdPrintLine("Jiajun Wang", 1, 0); // Mostra el primer nom
    halLcdPrintLine("Alicia Torres", 2, 0); // Mostra el segon nom
    halLcdPrintLine("READY", 3, 0); // Mostra el segon nom

}

struct RxReturn sensorSoroll(byte id_sensor)
{
    byte parametres[16];
    parametres[0] = 0x25;
    parametres[1] = 0x01;
    TxPacket(id_sensor, 2, 0x02, parametres);
    return RxPacket();

}

byte llegir_aplaudiments(){
    struct RxReturn resposta = sensorSoroll(100);
    return resposta.StatusPacket[5];
}

void comprovar_comencament()
{

    /*
     * Seguint la documentació del sensor, podem detectar quants cops aplaudeix
     */
    nombreAplaudiments = llegir_aplaudiments();
    // Si s'ha detectat algun aplaudiment, comencem el robot
    if (nombreAplaudiments != num_aplaudiments_anterior)
    {
        num_aplaudiments_anterior = nombreAplaudiments;

        if(nombreAplaudiments == 2 ){
            programaInicio = 1;
        }
        else {
            programaInicio = 0;
        }
    }
}

void noMoure(){
    moureEndevant(ID_DRETA, ID_ESQUERRA, 0);
}

void giraMitjaVolta(){
    pivotarSobreSiMateix(ID_DRETA, ID_ESQUERRA, 700);
    esperaTemps(1260);
}

void init_boton(){
    P3DIR &= ~BIT5;
    P3OUT |= BIT5;
    P3REN |= BIT5;
    P3IES |= BIT5;
    P3IFG &= ~BIT5;
    P3IE |= BIT5;
    NVIC->ICPR[1] |= 1 << ((PORT3_IRQn) & 31);
    NVIC->ISER[1] |= 1 << ((PORT3_IRQn) & 31);
}

void PORT3_IRQHandler(void){
    P3IE &= ~(BIT5);
    if (P3IFG & BIT5) {
//        canviarSentit = !canviarSentit;
        if (canviarSentit){
            canviarSentit = false;
        }
        else {
            canviarSentit = true;
        }
        gir180 = 0;
    }

    P3IFG &= ~BIT5;
    P3IE |= (BIT5);
}

void initVariables(){
    programaInicio = 0;
    num_aplaudiments_anteriorSentit = 0;
    canviarSentit = false;
    gir180 = 1;
}
