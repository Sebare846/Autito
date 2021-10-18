#include "mbed.h"
#include <stdbool.h>

#define NUMBUTT             4

#define MAXLED              4

#define NUMBEAT             4

#define INTERVAL    100

#define ULTRASONIDOINTERVAL 200

#define PERIOD 200

#define HEARTBEATINTERVAL 300

/**
 * @brief Enumeración que contiene los estados de la máquina de estados(MEF) que se implementa para 
 * el "debounce" 
 * El botón está configurado como PullUp, establece un valor lógico 1 cuando no se presiona   
 * y cambia a valor lógico 0 cuando es presionado.
 */
typedef enum{
    BUTTON_DOWN,    //0
    BUTTON_UP,      //1
    BUTTON_FALLING, //2
    BUTTON_RISING   //3
}_eButtonState;

/**
 * @brief Enumeración de eventos del botón
 * Se usa para saber si el botón está presionado o nó
 */
typedef enum{
    EV_PRESSED,
    EV_NOT_PRESSED,
    EV_NONE
}_eButtonEvent;

/**
 * @brief Esturctura de Teclas
 * 
 */
typedef struct{
    _eButtonState estado;
    _eButtonEvent event;
    int32_t timeDown;
    int32_t timeDiff;
}_sTeclas;

_sTeclas ourButton[NUMBUTT];
//                0001 ,  0010,  0100,  1000


uint16_t mask[]={0x0001,0x0002,0x0004,0x0008};

/**
 * @brief Enumeración de la MEF para decodificar el protocolo
 * 
 */
typedef enum{
    START,
    HEADER_1,
    HEADER_2,
    HEADER_3,
    NBYTES,
    TOKEN,
    PAYLOAD
}_eProtocolo;

_eProtocolo estadoProtocolo;

/**
 * @brief Enumeración de la lista de comandos
 * 
 */
 typedef enum{
        ACK=0x0D,
        ALIVE=0xF0,
        FIRMWARE = 0xF1,
        GET_IR = 0xA0,
        SET_POWER = 0xA1,
        SET_SERVO = 0xA2,
        GET_DISTANCE = 0xA3,
        GET_SPEED = 0xA4
    }_eID;

/**
 * @brief Estructura de datos para el puerto serie
 * 
 */
typedef struct{
    uint8_t timeOut;         //!< TiemOut para reiniciar la máquina si se interrumpe la comunicación
    uint8_t cheksumRx;       //!< Cheksumm RX
    uint8_t cheksumtx;       //!< Cheksumm Tx
    uint8_t indexWriteRx;    //!< Indice de escritura del buffer circular de recepción
    uint8_t indexReadRx;     //!< Indice de lectura del buffer circular de recepción
    uint8_t indexWriteTx;    //!< Indice de escritura del buffer circular de transmisión
    uint8_t indexReadTx;     //!< Indice de lectura del buffer circular de transmisión
    uint8_t bufferRx[256];   //!< Buffer circular de recepción
    uint8_t bufferTx[256];   //!< Buffer circular de transmisión
    uint8_t payload[32];     //!< Buffer para el Payload de datos recibidos
}_sDato ;

volatile _sDato datosComProtocol;

/**
 * @brief VOLATILE 
 * 
 * La palabra clave volatile se utiliza para evitar que el compilador aplique optimizaciones en variables
 * que pueden cambiar de formas en que el compilador no puede anticipar.
 * Los objetos declarados como volátiles se omiten de la optimización porque sus valores pueden ser cambiados por código 
 * fuera del alcance del código actual en cualquier momento. El sistema siempre lee el valor actual de un objeto volátil 
 * desde la ubicación de la memoria en lugar de mantener su valor en un registro temporal en el punto en el que se solicita, 
 * incluso si una instrucción previa solicitó un valor del mismo objeto. 
 * Se utilizan en variables globales modificadas por una rutina de servicio de interrupción(ISR). 
 * Si no declara la variable como volátil, el compilador optimizará el código.
 * Si no usamos un calificador volátil, pueden surgir los siguientes problemas
 *    1) Es posible que el código no funcione como se esperaba cuando la optimización está activada.
 *    2) Es posible que el código no funcione como se esperaba cuando las interrupciones están habilitadas y utilizadas.
 */



/**
 * @brief Mapa de bits para implementar banderas
 * Como uso una sola los 7 bits restantes los dejo reservados para futuras banderas
 */
typedef union{
    struct{
        uint8_t checkButtons :1;
        uint8_t servoMove: 1;
        uint8_t Reserved :6;
    }individualFlags;
    uint8_t allFlags;
}_bGeneralFlags;

volatile _bGeneralFlags myFlags;

uint8_t heartBeatEvent;

/**
 * @brief Unión para descomponer/componer datos mayores a 1 byte
 * 
 */
typedef union {
    int32_t i32;
    uint32_t ui32;
    uint16_t ui16[2];
    uint8_t ui8[4];
    int8_t i8[4];
}_udat;

_udat myWord;


/**
 * @brief Dato del tipo Enum para asignar los estados de la MEF
 * 
 */
_eButtonState myButton;

/*************************************************************************************************/
/* Prototipo de Funciones */

/**
 * @brief Función que se llama en la interrupción de recepción de datos
 * Cuando se llama la fucnión se leen todos los datos que llagaron.
 */
void onDataRx(void);

/**
 * @brief Decodifica las tramas que se reciben 
 * La función decodifica el protocolo para saber si lo que llegó es válido.
 * Utiliza una máquina de estado para decodificar el paquete
 */
void decodeProtocol(void);

/**
 * @brief Procesa el comando (ID) que se recibió
 * Si el protocolo es correcto, se llama a esta función para procesar el comando
 */
void decodeData(void);

/**
 * @brief Envía los datos a la PC
 * La función consulta si el puerto serie está libra para escribir, si es así envía 1 byte y retorna
 */
void sendData(void);

/**
 * @brief Función que se llama con la interrupción del TICKER
 * maneja el tiempo de debounce de los botones, cambia un falg cada vez que se deben revisar los botones
 */
void OnTimeOut(void);

/**
 * @brief Función Hearbeat
 * Ejecuta las tareas del hearbeat 
 */
// void hearbeatTask(void);
void servoMove(void);

void startEcho(void);

void endEcho(void);

void myTrigger(void);

void getSpeed(void);

void speedMoment1(void);

void speedMoment2(void);

void IRRead(void);

/*****************************************************************************************************/
/* Configuración del Microcontrolador */

DigitalOut HEARBEAT(PC_13); //!< Defino la salida del led

//Pines Servo
PwmOut SERVO(PB_10);

//Pines Horquillas
InterruptIn HR(PB_8);
InterruptIn HL(PB_9);

//Pines Motores
PwmOut MOTOR1(PA_8), MOTOR2(PB_5);
DigitalOut IN1(PB_14),IN2(PB_15),IN3(PB_7),IN4(PB_6);

//Pines Infrarrojo
DigitalOut TRIGGER(PB_13);
InterruptIn ECHO(PB_12);

//Pines Lineas
AnalogIn IRR(PA_0);
AnalogIn IRL(PA_1);

Serial pcCom(PA_9,PA_10,115200); //!< Configuración del puerto serie, la velocidad (115200) tiene que ser la misma en QT


Timer miTimer, echoTimer; //!< Timer general

Ticker timerGeneral, speedTicker;

int8_t anguloServo;

volatile uint32_t contSpeed1, contSpeed2;

uint32_t distance_us, speedM1, speedM2; //Distancia y velocidades leidas

int32_t powerM1, powerM2; //Potencia de los motores

uint16_t valueIRR, valueIRL; //Lecturas de sensores d linea

Timeout pulsoUltrasonido; 

uint8_t debounceTrigger;

/*****************************************************************************************************/
/************  Función Principal ***********************/

int main()
{
    miTimer.start();

    int hearbeatTime = 0;
   
    HR.rise(&speedMoment1);
    HL.rise(&speedMoment2);

    speedTicker.attach_us(&getSpeed,500000);

    echoTimer.start();

    debounceTrigger = 200;

    myFlags.individualFlags.checkButtons=false;

    myFlags.individualFlags.servoMove = false;
    
    pcCom.attach(&onDataRx,Serial::RxIrq);

    timerGeneral.attach_us(&OnTimeOut, 50000);

    ECHO.rise(&startEcho);
    ECHO.fall(&endEcho);
    SERVO.period_ms(20);

    MOTOR1.period_us(1000);
    MOTOR2.period_us(1000);

    


    while(true)
    {
        debounceTrigger--;
        if(!debounceTrigger){
            debounceTrigger = 200;
            TRIGGER.write(1);
            pulsoUltrasonido.attach_us(&myTrigger,10);
        }

        //Seteo motor 1
        MOTOR1.pulsewidth_us(powerM1);
        IN1 = false;
        IN2 = true;
        //Seteo motor 2
        MOTOR2.pulsewidth_us(powerM2);
        IN3 = false;
        IN4 = true;

        if ((miTimer.read_ms()-hearbeatTime) >= HEARTBEATINTERVAL){
            hearbeatTime=miTimer.read_ms();
            HEARBEAT = !HEARBEAT;
        }

        if(datosComProtocol.indexReadRx!=datosComProtocol.indexWriteRx) 
            decodeProtocol();

        if(datosComProtocol.indexReadTx!=datosComProtocol.indexWriteTx) 
            sendData();
    }
    return 0;
}

void myTrigger(void){
    TRIGGER.write(0);
}

void startEcho(void){
    echoTimer.reset();
}

void endEcho(void){
    distance_us = echoTimer.read_us();
    // if(distance_us <= 600)
    //     HEARBEAT = 1;
    // else
    //     HEARBEAT = 0;
}

void getSpeed(void){
    speedM1 = contSpeed1;
    speedM2 = contSpeed2;
    contSpeed2 = 0;
    contSpeed1 = 0;
    datosComProtocol.payload[1] = GET_SPEED;
    decodeData();
    valueIRL = IRL.read_u16();
    valueIRR = IRR.read_u16();
    datosComProtocol.payload[1] = GET_IR;
    decodeData();
    datosComProtocol.payload[1] = GET_DISTANCE;
    decodeData();
}

void speedMoment1(void){
    contSpeed1++;
}

void speedMoment2(void){
    contSpeed2++;
}
/*****************************************************************************************************/
/************  MEF para decodificar el protocolo serie ***********************/
void decodeProtocol(void)
{
    static int8_t nBytes=0, indice=0;
    while (datosComProtocol.indexReadRx!=datosComProtocol.indexWriteRx)
    {
        switch (estadoProtocolo) {
            case START:
                if (datosComProtocol.bufferRx[datosComProtocol.indexReadRx++]=='U'){
                    estadoProtocolo=HEADER_1;
                    datosComProtocol.cheksumRx=0;
                }
                break;
            case HEADER_1:
                if (datosComProtocol.bufferRx[datosComProtocol.indexReadRx++]=='N')
                   estadoProtocolo=HEADER_2;
                else{
                    datosComProtocol.indexReadRx--;
                    estadoProtocolo=START;
                }
                break;
            case HEADER_2:
                if (datosComProtocol.bufferRx[datosComProtocol.indexReadRx++]=='E')
                    estadoProtocolo=HEADER_3;
                else{
                    datosComProtocol.indexReadRx--;
                   estadoProtocolo=START;
                }
                break;
        case HEADER_3:
            if (datosComProtocol.bufferRx[datosComProtocol.indexReadRx++]=='R')
                estadoProtocolo=NBYTES;
            else{
                datosComProtocol.indexReadRx--;
               estadoProtocolo=START;
            }
            break;
            case NBYTES:
                nBytes=datosComProtocol.bufferRx[datosComProtocol.indexReadRx++];
               estadoProtocolo=TOKEN;
                break;
            case TOKEN:
                if (datosComProtocol.bufferRx[datosComProtocol.indexReadRx++]==':'){
                   estadoProtocolo=PAYLOAD;
                    datosComProtocol.cheksumRx ='U'^'N'^'E'^'R'^ nBytes^':';
                    datosComProtocol.payload[0]=nBytes;
                    indice=1;
                }
                else{
                    datosComProtocol.indexReadRx--;
                    estadoProtocolo=START;
                }
                break;
            case PAYLOAD:
                if (nBytes>1){
                    datosComProtocol.payload[indice++]=datosComProtocol.bufferRx[datosComProtocol.indexReadRx];
                    datosComProtocol.cheksumRx ^= datosComProtocol.bufferRx[datosComProtocol.indexReadRx++];
                }
                nBytes--;
                if(nBytes<=0){
                    estadoProtocolo=START;
                    if(datosComProtocol.cheksumRx == datosComProtocol.bufferRx[datosComProtocol.indexReadRx]){
                        decodeData();
                    }
                }
                break;
            default:
                estadoProtocolo=START;
                break;
        }
    }

}


/*****************************************************************************************************/
/************  Función para procesar el comando recibido ***********************/
void decodeData(void)
{
    uint8_t auxBuffTx[50], indiceAux=0, cheksum;
    auxBuffTx[indiceAux++]='U';
    auxBuffTx[indiceAux++]='N';
    auxBuffTx[indiceAux++]='E';
    auxBuffTx[indiceAux++]='R';
    auxBuffTx[indiceAux++]=0;
    auxBuffTx[indiceAux++]=':';

    switch (datosComProtocol.payload[1]) {
        case ALIVE: //Confirma Conexion
            auxBuffTx[indiceAux++]=ALIVE;
            auxBuffTx[indiceAux++]=ACK;
            auxBuffTx[NBYTES]=0x03;
            break;
        case SET_POWER: //Setea potecia de motores
            myWord.i8[0] = datosComProtocol.payload[2];
            myWord.i8[1] = datosComProtocol.payload[3];
            myWord.i8[2] = datosComProtocol.payload[4];
            myWord.i8[3] = datosComProtocol.payload[5];
            powerM1 = myWord.i32;
            myWord.i8[0] = datosComProtocol.payload[6];
            myWord.i8[1] = datosComProtocol.payload[7];
            myWord.i8[2] = datosComProtocol.payload[8];
            myWord.i8[3] = datosComProtocol.payload[9];
            powerM2 = myWord.i32;
            auxBuffTx[indiceAux++] = SET_POWER;
            auxBuffTx[indiceAux++] = ACK;
            auxBuffTx[NBYTES] = 0x03;
            break;
        case SET_SERVO: //Mueve el servo
            auxBuffTx[indiceAux++] = SET_SERVO;
            anguloServo = datosComProtocol.payload[2];
            servoMove();
            // if(myFlags.individualFlags.servoMove){
            //     auxBuffTx[indiceAux++] = 0x0A;
            //     myFlags.individualFlags.servoMove = true;
            // }else{
            auxBuffTx[indiceAux++] = ACK;
            //     myFlags.individualFlags.servoMove = true;
            //     servoMove();
            // }
            auxBuffTx[NBYTES] = 0x03;
            break;
        case GET_IR: //Envia los datos de los sensores de linea
            auxBuffTx[indiceAux++] = GET_IR;
            myWord.ui16[0] = valueIRR;
            auxBuffTx[indiceAux++] = myWord.ui8[0];
            auxBuffTx[indiceAux++] = myWord.ui8[1];
            myWord.ui16[0] = valueIRL;
            auxBuffTx[indiceAux++] = myWord.ui8[0];
            auxBuffTx[indiceAux++] = myWord.ui8[1];
            auxBuffTx[NBYTES] = 0x06;
            break;
        case GET_DISTANCE: //Envia los datos obtenidos por el infrarrojo
            auxBuffTx[indiceAux++] = GET_DISTANCE;
            myWord.ui32 = distance_us;
            auxBuffTx[indiceAux++] = myWord.ui8[0];
            auxBuffTx[indiceAux++] = myWord.ui8[1];
            auxBuffTx[indiceAux++] = myWord.ui8[2];
            auxBuffTx[indiceAux++] = myWord.ui8[3];  
            auxBuffTx[NBYTES] =0x06;          
            break;
        case GET_SPEED: //Envia la velocidad de las ruedas
            //Leer velocidad
            auxBuffTx[indiceAux++] = GET_SPEED;
            myWord.ui32 = speedM1;
            auxBuffTx[indiceAux++] = myWord.ui8[0];
            auxBuffTx[indiceAux++] = myWord.ui8[1];
            auxBuffTx[indiceAux++] = myWord.ui8[2];
            auxBuffTx[indiceAux++] = myWord.ui8[3];            
            myWord.ui32 = speedM2;
            auxBuffTx[indiceAux++] = myWord.ui8[0];
            auxBuffTx[indiceAux++] = myWord.ui8[1];
            auxBuffTx[indiceAux++] = myWord.ui8[2];
            auxBuffTx[indiceAux++] = myWord.ui8[3];            
            auxBuffTx[NBYTES] = 0x0A;
            break;
        default:
            auxBuffTx[indiceAux++]=0xDD;
            auxBuffTx[NBYTES]=0x02;
            break;
    }
   cheksum=0;
   for(uint8_t a=0 ;a < indiceAux ;a++){
       cheksum ^= auxBuffTx[a];
       datosComProtocol.bufferTx[datosComProtocol.indexWriteTx++]=auxBuffTx[a];
   }
    datosComProtocol.bufferTx[datosComProtocol.indexWriteTx++]=cheksum;
}


/*****************************************************************************************************/
/************  Función para enviar los bytes hacia la pc ***********************/
void sendData(void)
{
    if(pcCom.writable())
        pcCom.putc(datosComProtocol.bufferTx[datosComProtocol.indexReadTx++]);

}

/**********************************************************************************/
/* Servicio de Interrupciones*/

void onDataRx(void)
{
    while (pcCom.readable())
    {
        datosComProtocol.bufferRx[datosComProtocol.indexWriteRx++]=pcCom.getc();
    }
}

void OnTimeOut(void)
{
    if(!myFlags.individualFlags.checkButtons)
        myFlags.individualFlags.checkButtons=true;
}

void servoMove(void){
    anguloServo = (anguloServo*11.11) + 500;

    SERVO.pulsewidth_us(anguloServo);
    
    // SERVO.pulsewidth_us(500);
    // wait_ms(3000);
    // SERVO.pulsewidth_us(2500);
    // wait_ms(3000);
    // SERVO.pulsewidth_us(1500);

}
