// P1-TasterLib

#include "microlib.h"
#include "driverlib/timer.h"

#define ADC_CH1_IN_PIN 3
#define ADC_MUX_CTRL_A 4
#define ADC_MUX_CTRL_B 5
#define ADC_MUX_CTRL_C 25

#define ADC_MUX_IN_POTI 1
#define ADC_MUX_IN_IL 6
#define ADC_MUX_IN_IR 7

#define DIR_0 26
#define DIR_1 27
#define PWM_MOTOR 10


#define ENCODER_A 22
#define ENCODER_B 28

#define KP      150
#define KI        3
#define TA       10
#define SCALE  1000


uint16_t Adc_Convert( uint8_t i);
uint16_t absolut16 (int16_t val);
void linearMapToPWM (uint16_t min, uint16_t max, uint16_t value, TIMER_TypeDef Timer);
int16_t linearMap (int16_t value, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max);
void setDirection(uint8_t val); //Sollte aus ISR aufrufbar sein, hier irrelevent
int16_t Limit (int16_t value, int16_t min, int16_t max);

uint16_t Adc_Convert( uint8_t i);
uint16_t AdcToVolt (uint16_t CW);
int16_t PiRegler( int16_t nsoll, int16_t nist );

void encoderISR(void); //Funktion sollte in zwingend in ITCM-RAM (über AHB) @STM32H7
void controlISR(void);


// Folgende Studierende haben den Versuch zusammen erstellt und abgegeben:
char Namen[] = "Michael Boeckelen, Tim Gebhard";

uint16_t RPM = 0, NSOLL = 0; //Globale Variable für RPM
int32_t esum = 0;  // globale Variable, Summe des Regelfehlers


void main(void) {
    System_Init();
    Display_Init(); //Display initialisieren

    Pin_Init(DIR_0, PIN_GPIO_OUT); //LED D1 bis D4 als Ausgang
    Pin_Init(DIR_1, PIN_GPIO_OUT);
    Pin_SetOutput(DIR_0, 1); //Am Anfang lieber mal ausschalten bevor undefine state auftritt
    Pin_SetOutput(DIR_1, 1);


    Pin_Init(ADC_MUX_CTRL_A, PIN_GPIO_OUT); //Pins zur Ansteuerung des ADC-Muxers konfigurieren (Output)
    Pin_Init(ADC_MUX_CTRL_B, PIN_GPIO_OUT);
    Pin_Init(ADC_MUX_CTRL_C, PIN_GPIO_OUT);
    Pin_Init(ADC_CH1_IN_PIN, PIN_GPIO_IN);   //ADC_CH1_IN_PIN als Eingang
    Adc_Init();

    Timer_InitPwm(TIMER_3A, 3999); //PWM für Motor, 20kHz
    Pin_Init(PWM_MOTOR, PIN_TIMER_OUT);

    Timer_InitPwm(TIMER_1A, 7999999); //Regler-Interrupt, 100Hz, M = 799.999
    Isr_Register(IRQ_TIMER_1A, controlISR);
    Timer_EnableOverflowIrq(TIMER_1A);

    Timer_InitCapture(TIMER_2A);
    Pin_Init(ENCODER_A, PIN_TIMER_IN);
    Timer_EnableCaptureIrq(TIMER_2A);
    Isr_Register(IRQ_TIMER_2A, encoderISR);
    Pin_Init(ENCODER_B, PIN_GPIO_IN);

    Isr_EnableAll();

    Uart_Init(UART_0, 115200);


    uint16_t CWPoti, CWIL, CWIR;    //Variablen zum Zwischenspeichern von Werten

       while(1) {

        CWPoti = Adc_Convert(ADC_MUX_IN_POTI);  //Analogwert von Poti auslesen
        CWIL = Adc_Convert(ADC_MUX_IN_IL);
        CWIR = Adc_Convert(ADC_MUX_IN_IR);

        NSOLL = linearMap((int16_t)CWPoti, 0, 4095, 0, 6000);//CWPoti [0;4095] auf das gewünschte Format mappen [0; 6000]

        Display_Printf(0, 0, "CWPoti %05d", CWPoti);
        Display_Printf(1, 0, "nsoll: %05d", NSOLL);
        Display_Printf(2, 0, "IL: %05d", AdcToVolt(CWIL));
        Display_Printf(3, 0, "IR: %05d", AdcToVolt(CWIR));
        Display_Printf(4, 0, "nist: %05d", RPM);

        printf("%d\r\n", RPM);
        Display_Update();

    }
}

void linearMapToPWM (uint16_t min, uint16_t max, uint16_t value, TIMER_TypeDef Timer){

    uint16_t P = (value-min)*4000 / (max-min); //Linear mapping auf PWM-Bereich TODO:Timer-Max aus struct holen

    Timer_SetPwmValue(Timer, P);

    return;
}

int16_t linearMap (int16_t value, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max){

    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; //Lineares Erweitern auf gewünschtes Format

}


uint16_t absolut16 (int16_t val){

    if(val < 0) return -val;
    else return val;

}

void setDirection(uint8_t val){ //0: vorwärts, im Uhrzeigersinn, 1: rückwärts

    Pin_SetOutput(DIR_0, val);
    Pin_SetOutput(DIR_1, !val); //eig. schlecht, in 2 monaten hab ich keine Ahnung mehr was ich hier mache
    return;

}


uint16_t Adc_Convert( uint8_t i){

    Pin_SetOutput(ADC_MUX_CTRL_A, i & (1<<0));  //Mux-Ansteuerung entsprechend i setzen, Vorteil: ABC entspricht Bitwert der Zahl i
    Pin_SetOutput(ADC_MUX_CTRL_B, i & (1<<1));
    Pin_SetOutput(ADC_MUX_CTRL_C, i & (1<<2));

    DelayUs(100);   //Warten bis Conversion vorbei

    return Adc_GetResult(ADC_CH1); //ADC-Resultat zurückgeben

}

uint16_t AdcToVolt (uint16_t CW){ //Codewort zu Millivolt bzw. Milliampere (durch Shunt und Gain 1:1)

    //return (CW*3256)/(1<<12); //3256 entspricht Vref in mV
    return (CW*7951)/10000; //Selbiges

}

void encoderISR(void){

    static uint32_t CO = 0;
    uint32_t CN = Timer_GetCaptureValue(TIMER_2A), N;

    CN > CO ? (N = CN - CO) : (N = (1<<24)-CO+CN); //Overflow?

    RPM = (int32_t) 240000000 / N; //80MHz * 60 / 2N

    if(Pin_GetInput(ENCODER_B)) RPM = -RPM;

    CO = CN;
    Timer_ClearCaptureFlag(TIMER_2A);


    return;

}

void controlISR(void){

    int16_t P = PiRegler(NSOLL, RPM);
    setDirection(P < 0);
    Timer_SetPwmValue(TIMER_3A, absolut16(P));
    Timer_ClearOverflowFlag(TIMER_1A);
    return;

}


int16_t PiRegler( int16_t nsoll, int16_t nist ) {
    int16_t e, y;
    e = nsoll - nist;
    esum += e;
    e = Limit(e, -2000, 2000);

    y = ((int32_t)e*KP + esum*KI*TA)/SCALE; // Zeitdiskreter PI-Regler


    return Limit(y, -4000, 4000);// Begrenzung fuer y
}

int16_t Limit (int16_t value, int16_t min, int16_t max){

    if(value < min) return min;
    else if(value > max) return max;
    else return value;

}
