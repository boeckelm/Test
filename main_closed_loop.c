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
#define PWM_MAX 4000


int16_t linearMap (int16_t value, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max);
uint16_t absolut16 (int16_t val);
void setDirection(uint8_t val);                                             //Sollte aus ISR aufrufbar sein, hier irrelevent
uint16_t Adc_Convert( uint8_t i);
uint16_t AdcToVolt (uint16_t CW);                                           //Funktion CW->Spannung/Strom
int16_t PiRegler( int16_t nsoll, int16_t nist );                            //PI-Regler für ISR
int32_t Limit (int32_t value, int32_t min, int32_t max);                    //Limit-Funktion

void encoderISR(void); //Für Encoder -> RPM Bestimmung
void controlISR(void); //Für Regelkreis


// Folgende Studierende haben den Versuch zusammen erstellt und abgegeben:
char Namen[] = "Michael Boeckelen, Tim Gebhard";

int16_t RPM = 0, NSOLL = 0; //Globale Variable für RPM
int32_t esum = 0;  // globale Variable, Summe des Regelfehlers


void main(void) {
    System_Init();
    Display_Init(); //Display initialisieren

    //DIR-Outputs
    Pin_Init(DIR_0, PIN_GPIO_OUT); //DIR
    Pin_Init(DIR_1, PIN_GPIO_OUT);
    Pin_SetOutput(DIR_0, 1); //Am Anfang lieber mal ausschalten bevor undefine state auftritt
    Pin_SetOutput(DIR_1, 1);

    //ADC-Config
    Pin_Init(ADC_MUX_CTRL_A, PIN_GPIO_OUT); //Pins zur Ansteuerung des ADC-Muxers konfigurieren (Output)
    Pin_Init(ADC_MUX_CTRL_B, PIN_GPIO_OUT);
    Pin_Init(ADC_MUX_CTRL_C, PIN_GPIO_OUT);
    Pin_Init(ADC_CH1_IN_PIN, PIN_GPIO_IN);   //ADC_CH1_IN_PIN als Eingang
    Adc_Init();

    //PWM für Motor
    Timer_InitPwm(TIMER_3A, PWM_MAX - 1); //PWM für Motor, 20kHz
    Pin_Init(PWM_MOTOR, PIN_TIMER_OUT);

    //Timer für diskreten Regler
    Timer_Init(TIMER_1A, 799999); //Regler-Interrupt, 100Hz, M = 799.999
    Timer_EnableOverflowIrq(TIMER_1A); //Overflow-IRQ aktivieren
    Timer_Start(TIMER_1A);  //Timer starten
    Isr_Register(IRQ_TIMER_1A, controlISR); //ISR registrieren

    //Capture für Motordrehzahl
    Timer_InitCapture(TIMER_2A); //Modul aktivieren
    Pin_Init(ENCODER_A, PIN_TIMER_IN);  //Eingang definieren
    Timer_EnableCaptureIrq(TIMER_2A);   //IRQ wenn Rising Edge
    Isr_Register(IRQ_TIMER_2A, encoderISR); //ISR regstrieren
    Pin_Init(ENCODER_B, PIN_GPIO_IN);   //Drehrichtung

    //ISRs aktiviern
    Isr_EnableAll();

    //Für Visualisierung
    Uart_Init(UART_0, 115200);

    //Zwischenspeicher
    uint16_t CWPoti, IL, IR;    //Variablen zum Zwischenspeichern von Werten

       while(1) {

        CWPoti = Adc_Convert(ADC_MUX_IN_POTI);  //Analogwert von Poti auslesen
        IL = AdcToVolt(Adc_Convert(ADC_MUX_IN_IL));
        IR = AdcToVolt(Adc_Convert(ADC_MUX_IN_IR));

        NSOLL = linearMap((int16_t)CWPoti, 0, 4095, -6000, 6000);//CWPoti [0;4095] auf das gewünschte Format mappen [-6000; 6000]

        Display_Printf(0, 0, "REGELUNG");
        Display_Printf(1, 0, "CWPoti %04d", CWPoti);
        Display_Printf(2, 0, "IL: %04d", CWIL);
        Display_Printf(3, 0, "IR: %04d", CWIR);
        Display_Printf(4, 0, "nsoll: %05d RPM", NSOLL);
        Display_Printf(5, 0, "nist:  %05d RPM", RPM);

        printf("%d,%d,%d,%d\r\n", NSOLL, RPM, IL, IR); //Für Plot
        DelayMs(100);
        Display_Update();

    }
}

int16_t linearMap (int16_t value, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max){

    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; //Lineares Erweitern auf gewünschtes Format

}


uint16_t absolut16 (int16_t val){

    if(val < 0) return -val;
    else return val;

}

void setDirection(uint8_t val){ //0: vorwärts, im Uhrzeigersinn, 1: rückwärts

    Pin_SetOutput(DIR_0, val); //0000(!VAL)(VAL)00
    Pin_SetOutput(DIR_1, !val);

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

    RPM = (int32_t) 1600000000 / N; //80MHz * 60 / 2N

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
    static int16_t lim = 4000*SCALE/(KI*TA);
    int16_t e, y;
    e = nsoll - nist;
    esum += e;
    e = Limit(e, -lim, lim);

    y = ((int32_t)e*KP + esum*KI*TA)/SCALE; // Zeitdiskreter PI-Regler


    return Limit(y, -PWM_MAX, PWM_MAX );// Begrenzung fuer y
}

int32_t Limit (int32_t value, int32_t min, int32_t max){

    if(value < min) return min;
    else if(value > max) return max;
    else return value;

}
