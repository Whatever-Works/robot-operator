#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"

#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/PWM.h>
#include "driverlib/adc.h"
#include "utils/uartstdio.h"
#include "Board.h"
#include <math.h>
#include "driverlib/timer.h"

//#include "rom.h"
PWM_Handle motor1;
PWM_Handle motor2;      //global variables
PWM_Params params;
int frontdist;
int sidedist;
int pidcounter = 0;
double LastError = 0;

double findDistanceFront()
{  //PE3
    uint32_t ADCValue[1];
    ADCProcessorTrigger(ADC0_BASE, 3);
    while (!ADCIntStatus(ADC0_BASE, 3, false)) { }
    ADCIntClear(ADC0_BASE, 3);
    ADCSequenceDataGet(ADC0_BASE, 3, ADCValue);
    double distance = 14871 * pow(ADCValue[0], -.995); //this eq converts ADC value to cm
    return distance;
}

double findDistanceSide()
{   //PE2
    uint32_t ADCValue[1];
    ADCProcessorTrigger(ADC0_BASE, 2);
    while (!ADCIntStatus(ADC0_BASE, 2, false)) { }
    ADCIntClear(ADC0_BASE, 2);
    ADCSequenceDataGet(ADC0_BASE, 2, ADCValue);
    double distance = 14871 * pow(ADCValue[0], -.995); //this eq converts ADC value to cm
    return distance;
}

void printDistance()
{
    int i = 0;
    for (i = 0; i < 15; i++)
    {

        UARTprintf("Distance Front[cm] = %d  Distance Side[cm] = %d      \r",
                   findDistanceFront(), findDistanceSide());

        SysCtlDelay(SysCtlClockGet() / 6);
    }
    UARTprintf("\n");

}

void getstring1(char uartstring[])
{
    uartstring[0] = UARTCharGet(UART1_BASE); //UART0 = USB               UART1 = BLUETOOTH
    uartstring[1] = UARTCharGet(UART1_BASE);

}

void ConfigureADC()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);

    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0,
    ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);  //PE3
    ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 0,
    ADC_CTL_CH1 | ADC_CTL_IE | ADC_CTL_END);  //PE2
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCSequenceEnable(ADC0_BASE, 2);
    ADCIntClear(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE, 2);
}

void setCustomSpeedBothMotors()
{
    char mystring[3];
    UARTprintf("Enter speed 0-99\n");
    getstring1(mystring);
    int percentage = ((int) mystring[0] - 48) * 10 + ((int) mystring[1] - 48);
    UARTprintf("both motors set to %d percent\n", percentage);
    leftmotorcustomspeed((double) percentage);
    rightmotorcustomspeed((double) percentage);
}

void leftmotorfast()
{  // this function sets PWM to 100% duty cycle
    PWM_setDuty(motor1, 65535);
    UARTprintf("Left Motor Fast\n");
}

void leftmotorslow()
{  // this function sets PWM to 33% duty cycle
    PWM_setDuty(motor1, 65535 / 3);
    UARTprintf("Left Motor Slow\n");
}

void leftmotorstop()
{  // this function turns duty cycle to zero
    PWM_setDuty(motor1, 0);
    //PWM_close(motor1);
    UARTprintf("Left Motor Stop\n");
}

void leftmotorstart()
{ //enables PWM with a 0% duty cycle. you must call leftmotorfast or leftmotorslow to actually get the motor going.
    motor1 = PWM_open(Board_PWM0, &params);   //PWM0= PWM6 = PF2= Blue LED
    UARTprintf("Left Motor Start \n");
}

void leftmotorcustomspeed(double speed)
{ // ENTER PERCENTAGE 0-100
    PWM_setDuty(motor1, 65535 * (speed / 100));
}

void rightmotorfast()
{  // this function sets PWM to 100% duty cycle
    PWM_setDuty(motor2, 65535);
    UARTprintf("Right Motor Fast\n");
}

void rightmotorcustomspeed(double speed)
{ // ENTER PERCENTAGE 0-100
    PWM_setDuty(motor2, 65535 * (speed / 100));

}

void rightmotorslow()
{  // this function sets PWM to 33% duty cycle
    PWM_setDuty(motor2, 65535 / 3);
    UARTprintf("Right Motor Slow\n");
}

void rightmotorstop()
{ // duty cycle to zero
    PWM_setDuty(motor2, 0);
    //PWM_close(motor2);
    UARTprintf("Right Motor Stop\n");
}

void rightmotorstart()
{ //enables PWM with a 0% duty cycle. you must call rightmotorfast or rightmotorslow to actually get the motor going.
    motor2 = PWM_open(Board_PWM1, &params);   //PWM0= PWM7 = PF3= GREEN LED
    UARTprintf("Right Motor Start\n");
}

struct lut
{
    char funname[2];
    void (*func)(void);
};

struct lut LUT[25] = {
{ "bc", setCustomSpeedBothMotors },
{ "ls", leftmotorstart },
{ "lf", leftmotorfast },
{ "sl", leftmotorslow },
{ "el", leftmotorstop },
{ "rs", rightmotorstart },
{ "rf", rightmotorfast },
{ "sr", rightmotorslow },
{ "er", rightmotorstop },
{ "pd", printDistance }};

void bluetoothSendMessage(char *array);

void ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    //
    // Enable UART1
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);

    //
    // Configure GPIO Pins for UART mode.
    //
    GPIOPinConfigure(GPIO_PC4_U1RX);                               //PC4RX PC5TX
    GPIOPinConfigure(GPIO_PC5_U1TX);
    GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART1_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(1, 9600, 16000000);
}

void btUART()
{
    char uartstring[3] = "";
    int i = 0;
    void (*fun)(void);
    while (1)
    {
        getstring1(uartstring); //pulls 2 character string from UART
        UARTprintf("your input: %s\n", uartstring);
        for (i = 0; i < 25; i++)
        {
            if (strcmp(uartstring, LUT[i].funname) == 0) //IF UART STRING ==  Function abbreviation. then set function pointer to respective function.
            {
                fun = LUT[i].func;
                fun();
                break;
            }
        }
        if (i == 25) UARTprintf("Invalid input!\n");
    }
}

void pid()
{
    //PID CALCULATIONS
    double frontdistance = findDistanceFront();
    double sidedistance = findDistanceSide();
    double Error = (sidedistance - 10);
    double P = 15 * fabs(Error);
    double I = 1.5 * (Error + LastError);
    double D = 1 * (Error - LastError);
    double PID = (P + I + D);
    if (PID > 98)
        PID = 98; // dont let PID value go over 98

    if (Error > 0)
    {                       //ROBOT IS VEERING LEFT        SLOW DOWN RIGHT MOTOR
        rightmotorcustomspeed(99 - PID);
        leftmotorcustomspeed(99);
    }
    if (Error < 0)
    {                       // ROBOT IS VEERING RIGHT       SLOW DOWN LEFT MOTOR
        leftmotorcustomspeed(99 - PID);
        rightmotorcustomspeed(99);
    }

    LastError = Error;

    if (frontdistance < 8)                       //IF FRONTDISTANCE <8 cm U TURN
    {

        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7); //SET LEFT MOTOR TO REVERSE
        leftmotorcustomspeed(99);
        rightmotorcustomspeed(99);
        while (findDistanceFront() < 16)
            ; //turn until front distance >16
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0); //set motor back to forward
    }
    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
}

void ConfigureTimer2A()
{
    // Timer 2 setup code           50 MS
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);  // enable Timer 2 periph clks
    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC); // cfg Timer 2 mode - periodic

    uint32_t ui32Period = (SysCtlClockGet() / 20); // period = 1/20th of a second AKA 50MS
    TimerLoadSet(TIMER2_BASE, TIMER_A, ui32Period);        // set Timer 2 period

    TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT); // enables Timer 2 to interrupt CPU

    TimerEnable(TIMER2_BASE, TIMER_A);                      // enable Timer 2
}

void ConfigureTimer1A()
{
    // Timer 1 setup code           10 MS
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);  // enable Timer 1 periph clks
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC); // cfg Timer 1 mode - periodic

    uint32_t ui32Period = (SysCtlClockGet() / 100); // period = 1/100th of a second AKA 10MS
    TimerLoadSet(TIMER1_BASE, TIMER_A, ui32Period);        // set Timer 1 period

    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT); // enables Timer 2 to interrupt CPU

    TimerEnable(TIMER1_BASE, TIMER_A);                      // enable Timer 2
}

int reflectance()
{
    int cycles = 0;

    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_6);  //set output
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6); //set high... starts charging capacitor
    SysCtlDelay(SysCtlClockGet() / 80000); //wait for capacitor to charge 12.5us
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_6);            //set input
    while (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6) // while capacitor is not discharged..... finds how many [us] it takes to discharge capacitor
    {
        SysCtlDelay(SysCtlClockGet() / 1000000);      //delay 1 us
        cycles++;
    }
    if (cycles > 250)
        return 1;      //UARTprintf("Black\n",cycles);
    else
        return 0;      //UARTprintf("White\n");
}

int numThinLines = 0;

void blacklineinterrupt()  //CALLED EVERY 10 MS
{
    int cycles = 0;
    while (reflectance() == 1)       //WHILE SENSOR READING BLACK
    {
        cycles++;
    }
    if (cycles > 15)            //IF MORE THAN 15 CYCLES IT MUST BE THICCCCCC
    {
        // STOP
        UARTprintf("Large strip hit numCycles:%d\n", cycles);
    }
    if ((cycles > 3) && (cycles < 16))
    {                     //IF LESS THAN 16 cycles must be thin line
        numThinLines++;
        UARTprintf("Thin line #%d hit   numCycles:%d \n", numThinLines, cycles);
    }
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
}

int main(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
    Board_initGeneral();
    Board_initGPIO();
    Board_initPWM();
    ConfigureUART();
    ConfigureADC();
    ConfigureTimer2A();
    ConfigureTimer1A();
    UARTprintf("Hello World!\n");
    PWM_Params_init(&params);
    params.dutyMode = PWM_DUTY_SCALAR; //0=0%duty cycle 65535=100% duty cycle
    sidedist = findDistanceSide(); //initial side distance for pid calc
    leftmotorstart();
    rightmotorstart();
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_7);

    BIOS_start();

    //UARTDisable(UART0_BASE);
}

