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
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Swi.h>
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
extern const Swi_Handle swi0;
extern const ti_sysbios_knl_Semaphore_Handle Semaphore0;
extern const ti_sysbios_knl_Semaphore_Handle Semaphore1;
PWM_Handle motor1;
PWM_Handle motor2;                                       //global variables
PWM_Params params;
double LastError = 0;
int numThinLines=0;    //keep track of how many thin black lines have been passed

int findDistanceFront() //return distance from front sensor as [CM]
{  //PE3
    uint32_t ADCValue[1];
    ADCProcessorTrigger(ADC0_BASE, 3);
    while (!ADCIntStatus(ADC0_BASE, 3, false))
    {
    }
    ADCIntClear(ADC0_BASE, 3);
    ADCSequenceDataGet(ADC0_BASE, 3, ADCValue);
    //double distance = 14871 * pow(ADCValue[0], -.995); //this eq converts ADC value to cm
    int distance = ADCValue[0];
    return distance;
}

int findDistanceSide()// return distance from side sensor as [cm]
{   //PE2
    uint32_t ADCValue[1];
    ADCProcessorTrigger(ADC0_BASE, 2);
    while (!ADCIntStatus(ADC0_BASE, 2, false))
    {
    }
    ADCIntClear(ADC0_BASE, 2);
    ADCSequenceDataGet(ADC0_BASE, 2, ADCValue);
    //double distance = 14871 * pow(ADCValue[0], -.995); //this eq converts ADC value to cm
    int distance=ADCValue[0];
    return distance;

}
void printDistance()
{
    int i = 0;
    for (i = 0; i < 15; i++)
    {

        UARTprintf("Distance Front[cm] = %d  Distance Side[cm] = %d      \r", 14871 * pow(findDistanceFront(), -.995), 14871 * pow(findDistanceSide(), -.995));// print as int... for some reason printing a double on bluetooth gives error.
        SysCtlDelay(SysCtlClockGet() / 6);
    }
    UARTprintf("\n");

}
void getstring1(char uartstring[])        //used to grab a 2 character string from UART
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
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);  //PE3
    ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_CH1 | ADC_CTL_IE | ADC_CTL_END);  //PE2
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCSequenceEnable(ADC0_BASE, 2);
    ADCIntClear(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE, 2);
}

void leftmotorfast() // this function sets PWM to 100% duty cycle
{
    PWM_setDuty(motor1, 65535);
    UARTprintf("Left Motor Fast\n");
}
void leftmotorslow()// this function sets PWM to 33% duty cycle
{
    PWM_setDuty(motor1, 65535 / 3);
    UARTprintf("Left Motor Slow\n");
}
void leftmotorstop()// this function turns duty cycle to zero
{
    PWM_setDuty(motor1, 0);
    UARTprintf("Left Motor Stop\n");
}
void leftmotorstart()//enables PWM with a 0% duty cycle. you must call leftmotorfast or leftmotorslow to actually get the motor going.
{
    motor1 = PWM_open(Board_PWM0, &params);   //PWM0= PB6
    UARTprintf("Left Motor Start \n");
}
leftmotorcustomspeed(double speed)//sets motor speed 0-99.99999 via PWM duty cycle
{
    PWM_setDuty(motor1, 65535 * (speed / 100));

}

void rightmotorfast()// this function sets PWM to 100% duty cycle
{
    PWM_setDuty(motor2, 65535);
    UARTprintf("Right Motor Fast\n");
}
rightmotorcustomspeed(double speed)//sets motor speed 0-99.99999 via PWM duty cycle
{
    PWM_setDuty(motor2, 65535 * (speed / 100));

}
void setCustomSpeedBothMotors()// ask user to enter speed for both motors over UART
{
    char mystring[3];
    UARTprintf("Enter speed 0-99\n");
    getstring1(mystring);
    int percentage = ((int) mystring[0] - 48) * 10 + ((int) mystring[1] - 48); //convert 2 char string to an integer
    UARTprintf("both motors set to %d percent\n", percentage);
    leftmotorcustomspeed((double) percentage);
    rightmotorcustomspeed((double) percentage);

}
void rightmotorslow()// this function sets PWM to 33% duty cycle
{
    PWM_setDuty(motor2, 65535 / 3);
    UARTprintf("Right Motor Slow\n");
}
void rightmotorstop()//This function sets duty cycle to zero
{
    PWM_setDuty(motor2, 0);
    UARTprintf("Right Motor Stop\n");
}
void rightmotorstart()//enables PWM with a 0% duty cycle. you must call rightmotorfast or rightmotorslow to actually get the motor going.
{
    motor2 = PWM_open(Board_PWM1, &params);   //PWM1= PB7
    UARTprintf("Right Motor Start\n");
}




struct lut
{
    char funname[2];
    void (*func)(void);
};

struct lut LUT[25] = {
                       {"bc", setCustomSpeedBothMotors},
                       {"ls", leftmotorstart},
                       {"lf", leftmotorfast},
                       {"sl", leftmotorslow},
                       {"el", leftmotorstop},
                       {"rs", rightmotorstart},
                       {"rf", rightmotorfast},
                       {"sr", rightmotorslow},
                       {"er", rightmotorstop},
                       {"pd", printDistance}
                     };
PWMConfig()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
        GPIOPinConfigure(GPIO_PB6_M0PWM0 );
        GPIOPinConfigure(GPIO_PB7_M0PWM1);
        GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6 |GPIO_PIN_7);
        PWM_init();
        PWM_Params_init(&params);
               params.dutyMode = PWM_DUTY_SCALAR;             //sets mode of duty cycle to  0 = 0%duty cycle 65535 = 100% duty cycle
}

ConfigureUART(void)
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

void btUART()    //asks users for 2 char function from lookup table. only used for debugging. not used in final.
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
void PIDInterruptHandler()// called every 50 ms
{

    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    Semaphore_post(Semaphore0); // this is gonna start pid task

}

void UTURN()
{
   GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7); //SET LEFT MOTOR TO REVERSE
   leftmotorcustomspeed(99);                              //U-TURN AT MAX SPEED
   rightmotorcustomspeed(99);                             //U-TURN AT MAX SPEED
   while (14871 * pow(findDistanceFront(), -.995) < 16);                      //turn until front distance >16 cm
   GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0);          //set motor back to forward
        }
int pidCycles=0;
int ping[20];
int pong[20];
int*  currentbuffer=ping;
void switchBuffer()//toggle ping/pong buffer
{
    if (currentbuffer==ping)currentbuffer=pong;
    else currentbuffer=ping;
}


void pingPongSWIHandler()//CALLED WHEN BUFFER IS FULL
{
    int i;
    for( i=0;i<20;i++)UARTprintf("%d [ms]  : %d [ADC Value]\n",((pidCycles-40+2*i)*50),currentbuffer[i]);//print buffer
    switchBuffer();  //switch from ping to pong buffer or pong to ping
}








void pidTask()  //PID for following right wall. also handles U-TURNS.
{
while(1){

    Semaphore_pend(Semaphore0, BIOS_WAIT_FOREVER);  //task unblocked every 50 [ms] by timer

    int frontdistanceADC = findDistanceFront();
    int sidedistanceADC = findDistanceSide();
    double frontdistance=14871 * pow(frontdistanceADC, -.995); //convert to cm
    double sidedistance=14871 * pow(sidedistanceADC, -.995);//convert to cm
    int ADCValueError = sidedistanceADC-1542;
    double Error = (sidedistance - 10); //we want robot to ride 10 cm from wall
    double P = 15 * fabs(Error);
    double I = 1.5 * (Error + LastError);
    double D = 1 * (Error - LastError);
    double PID = (P + I + D);
    if (PID > 98) PID = 98; // dont let PID value go over 98

    if (Error > 0) //ROBOT IS VEERING LEFT        SLOW DOWN RIGHT MOTOR
    {
        rightmotorcustomspeed(99 - PID);
        leftmotorcustomspeed(99);
    }
    if (Error < 0)// ROBOT IS VEERING RIGHT       SLOW DOWN LEFT MOTOR
    {
        leftmotorcustomspeed(99 - PID);
        rightmotorcustomspeed(99);
    }

    LastError = Error;  //Set LastError  for I AND D CALCULATIONS

    if (frontdistance < 8) UTURN();    //IF FRONTDISTANCE <8 cm U TURN
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);//turn off green

     if(numThinLines==2)//after second thin line stop collecting and printing data also set numthinlines to 999 so it wont keep re-enter this if statement. print remai
        {
         int i;
         for( i=0;i<pidCycles%40/2;i++)UARTprintf("%d [ms]  : %d [ADC Value]\n",((pidCycles-40+2*i)*50),currentbuffer[i]);        //PRINT ANYTHING REMAINING IN BUFFER SO NO DATA IS LOST
         GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);//turn off blue
                  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);//turn off green
                  numThinLines=999;//do this so it doesnt repeat itself every 10ms
        }



    if((pidCycles%2==0)&&(numThinLines==1)) //every 100[ms] (every other PID cycle) && after first thin line and before second thin line
    {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
        currentbuffer[pidCycles%40/2]=ADCValueError;
        if((pidCycles%40==0)&&(pidCycles!=0)) //every 40 PID cycles 2[s] we call SWI to print buffer.
        {
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);//blink green
            Swi_post(swi0);//SWI

        }

        }
    if(numThinLines==1)pidCycles++;
}}

void ConfigureTimer2A()// Timer 2 setup code      this calls PIDTaskHandler every 50 MS
{

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);            // enable Timer 2 periph clks
    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);         // cfg Timer 2 mode - periodic
    uint32_t ui32Period = (SysCtlClockGet() / 20);           // period = 1/20th of a second AKA 50MS
    TimerLoadSet(TIMER2_BASE, TIMER_A, ui32Period);          // set Timer 2 period
    TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);         // enables Timer 2 to interrupt CPU
    TimerEnable(TIMER2_BASE, TIMER_A);                       // enable Timer 2
}

void ConfigureTimer1A()                              // Timer 1 setup code           10 MS   used for calling blacklineInterrupt
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);    // enable Timer 1 periph clks
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC); // cfg Timer 1 mode - periodic
    uint32_t ui32Period = (SysCtlClockGet() / 100);  // period = 1/100th of a second AKA 10MS
    TimerLoadSet(TIMER1_BASE, TIMER_A, ui32Period);  // set Timer 1 period

    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT); // enables Timer 2 to interrupt CPU

    TimerEnable(TIMER1_BASE, TIMER_A);                      // enable Timer 2
}

int reflectance()       //This function tells you if surface is black or white        returns 1 for black 0 for white.
{
    int cycles = 0;

    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_6);            //set output
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);         //set high... starts charging capacitor
    SysCtlDelay(SysCtlClockGet() / 80000);                         //wait for capacitor to charge 12.5us
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_6);             //set input... capacitor starts discharging
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
int counter=0;
void BlackLineInterruptHandler() // called every 10 ms
{
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);     // reset 10 ms timer
    counter++;                                          //THIS WILL BE USED TO FIND TOTAL RUN TIME IN MAZE. INCREMENTED EVERY 10MS THE ROBOT RUNS.
    Semaphore_post(Semaphore1);
    // this is gonna start blackline task
}
int currentreflectance;
int lastreflectance;
void blacklineTask()       //detects if robot drives over thin or thick black line.
{
    while(1){
        Semaphore_pend(Semaphore1, BIOS_WAIT_FOREVER); //task unblocked every 10 [ms] by timer
    int cycles = 0;
    while ((currentreflectance=reflectance() == 1)||(lastreflectance==1))       //WHILE SENSOR READING BLACK. allows one cycle of white incase of error in reading. sometimes the sensor would read one white in the middle of a black line and mess everything up. so this disregards one white reading in the middle of a black line.
    {
        cycles++;
        lastreflectance=currentreflectance;
    }
    if (cycles > 16)            //IF MORE THAN 15 CYCLES IT MUST BE THICC
    {
        //
        UARTprintf("Large strip hit numCycles:%d\n",cycles);
        UARTprintf("Total Run Time: %d seconds\n",counter/100);
        leftmotorstop();
        rightmotorstop();
        BIOS_exit(0);
    }
    if((cycles>3)&&(cycles<17))//IF LESS THAN 22 cycles must be thin line
    {

        numThinLines++;
        UARTprintf("Thin line #%d hit   numCycles:%d \n",numThinLines,cycles);
    }


    }
}

void GPIOConfig(){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);       // enable port A for left motor polarity, and reflectance sensor
        GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_7);//THIS IS USED FOR LEFT MOTOR POLARITY LO=forward HI=reverse
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
        GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);//red led
        GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);//blue led
        GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);//green led
}

int main(void)
{

    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
   // Board_initGeneral();       //                                                               ReflectanceSensor: PA6
   // Board_initGPIO();          //                                                               LeftMotorPolarity: PA7
   PWMConfig();
   GPIOConfig();
    //Board_initPWM();           // Configure PWM                                               LeftMotor        : PB6     RightMotor: PB7
    ConfigureUART();           //Configure UART for BT                                          RX               : PC4     TX        : PC5
    ConfigureADC();            //Configure ADC for distance sensors                             FrontSensor      : PE3     SideSensor: PE2
    ConfigureTimer2A();        // 50ms timer for pid
    ConfigureTimer1A();        //10ms timer for blackline
    UARTprintf("Hello World!\n");
    LastError = findDistanceSide()-10;                 //Get initial Error for PID calc.
    leftmotorstart();                                  //Configures left motor. starts with 0% duty
    rightmotorstart();                                 // Configures right motor. starts with 0% duty
    BIOS_start();
//UARTprintf("Total Run Time: %d seconds\n",counter/20);
    //UARTDisable(UART0_BASE);
}

