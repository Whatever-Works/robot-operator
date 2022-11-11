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
extern void ConfigureADC();
extern void ConfigureTimer1A();
extern void ConfigureTimer2A();
extern void ConfigureUART();
extern void GPIOConfig();
extern void PWMConfig();
extern const Swi_Handle swi0;
extern const ti_sysbios_knl_Semaphore_Handle Semaphore0;
extern const ti_sysbios_knl_Semaphore_Handle Semaphore1;
extern const ti_sysbios_knl_Semaphore_Handle Semaphore2;
extern const ti_sysbios_knl_Semaphore_Handle Semaphore3;
PWM_Handle motor1;
PWM_Handle motor2;                                       //global variables
extern PWM_Params params;
double LastError = 0;
int numThinLines = 0; //keep track of how many thin black lines have been passed

int findDistanceFront() //return distance from front sensor as ADC VALUE
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

int findDistanceSide()  // return distance from side sensor as ADC VALUE
{   //PE2
    uint32_t ADCValue[1];
    ADCProcessorTrigger(ADC0_BASE, 2);
    while (!ADCIntStatus(ADC0_BASE, 2, false))
    {
    }
    ADCIntClear(ADC0_BASE, 2);
    ADCSequenceDataGet(ADC0_BASE, 2, ADCValue);
    //double distance = 14871 * pow(ADCValue[0], -.995); //this eq converts ADC value to cm
    int distance = ADCValue[0];
    return distance;
}

void leftmotorstop()   // this function turns duty cycle to zero
{
    PWM_setDuty(motor1, 0);
    UARTprintf("Left Motor Stop\n");
}

void leftmotorstart() //enables PWM with a 0% duty cycle. you must call leftmotorfast or leftmotorslow to actually get the motor going.
{
    motor1 = PWM_open(Board_PWM0, &params);   //PWM0= PB6
    UARTprintf("Left Motor Start \n");
}

void leftmotorcustomspeed(double speed) //sets motor speed 0-99.99999 via PWM duty cycle
{
    // TODO: Add conditional logic to avoid inputting any invalid values to PWN_setDuty
    PWM_setDuty(motor1, 65535 * (speed / 100));
}

void rightmotorcustomspeed(double speed) //sets motor speed 0-99.99999 via PWM duty cycle
{
    // TODO: Add conditional logic to avoid inputting any invalid values to PWN_setDuty
    PWM_setDuty(motor2, 65535 * (speed / 100));
}

void rightmotorstop()   //This function sets duty cycle to zero
{
    PWM_setDuty(motor2, 0);
    UARTprintf("Right Motor Stop\n");
}

void rightmotorstart() //enables PWM with a 0% duty cycle. you must call rightmotorfast or rightmotorslow to actually get the motor going.
{
    motor2 = PWM_open(Board_PWM1, &params);   //PWM1= PB7
    UARTprintf("Right Motor Start\n");
}

void PIDInterruptHandler()   // called every 50 ms
{

    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    Semaphore_post(Semaphore0); // this is gonna start pid task

}

int uturning = 0;

void UTURN()
{
    while (1)
    {
        Semaphore_pend(Semaphore3, BIOS_WAIT_FOREVER);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7); //SET LEFT MOTOR TO REVERSE
        leftmotorcustomspeed(99);                          //U-TURN AT MAX SPEED
        rightmotorcustomspeed(99);                         //U-TURN AT MAX SPEED
        while (14871 * pow(findDistanceFront(), -.995) < 16);  //turn until front distance >16 cm
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0); //set motor back to forward
        uturning = 0;
    }
}

int pidCycles = 0;
int printHelper = 0;
int ping[20];
int pong[20];
int *currentbuffer = ping;

void switchBuffer()          //toggle ping/pong buffer
{
    if (currentbuffer == ping)
    {
        currentbuffer = pong;
    }
    else
    {
        currentbuffer = ping;
    }
}

void pingPongSWIHandler()          //CALLED WHEN BUFFER IS FULL
{
    Semaphore_post(Semaphore2);
}
void pingPongTask()
{
    // int temp = pidCycles;
    while (1)
    {
        Semaphore_pend(Semaphore2, BIOS_WAIT_FOREVER);
        int count;
        if(printHelper % 40 == 0) {
            count = 20;
        } else {
            count = printHelper % 40 / 2;
        }
        int i; //TODO: Are we not able to declare i inside the for-loop header?
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);    //turn off blue
        for (i = 0; i < count; i++)
        {
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);    //turn on green
            UARTprintf("%d [ms]  : %d [ADC Value]\n",
                       ((printHelper - 40 + 2 * i) * 50), currentbuffer[i]); //print buffer
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);    //turn off green
        }
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);    //turn on blue
        switchBuffer();  //switch from ping to pong buffer or pong to ping
    }

}


void pidTask()  //PID for following right wall. also handles U-TURNS.
{
    while (1)
    {
        Semaphore_pend(Semaphore0, BIOS_WAIT_FOREVER); //task unblocked every 50 [ms] by timer
        int frontdistanceADC = findDistanceFront();
        int sidedistanceADC = findDistanceSide();
        double frontdistance = 14871 * pow(frontdistanceADC, -.995); //convert to cm
        double sidedistance = 14871 * pow(sidedistanceADC, -.995); //convert to cm
        int ADCValueError = sidedistanceADC - 1542;

        if (uturning == 0)
        {
            double Error = (sidedistance - 10); //we want robot to ride 10 cm from wall
            double P = 15 * fabs(Error);
            double I = 1.5 * (Error + LastError);
            double D = 1 * (Error - LastError);
            double PID = (P + I + D);
            if (PID > 98)
                PID = 98; // dont let PID value go over 98

            if (Error > 0) //ROBOT IS VEERING LEFT        SLOW DOWN RIGHT MOTOR
            {
                rightmotorcustomspeed(99 - PID);
                leftmotorcustomspeed(99);
            }
            if (Error < 0) // ROBOT IS VEERING RIGHT       SLOW DOWN LEFT MOTOR
            {
                leftmotorcustomspeed(99 - PID);
                rightmotorcustomspeed(99);
            }
            LastError = Error;  //Set LastError  for I AND D CALCULATIONS
        }



        if (frontdistance < 8) {
            uturning = 1;
            Semaphore_post(Semaphore3);    //IF FRONTDISTANCE <8 cm U TURN
        }

        if (numThinLines == 2) //after second thin line stop collecting and printing data also set numthinlines to 999 so it wont keep re-enter this if statement. print remai
        {
            printHelper = pidCycles;
            int i;
            for (i = 0; i < printHelper % 40 / 2; i++)
            {
                UARTprintf("%d [ms]  : %d [ADC Value]\n",
                           ((printHelper - (printHelper % 40 / 2) + 2 * i) * 50), currentbuffer[i]); //PRINT ANYTHING REMAINING IN BUFFER SO NO DATA IS LOST
            }
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);       //turn off blue
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);       //turn off green
            numThinLines = 999; //do this so it doesn't repeat itself every 10ms
        }

        if ((pidCycles % 2 == 0) && (numThinLines == 1)) //every 100[ms] (every other PID cycle) && after first thin line and before second thin line
        {
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
            currentbuffer[pidCycles % 40 / 2] = ADCValueError;
            if ((pidCycles % 40 == 0) && (pidCycles != 0)) //every 40 PID cycles 2[s] we call SWI to print buffer.
            {
                printHelper = pidCycles;
                Swi_post(swi0); //SWI
            }
        }
        if (numThinLines == 1)
            pidCycles++;
    }
}

int reflectance() //This function tells you if surface is black or white        returns 1 for black 0 for white.
{
    int cycles = 0;
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_6);            //set output
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6); //set high... starts charging capacitor
    SysCtlDelay(SysCtlClockGet() / 80000); //wait for capacitor to charge 12.5us
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_6); //set input... capacitor starts discharging
    while (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6) // while capacitor is not discharged..... finds how many [us] it takes to discharge capacitor
    {
        SysCtlDelay(SysCtlClockGet() / 1000000);      //delay 1 us
        cycles++;
    }
    if (cycles > 250)
    {
        return 1;      //UARTprintf("Black\n",cycles);
    }
    else
    {
        return 0; //UARTprintf("White\n");
    }
}

int counter = 0;

void BlackLineInterruptHandler() // called every 10 ms
{
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);     // reset 10 ms timer
    counter++; //THIS WILL BE USED TO FIND TOTAL RUN TIME IN MAZE. INCREMENTED EVERY 10MS THE ROBOT RUNS.
    Semaphore_post(Semaphore1);
    // this is gonna start blackline task
}

int currentreflectance;
int lastreflectance;
void blacklineTask()    //detects if robot drives over thin or thick black line.
{
    while (1)
    {
        Semaphore_pend(Semaphore1, BIOS_WAIT_FOREVER); //task unblocked every 10 [ms] by timer
        int cycles = 0;
        while ((currentreflectance = reflectance() == 1) || (lastreflectance == 1)) //WHILE SENSOR READING BLACK. allows one cycle of white incase of error in reading. sometimes the sensor would read one white in the middle of a black line and mess everything up. so this disregards one white reading in the middle of a black line.
        {
            cycles++;
            lastreflectance = currentreflectance;
        }
        if (cycles > 20)    //IF MORE THAN 15 CYCLES IT MUST BE THICC
        {
            UARTprintf("Large strip hit numCycles:%d\n", cycles);
            UARTprintf("Total Run Time: %d seconds\n", counter / 100);
            leftmotorstop();
            rightmotorstop();
            BIOS_exit(0);
        }
        if ((cycles > 3) && (cycles < 20)) //IF LESS THAN 22 cycles must be thin line
        {
            numThinLines++;
            UARTprintf("Thin line #%d hit   numCycles:%d \n", numThinLines,
                       cycles);
        }
    }
}

int main(void)
{
    SysCtlClockSet(
    SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN); //   LeftMotorPolarity: PA7
    PWMConfig();
    GPIOConfig(); //                                                    LeftMotor        : PB6     RightMotor: PB7
    ConfigureUART(); // Configure UART for BT                              RX               : PC4     TX        : PC5
    ConfigureADC(); // Configure ADC for distance sensors                 FrontSensor      : PE3     SideSensor: PE2
    ConfigureTimer2A();     // 50ms timer for pid
    ConfigureTimer1A();     // 10ms timer for blackline
    UARTprintf("Hello World!\n");
    LastError = findDistanceSide() - 10;       //Get initial Error for PID calc.
    leftmotorstart();               //Configures left motor. starts with 0% duty
    rightmotorstart();            // Configures right motor. starts with 0% duty
    BIOS_start();
    //UARTprintf("Total Run Time: %d seconds\n",counter/20);
    //UARTDisable(UART0_BASE);
}

