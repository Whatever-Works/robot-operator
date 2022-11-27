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
PWM_Handle motor2;                                       // global variables
extern PWM_Params params;
double LastError = 0;
int numThinLines = 0; // keep track of how many thin black lines have been passed

int findDistanceFront() // return distance from front sensor as ADC VALUE
{  // PE3
    uint32_t ADCValue[1];
    ADCProcessorTrigger(ADC0_BASE, 3);
    while (!ADCIntStatus(ADC0_BASE, 3, false)) { }
    ADCIntClear(ADC0_BASE, 3);
    ADCSequenceDataGet(ADC0_BASE, 3, ADCValue);
    int distance = ADCValue[0];
    return distance;
}

int findDistanceSide()  // return distance from side sensor as ADC VALUE
{   // Utilizes PE2
    uint32_t ADCValue[1];
    ADCProcessorTrigger(ADC0_BASE, 2);
    while (!ADCIntStatus(ADC0_BASE, 2, false)) { }
    ADCIntClear(ADC0_BASE, 2);
    ADCSequenceDataGet(ADC0_BASE, 2, ADCValue);
    int distance = ADCValue[0];
    return distance;
}

void leftmotorstop()   // this function turns duty cycle to zero
{
    PWM_setDuty(motor1, 0);
}

void leftmotorstart() //enables PWM with a 0% duty cycle. you must call leftmotorfast or leftmotorslow to actually get the motor going.
{
    motor1 = PWM_open(Board_PWM0, &params);   // PWM0 = PB6
}

void leftmotorcustomspeed(double speed) //sets motor speed 0-99.99999 via PWM duty cycle
{
    if(speed > 99.99999) {
        speed = 99.99999;
    }
    if(speed < 0) {
        speed = 0;
    }
    PWM_setDuty(motor1, 65535 * (speed / 100));
}

void rightmotorcustomspeed(double speed) //sets motor speed 0-99.99999 via PWM duty cycle
{
    if(speed > 99.99999) {
        speed = 99.99999;
    }
    if(speed < 0) {
        speed = 0;
    }
    PWM_setDuty(motor2, 65535 * (speed / 100));
}

void rightmotorstop()   // This function sets duty cycle to zero
{
    PWM_setDuty(motor2, 0);
}

void rightmotorstart() // Enables PWM with a 0% duty cycle. You must call rightmotorfast or rightmotorslow to actually get the motor going.
{
    motor2 = PWM_open(Board_PWM1, &params);   // PWM1 = PB7
}

void PIDInterruptHandler()   // called every 50 [ms]
{

    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    Semaphore_post(Semaphore0); // this will start the PID task

}

int uturning = 0;

void UTURN()
{
    while (1)
    {
        Semaphore_pend(Semaphore3, BIOS_WAIT_FOREVER);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7);  //Set left motor direction to reverse.
        leftmotorcustomspeed(99); //U-TURN AT MAX SPEED
        rightmotorcustomspeed(99); //U-TURN AT MAX SPEED
        while (14871 * pow(findDistanceFront(), -.995) < 16);   // turn until front distance > 16 [cm]
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0); // set left motor direction back to forwards
        uturning = 0; // no longer u-turning, so we set the uturning flag to 0.
    }
}

int pidCycles = 0;
int printHelper = 0;
char ping[40];
char pong[40];
char *currentBuffer = ping;
char *lastBuffer = pong;
int bufferIndex = 0;
int lastItemIndex = 0;

void switchBuffer() // toggle current buffer for data recording and reset lastItemIndex
{
    if (currentBuffer == ping)
    {
        lastBuffer = ping;
        currentBuffer = pong;
    }
    else
    {
        lastBuffer = pong;
        currentBuffer = ping;
    }
    lastItemIndex = bufferIndex - 1;
    bufferIndex = 0;
}

void clearLastBuffer() {
    int i = 0;
    for (i = 0; i < 40; i++) {
        lastBuffer[i] = '\0';   // clear buffer
    }
}

void pingPongSWIHandler()   // Called when buffer is full and ready to print
{
    Semaphore_post(Semaphore2);
}

void pingPongTask()
{
    while (1)
    {
        Semaphore_pend(Semaphore2, BIOS_WAIT_FOREVER);
        int i = 0;
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0); // turn off blue led so we can see green led
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3); // turn on green led
        while (i <= lastItemIndex )
        {
            UARTCharPut(UART1_BASE, lastBuffer[i]); // print buffer
            i++;
        }
        clearLastBuffer();
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0); // turn off green led
        if (numThinLines == 1)
        {
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2); // turn on blue led
        }
    }

}

void pidTask()  // PID for following right wall. Also handles executing U-TURNS.
{
    while (1)
    {
        Semaphore_pend(Semaphore0, BIOS_WAIT_FOREVER); // Task unblocked every 50 [ms] by Timer2A
        int frontDistanceADC = findDistanceFront();
        int sideDistanceADC = findDistanceSide();
        double frontDistance = 14871 * pow(frontDistanceADC, -.995); // convert to [cm]
        double sideDistance = 14871 * pow(sideDistanceADC, -.995); // convert to [cm]
        int ADCValueError = sideDistanceADC - 1542;

        if(bufferIndex >= 39) {
            switchBuffer();
            Swi_post(swi0);
        }

        if (uturning == 0)
        {
            double Error = (sideDistance - 10); // We want the robot to follow the right wall from a distance of 10 [cm]
            double P = 15 * fabs(Error);
            double I = 1.5 * (Error + LastError);
            double D = 1 * (Error - LastError);
            double PID = (P + I + D);
            if (PID > 98)
                PID = 98; // Don't let PID value go over 98

            if (Error > 0) // Robot is veering left, so slow down the right motor
            {
                rightmotorcustomspeed(99 - PID);
                leftmotorcustomspeed(99);
            }
            if (Error < 0) // Robot is veering right, so slow down the left motor
            {
                leftmotorcustomspeed(99 - PID);
                rightmotorcustomspeed(99);
            }
            LastError = Error;  // Set LastError for I and D calculations
        }

        if (frontDistance < 8)
        {
            uturning = 1;
            Semaphore_post(Semaphore3);    // If frontDistance < 8 [cm], execute U-TURN
        }

        // After the second thin line is hit, stop collecting data and printing data,
        // set numThinLines to 999, so it wont re-enter this if statement,
        // and print remaining data left in the currentBuffer.
        if (numThinLines == 2)
        {
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);       //turn off blue
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);       //turn off green
            switchBuffer();
            Swi_post(swi0);     // print remaining buffer
            numThinLines = 999; // do this so it doesn't repeat itself every 10 [ms]
        }

        if ((pidCycles % 2 == 0) && (numThinLines == 1)) // every 100[ms] (every other PID cycle) && between first and second thin lines
        {
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2); // turn on blue
            int absError = abs(ADCValueError);
            if (bufferIndex <= 38)
            {
                currentBuffer[bufferIndex] = (absError >> 8);
                currentBuffer[bufferIndex + 1] = (absError & 0xFF);
                bufferIndex += 2;
            }
        }
        pidCycles++;
    }
}

int reflectance() // This function tells you if a surface is black or white, returning a 1 for black and a 0 for white.
{
    int cycles = 0;
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_6);            // set output
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);         // set high... starts charging capacitor
    SysCtlDelay(SysCtlClockGet() / 80000);                         // wait for capacitor to charge 12.5us
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_6);             // set input... capacitor starts discharging
    while (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6) // while capacitor is not discharged..... finds how many [us] it takes to discharge capacitor
    {
        SysCtlDelay(SysCtlClockGet() / 1000000); // 1 microsecond delay
        cycles++;
    }
    if (cycles > 250)
    {
        return 1;      // Black reading
    }
    else
    {
        return 0; // White reading
    }
}

int counter = 0;

void BlackLineInterruptHandler() // Called every 10 [ms] by Timer1A
{
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT); // Reset 10 [ms] Timer1A
    counter++; // This is used to find the maze total run time. It is incremented every 10 [ms].
    Semaphore_post(Semaphore1); // Starts blacklineTask.
}

int currentReflectance;
int lastReflectance;
void blacklineTask() // Detects if robot drives over thin or thick black line.
{
    while (1)
    {
        Semaphore_pend(Semaphore1, BIOS_WAIT_FOREVER); //task unblocked every 10 [ms] by timer
        int cycles = 0;
        while ((currentReflectance = reflectance() == 1) || (lastReflectance == 1))
            // While the sensor is reading BLACK, we increment cycles repeatedly,
            // and we set lastReflectance to currentReflectance to allow
            // one cycle of white in case of error in the sensor reading.
            // Sometimes the sensor erroneously reads white in the middle of a black line,
            // so this disregards one white reading in the middle of a black line.
        {
            cycles++;
            lastReflectance = currentReflectance;
        }
        if (cycles >= 20)    // If cycles is greater than or equal to 20, we assert that
        {                    // a thick line was hit, and we print the corresponding number of cycles,
                             // print the total run time, and stop both motors before calling BIOS_exit(0).
            UARTprintf("Large strip hit numCycles: %d\n", cycles);
            UARTprintf("Total Run Time: %d seconds\n", counter / 100);
            leftmotorstop();
            rightmotorstop();
            BIOS_exit(0);
        }
        if ((cycles > 3) && (cycles < 20)) // If cycles is less than 20 and greater than 3,
        {                                  // we assert that a thin line was hit and increment
            numThinLines++;                // numThinLines by 1.
        }
    }
}

int main(void)
{
    SysCtlClockSet(
    SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN); //   LeftMotorPolarity: PA7
    PWMConfig();
    GPIOConfig(); //                                                        LeftMotor        : PB6     RightMotor: PB7
    ConfigureUART(); // Configure UART for BT                               RX               : PC4     TX        : PC5
    ConfigureADC(); // Configure ADC for distance sensors                   FrontSensor      : PE3     SideSensor: PE2
    ConfigureTimer2A();     // 50ms timer for pid
    ConfigureTimer1A();     // 10ms timer for blackline
    LastError = findDistanceSide() - 10;       //Get initial Error for PID calc.
    leftmotorstart();               //Configures left motor. starts with 0% duty
    rightmotorstart();            // Configures right motor. starts with 0% duty
    BIOS_start();
}

