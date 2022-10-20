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

//#include <xdc/runtime/System.h>
//#include <ti/sysbios/BIOS.h>
//#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/PWM.h>
#include "driverlib/adc.h"
#include "utils/uartstdio.h"
#include "Board.h"
#include <math.h>

//#include "rom.h"
PWM_Handle motor1;
PWM_Handle motor2;      //global variables
PWM_Params params;
void printDistance(){
    int i=0;
    for(i=0;i<15;i++){


            UARTprintf("Distance Front[cm] = %d  Distance Side[cm] = %d      \r", findDistanceFront(), findDistanceSide());

            SysCtlDelay(SysCtlClockGet() / 6);
        }
UARTprintf("\n");

}
int findDistanceFront() {  //PE3
    uint32_t ADCValue[1];
    ADCProcessorTrigger(ADC0_BASE, 3);
    while (!ADCIntStatus(ADC0_BASE, 3, false)) {}
    ADCIntClear(ADC0_BASE, 3);
    ADCSequenceDataGet(ADC0_BASE, 3, ADCValue);
    int distance = (int)14871 * pow(ADCValue[0], -.995); //this eq converts ADC value to cm
    return distance;
}

int findDistanceSide() {   //PE2
    uint32_t ADCValue[1];
    ADCProcessorTrigger(ADC0_BASE, 2);
    while (!ADCIntStatus(ADC0_BASE, 2, false)) {}
    ADCIntClear(ADC0_BASE, 2);
    ADCSequenceDataGet(ADC0_BASE, 2, ADCValue);
    int distance = (int)14871 * pow(ADCValue[0], -.995); //this eq converts ADC value to cm
    return distance;

}

void ConfigureADC() {
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
void setCustomSpeedBothMotors(){
    char mystring[3];
    UARTprintf("Enter speed 0-99\n");
    getstring(mystring);
    int percentage = ((int)mystring[0]-48)*10 +((int)mystring[1]-48);
    UARTprintf("both motors set to %d percent\n",percentage);
    leftMotorCustomSpeed((double)percentage);
    rightMotorCustomSpeed((double)percentage);

}
void leftmotorfast(){  // this function sets PWM to 100% duty cycle
    PWM_setDuty(motor1,65535);
    UARTprintf("Left Motor Fast\n");
}
void leftmotorslow(){  // this function sets PWM to 33% duty cycle
    PWM_setDuty(motor1,65535/3);
    UARTprintf("Left Motor Slow\n");
}
void leftmotorstop(){// this function turns duty cycle to zero
    PWM_setDuty(motor1,0);
    //PWM_close(motor1);
    UARTprintf("Left Motor Stop\n");
}
void leftmotorstart(){  //enables PWM with a 0% duty cycle. you must call leftmotorfast or leftmotorslow to actually get the motor going.
           motor1 =  PWM_open(Board_PWM0, &params);   //PWM0= PWM6 = PF2= Blue LED
           UARTprintf("Left Motor Start \n");
       }
leftMotorCustomSpeed(double speed){ // ENTER PERCENTAGE 0-100
    PWM_setDuty(motor1,65535*(speed/100));

}

void rightmotorfast(){  // this function sets PWM to 100% duty cycle
    PWM_setDuty(motor2,65535);
    UARTprintf("Right Motor Fast\n");
}
rightMotorCustomSpeed(double speed){ // ENTER PERCENTAGE 0-100
    PWM_setDuty(motor2,65535*(speed/100));

}
void rightmotorslow(){  // this function sets PWM to 33% duty cycle
    PWM_setDuty(motor2,65535/3);
    UARTprintf("Right Motor Slow\n");
}
void rightmotorstop(){ // duty cycle to zero
    PWM_setDuty(motor2,0);
    //PWM_close(motor2);
    UARTprintf("Right Motor Stop\n");
}
void rightmotorstart(){  //enables PWM with a 0% duty cycle. you must call rightmotorfast or rightmotorslow to actually get the motor going.
           motor2 =  PWM_open(Board_PWM1, &params);   //PWM0= PWM7 = PF3= GREEN LED
           UARTprintf("Right Motor Start\n");
       }

void bluetoothSendMessage(char *array)
{
    while (*array)
    {
        UARTCharPut(UART3_BASE, *array);
        array++;
    }
}

void red()
{
    int LED = 2;
    UARTprintf("\nRED\n");
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, LED); //14=white 8= green 4=blue 2=red      binary
}

void blue()
{
    int LED = 4;

    UARTprintf("\nBLUE\n");
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, LED);
}

void green()
{
    int LED = 8;
    UARTprintf("\nGREEN\n");
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, LED);
}

void white()
{
    int LED = 14;
    UARTprintf("\nWHITE\n");
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, LED);
}
void clear()
{
    int LED = 0;
    UARTprintf("\nCLEAR\n");
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, LED);
}
void getstring(char uartstring[]){

    uartstring[0] = UARTCharGet(UART1_BASE); //UART0 = USB               UART1 = BLUETOOTH
            uartstring[1] = UARTCharGet(UART1_BASE);

}
struct lut
{
    char funname[2];
    void (*func)(void);
};

struct lut LUT[25] =
    {
        {"re", red},
        {"bc", setCustomSpeedBothMotors},
        {"cl", clear},
        {"ls", leftmotorstart},
        {"lf", leftmotorfast},
        {"sl", leftmotorslow},
        {"el", leftmotorstop},
        {"rs", rightmotorstart},
        {"rf", rightmotorfast},
        {"sr", rightmotorslow},
        {"er", rightmotorstop},
        {"pd",printDistance}
};

void bluetoothSendMessage(char *array);


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
   GPIOPinConfigure(GPIO_PC4_U1RX);                                 //PC4RX PC5TX
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




int main(void)
{
char uartstring[3]="";
int i = 0;
void (*fun)(void);
SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
Board_initGeneral();
  Board_initGPIO();
       Board_initPWM();
       ConfigureUART();
       ConfigureADC();
       UARTprintf("Hello World!\n");
       PWM_Params_init(&params);
       params.dutyMode =PWM_DUTY_SCALAR; //0=0%duty cycle 65535=100% duty cycle



    while (1)
    {

getstring(uartstring); //pulls 2 character string from UART
UARTprintf("your input: %s\n",uartstring);
        for (i = 0; i < 25; i++)
        {
            if (strcmp(uartstring, LUT[i].funname) == 0)  //IF UART STRING ==  Function abbreviation. then set function pointer to respective function.
            {
                fun = LUT[i].func;
                fun();
                break;
            }


        }
        if(i==25)UARTprintf("Invalid input!\n");

    }

    UARTDisable(UART0_BASE);
}

