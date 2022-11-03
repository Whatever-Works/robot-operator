//These Functions are only used in debug mode so we hide them here
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

extern PWM_Handle motor1;
extern PWM_Handle motor2;
extern int findDistanceFront();
extern int findDistanceSide();
extern void leftmotorcustomspeed(int);
extern void rightmotorcustomspeed(int);
extern void rightmotorstart();
extern void leftmotorstart();
extern void rightmotorstop();
extern void leftmotorstop();
struct lut
{
    char funname[2];
    void (*func)(void);
};


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
void rightmotorfast()// this function sets PWM to 100% duty cycle
{
    PWM_setDuty(motor2, 65535);
    UARTprintf("Right Motor Fast\n");
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
