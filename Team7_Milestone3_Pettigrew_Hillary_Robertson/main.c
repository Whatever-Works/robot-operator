#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"

#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"

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
    bluetoothSendMessage("\nRED\n");
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, LED); //14=white 8= green 4=blue 2=red      binary
}

void blue()
{
    int LED = 4;
    bluetoothSendMessage("\nBLUE\n");
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, LED);
}

void green()
{
    int LED = 8;
    bluetoothSendMessage("\nGREEN\n");
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, LED);
}

void white()
{
    int LED = 14;
    bluetoothSendMessage("\nWHITE\n");
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, LED);
}
void clear()
{
    int LED = 0;
    bluetoothSendMessage("\nCLEAR\n");
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, LED);
}

struct lut
{
    char funname[2];
    void (*func)(void);
};

struct lut LUT[10] =
    {
        {"re", red},
        {"bl", blue},
        {"gr", green},
        {"wh", white},
        {"cl", clear}
};

void bluetoothSendMessage(char *array);

/*void USBUARTCONFIG(){
SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
       GPIOPinConfigure(GPIO_PA0_U0RX);
       GPIOPinConfigure(GPIO_PA1_U0TX);
       GPIOPinTypeUART(GPIO_PORTA_BASE,GPIO_PIN_0|GPIO_PIN_1);
       SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
       UARTConfigSetExpClk(UART0_BASE,SysCtlClockGet(),9600,(UART_CONFIG_WLEN_8 | UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE));
       UARTEnable(UART0_BASE);

}
*/

void btConfig()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    GPIOPinConfigure(GPIO_PC6_U3RX);                           // PC6 RECEIVE
    GPIOPinConfigure(GPIO_PC7_U3TX);                           //PC7 Transmit
    GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_6 | GPIO_PIN_7); //SET PC6 and PC7 to uart
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);
    UARTConfigSetExpClk(UART3_BASE, SysCtlClockGet(), 9600, (UART_CONFIG_WLEN_8 | UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE));
    UARTEnable(UART3_BASE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
}

int main(void)
{
    char uartstring[2];
    void (*fun)(void);
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
    btConfig();
    //USBUARTCONFIG();            // USB SERIAL UART 9600 baud
    int i = 0;

    while (1)
    {
        uartstring[0] = UARTCharGet(UART3_BASE); //UART0 = USB               UART3 = BLUETOOTH
        uartstring[1] = UARTCharGet(UART3_BASE);

        for (i = 0; i < 10; i++)
        {
            if (strcmp(uartstring, LUT[i].funname) == 0)
            {
                fun = LUT[i].func;
                break;
            }
        }

        fun();
        SysCtlDelay(10000000);
    }

    UARTDisable(UART3_BASE);
}

