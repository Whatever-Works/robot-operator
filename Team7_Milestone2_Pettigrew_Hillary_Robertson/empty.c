#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <inttypes.h>

#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"

#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"

void bluetoothSendMessage(char *array);

void btConfig()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    GPIOPinConfigure(GPIO_PC6_U3RX);    // PC6 RECEIVE
    GPIOPinConfigure(GPIO_PC7_U3TX);         //PC7 Transmit
    GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_6 | GPIO_PIN_7); //SET PC6 and PC7 to uart
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);
    UARTConfigSetExpClk(
            UART3_BASE, SysCtlClockGet(), 9600,
            (UART_CONFIG_WLEN_8 | UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE));
    UARTEnable(UART3_BASE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,
                          GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

}

int main(void)
{
    unsigned char data;
    int LED = 0;

    SysCtlClockSet(
            SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ
                    | SYSCTL_OSC_MAIN);
    btConfig();

    while (1)
    {
        while (!UARTCharsAvail(UART3_BASE))
            ;
        data = UARTCharGetNonBlocking(UART3_BASE);

        if (data == 'r')
        {
            LED = 2;
            bluetoothSendMessage("\nRED\n");
        }
        else if (data == 'b')
        {
            LED = 4;

            bluetoothSendMessage("\nBLUE\n");
        }
        else if (data == 'g')
        {
            LED = 8;

            bluetoothSendMessage("\nGREEN\n");
        }
        else if (data == 'w')
        {
            LED = 14;

            bluetoothSendMessage("\nWHITE\n");
        }

        else if (data == 'c')
        {
            LED = 0;

            bluetoothSendMessage("\nCLEAR\n");
        }

        else
        {

            bluetoothSendMessage("\nINVALID\n");
        }

        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
                     LED); //14=white 8= green 4=blue 2=red      binary
        SysCtlDelay(10000000);

    }
    UARTDisable(UART1_BASE);
}

void bluetoothSendMessage(char *array)
{
    while (*array)
    {
        UARTCharPut(UART3_BASE, *array);
        array++;
    }
}
