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

void ConfigureTimer1A()
{
    // Timer 2 setup code           10 MS
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);  // enable Timer 2 periph clks
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC); // cfg Timer 2 mode - periodic

    uint32_t ui32Period = (SysCtlClockGet() / 100); // period = 1/100th of a second AKA 10MS
    TimerLoadSet(TIMER1_BASE, TIMER_A, ui32Period);        // set Timer 2 period

    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT); // enables Timer 2 to interrupt CPU

    TimerEnable(TIMER1_BASE, TIMER_A);                      // enable Timer 2
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


int i1=0;
int i2=0;
void timer1(){
    UARTprintf("i1\n");
   if((i1%2)==0)GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1,GPIO_PIN_1);
    else GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1,0);
   i1++;

   TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

}
void timer2(){
    UARTprintf("i2\n");
    if(i2%2==0)GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2,GPIO_PIN_2);
        else GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1,0);
    i2++;
    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
}

int main(void)
{

    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
    Board_initGeneral();
    ConfigureUART();
    Board_initGPIO();
    ConfigureTimer2A();
    ConfigureTimer1A();
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
    UARTprintf("Hello World!\n");
    BIOS_start();


}
