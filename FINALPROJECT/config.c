// THIS FILE CONTAINS FUNCTIONS THAT CONFIGURE PWM/TIMERS/GPIO/UART/ADC

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
PWM_Params params;
PWMConfig()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinConfigure(GPIO_PB7_M0PWM1);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6 | GPIO_PIN_7);
    PWM_init();
    PWM_Params_init(&params);
    params.dutyMode = PWM_DUTY_SCALAR; //sets mode of duty cycle to  0 = 0%duty cycle 65535 = 100% duty cycle
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
    GPIOPinConfigure(GPIO_PC4_U1RX);  //PC4RX PC5TX
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

void ConfigureTimer2A() // Timer 2A setup code. This calls PIDTaskHandler every 50 [ms]
{

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);  // enable Timer 2 periph clks
    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC); // cfg Timer 2 mode - periodic
    uint32_t ui32Period = (SysCtlClockGet() / 20); // period = 1/20th of a second AKA 50MS
    TimerLoadSet(TIMER2_BASE, TIMER_A, ui32Period);        // set Timer 2 period
    TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT); // enables Timer 2 to interrupt CPU
    TimerEnable(TIMER2_BASE, TIMER_A);                       // enable Timer 2
}

void ConfigureTimer1A() // Timer 1A setup code. 10 [ms] used for calling blackline interrupt
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);  // enable Timer 1 periph clks
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC); // cfg Timer 1 mode - periodic
    uint32_t ui32Period = (SysCtlClockGet() / 100); // period = 1/100th of a second AKA 10MS
    TimerLoadSet(TIMER1_BASE, TIMER_A, ui32Period);  // set Timer 1 period
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT); // enables Timer 2 to interrupt CPU
    TimerEnable(TIMER1_BASE, TIMER_A); // enable Timer 2
}

void GPIOConfig()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // enable port A for left motor polarity, and reflectance sensor
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_7); // This is used to control left motor directional polarity. LO=forward HI=reverse
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);       //red led
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);       //blue led
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);       //green led
}

// Configures ADC usage for PE3 and PE2
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
