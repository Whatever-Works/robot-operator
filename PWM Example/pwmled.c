/*
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== pwmled.c ========
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/PWM.h>

/* Example/Board Header files */
#include "Board.h"

#define TASKSTACKSIZE   512

Task_Struct tsk0Struct;
UInt8 tsk0Stack[TASKSTACKSIZE];
Task_Handle task;

/*
 *  ======== pwmLEDFxn ========
 *  Task periodically increments the PWM duty for the on board LED.
 */
Void pwmLEDFxn(UArg arg0, UArg arg1)
{
    PWM_Handle pwm0;
    PWM_Params params;
    uint16_t   pwmPeriod = 3000;      // Period and duty in microseconds

    PWM_Params_init(&params);
    params.period = pwmPeriod;
    params.dutyMode = PWM_DUTY_SCALAR;
    pwm0 = PWM_open(Board_PWM0, &params);
    if (pwm0 == NULL) {
        System_abort("Board_PWM0 did not open");
    }

    // duty value is scalar meaning 0% == 0 and 100% == 65,535
    while(1) {
        PWM_setDuty(pwm0, 19660);
        Task_sleep(1000);
        PWM_setDuty(pwm0, 58981);
        Task_sleep(1000);
    }
}

/*
 *  ======== main ========
 */
int main(void)
{
    Task_Params tskParams;

    /* Call board init functions. */
    Board_initGeneral();
    Board_initGPIO();
    Board_initPWM();

    /* Construct LED Task thread */
    Task_Params_init(&tskParams);
    tskParams.stackSize = TASKSTACKSIZE;
    tskParams.stack = &tsk0Stack;
    tskParams.arg0 = 50;
    Task_construct(&tsk0Struct, (Task_FuncPtr)pwmLEDFxn, &tskParams, NULL);

    /* Obtain instance handle */
//    Task_handle(&tsk0Struct);

    /* Turn on user LED */
    GPIO_write(Board_LED0, Board_LED_ON);

    System_printf("Starting the example\nSystem provider is set to SysMin. "
                  "Halt the target to view any SysMin contents in ROV.\n");

    /* SysMin will only print to the console when you call flush or exit */
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
