#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include "inc/hw_i2c.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_qei.h"
#include "inc/hw_timer.h"
#include "driverlib/rom.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/pwm.h"
#include "driverlib/timer.h"

#include "driverlib/qei.h"
#include "utils/uartstdio.h"
#include "utils/uartstdio.c"

#include "periphConf.h"

#include "my_util.h"
#include "my_i2c.h"

#include "buzzer.h"
#include "lcd_SB1602.h"

int32_t idx = 0;
int32_t line = 0;
void writeCharLCDx(uint8_t ch) {
    if (line == 0 && idx == 16) {
        idx = 0;
        line = 1;
        setAddressLCD(0, 1);
        writeTextLCD("                ", 16);
    } else if (line == 1 && idx == 16) {
        idx = 0;
        line = 0;
        setAddressLCD(0, 0);
        writeTextLCD("                ", 16);
    }
    setAddressLCD(idx++, line);
    uint8_t a[] = {ch};
    writeTextLCD(a, 1);
}

void initConsole(void) {
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  GPIOPinConfigure(GPIO_PA0_U0RX);
  GPIOPinConfigure(GPIO_PA1_U0TX);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  UARTStdioConfig(0, 9600, 16000000);
}

void initInterruptPins(void) {
    GPIOIntClear(GPIO_PORTF_BASE, INT_ALL_BUTTONS);
    GPIOIntRegister(GPIO_PORTF_BASE, SW1PinIntHandler);
    GPIOIntTypeSet(GPIO_PORTF_BASE, INT_ALL_BUTTONS, GPIO_FALLING_EDGE);
}

uint32_t press_cnt = 0;
uint32_t nprs_cnt = 0;

int32_t incIdx = 0;
int32_t codeGen = 0;

int32_t code_idx = 0;
int32_t code[5];

uint8_t getCode() {
    switch (code_idx) {
        case 1:
            if (code[0] == 0) return 'E';
            if (code[0] == 1) return 'T';
        case 2:
            if (code[0] == 0 && code[1] == 1) return 'A';
            if (code[0] == 0 && code[1] == 0) return 'I';
            if (code[0] == 1 && code[1] == 1) return 'M';
            if (code[0] == 1 && code[1] == 0) return 'N';
        case 3:
            if (code[0] == 1 && code[1] == 0 && code[2] == 0) return 'D';
            if (code[0] == 1 && code[1] == 1 && code[2] == 0) return 'G';
            if (code[0] == 1 && code[1] == 0 && code[2] == 1) return 'K';
            if (code[0] == 1 && code[1] == 1 && code[2] == 1) return 'O';
            if (code[0] == 0 && code[1] == 1 && code[2] == 0) return 'R';
            if (code[0] == 0 && code[1] == 0 && code[2] == 0) return 'S';
            if (code[0] == 0 && code[1] == 0 && code[2] == 1) return 'U';
            if (code[0] == 0 && code[1] == 1 && code[2] == 1) return 'W';
        case 4:
            if (code[0] == 1 && code[1] == 0 && code[2] == 0 && code[3] == 0) return 'B';
            if (code[0] == 1 && code[1] == 0 && code[2] == 1 && code[3] == 0) return 'C';
            if (code[0] == 0 && code[1] == 0 && code[2] == 1 && code[3] == 0) return 'F';
            if (code[0] == 0 && code[1] == 0 && code[2] == 0 && code[3] == 0) return 'H';
            if (code[0] == 0 && code[1] == 1 && code[2] == 1 && code[3] == 1) return 'J';
            if (code[0] == 0 && code[1] == 1 && code[2] == 0 && code[3] == 0) return 'L';
            if (code[0] == 0 && code[1] == 1 && code[2] == 1 && code[3] == 0) return 'P';
            if (code[0] == 1 && code[1] == 1 && code[2] == 0 && code[3] == 1) return 'Q';
            if (code[0] == 0 && code[1] == 0 && code[2] == 0 && code[3] == 1) return 'V';
            if (code[0] == 1 && code[1] == 0 && code[2] == 0 && code[3] == 1) return 'X';
            if (code[0] == 1 && code[1] == 0 && code[2] == 1 && code[3] == 1) return 'Y';
            if (code[0] == 1 && code[1] == 1 && code[2] == 0 && code[3] == 0) return 'Z';
        case 5:
            if (code[0] == 0 && code[1] == 1 && code[2] == 1 && code[3] == 1 && code[4] == 1) return '1';
            if (code[0] == 0 && code[1] == 0 && code[2] == 1 && code[3] == 1 && code[4] == 1) return '2';
            if (code[0] == 0 && code[1] == 0 && code[2] == 0 && code[3] == 1 && code[4] == 1) return '3';
            if (code[0] == 0 && code[1] == 0 && code[2] == 0 && code[3] == 0 && code[4] == 1) return '4';
            if (code[0] == 0 && code[1] == 0 && code[2] == 0 && code[3] == 0 && code[4] == 0) return '5';
            if (code[0] == 1 && code[1] == 0 && code[2] == 0 && code[3] == 0 && code[4] == 0) return '6';
            if (code[0] == 1 && code[1] == 1 && code[2] == 0 && code[3] == 0 && code[4] == 0) return '7';
            if (code[0] == 1 && code[1] == 1 && code[2] == 1 && code[3] == 0 && code[4] == 0) return '8';
            if (code[0] == 1 && code[1] == 1 && code[2] == 1 && code[3] == 1 && code[4] == 0) return '9';
            if (code[0] == 1 && code[1] == 1 && code[2] == 1 && code[3] == 1 && code[4] == 1) return '0';
        default:
            return '.';
    }
}

int32_t prev_idx = 0;
void SysTickIntHandler(void) {

    if (GPIOPinRead(GPIO_PORTF_BASE, INT_ALL_BUTTONS) == 0) {
        if (prev_idx != code_idx) press_cnt = 0;
        incIdx = 1;

        if (press_cnt > 7) {
            code[code_idx] = 1;
            UARTprintf("[info] long\n");
        } else {
            code[code_idx] = 0;
            UARTprintf("[info] short\n");
        }

        GPIOPinWrite(GPIO_PORTF_BASE, LED_WHITE, 0); // turn off all LEDs
        GPIOPinWrite(GPIO_PORTF_BASE, LED_WHITE, LED_ALL);
        toneBuzzer(2270);

        press_cnt++;
        nprs_cnt = 0;
        prev_idx = code_idx;
    } else {
        if (nprs_cnt > 15 && press_cnt != 0) {
            press_cnt = 0;
            codeGen = 1;
            UARTprintf("[info] code gen\n");
        }

        if (incIdx == 1 && press_cnt != 0 && code_idx < 5) {
            incIdx = 0;
            code_idx++;
        }

        GPIOPinWrite(GPIO_PORTF_BASE, LED_WHITE, 0);
        restBuzzer();

        nprs_cnt++;
    }

    if (codeGen == 1) {
        codeGen = 0;

        uint8_t ch = getCode();
        UARTprintf("[info] write LCD '%c'\n", ch);
        writeCharLCDx(ch);

        code_idx = 0;
    }
}

void SW1PinIntHandler(void) {
    UARTprintf("[INT] SW1\n");

    GPIOIntDisable(GPIO_PORTF_BASE, INT_ALL_BUTTONS);
    GPIOIntClear(GPIO_PORTF_BASE, INT_ALL_BUTTONS);

    //

    GPIOIntEnable(GPIO_PORTF_BASE, INT_ALL_BUTTONS);
}

int main(void) {
  ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
             SYSCTL_XTAL_16MHZ);
  PortFunctionInit();

  initInterruptPins();
  GPIOIntEnable(GPIO_PORTF_BASE, INT_ALL_BUTTONS);

  initConsole();
  UARTprintf("[RST]\n");

  initBuzzer();

  initI2C(I2C3_BASE);
  initLCD();

  SysTickPeriodSet(SysCtlClockGet() / SYSTICKS_PER_SEC);
  SysTickEnable();
  SysTickIntRegister(SysTickIntHandler);
  SysTickIntEnable();

  while(1);
}
