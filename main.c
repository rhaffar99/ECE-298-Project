#include "main.h"
#include "driverlib/driverlib.h"
#include "hal_LCD.h"
#include "stdlib.h"


#define X_POS_INIT 0
#define Y_POS_INIT 0
#define STEP_CONVERSION_CONST 43 * 8

uint8_t uart_input = 0;
char ADCState = 0; //Busy state of the ADC
int16_t ADCResult = 0; //Storage for the ADC conversion result

void display_bar_graph(int x_pos, int y_pos);
int drive_motor_x(int direction, int curStep);
int drive_motor_y(int direction, int curStep);
int pos_to_step_amnt(int posOne, int posTwo);

void main(void)
{
    /*
     * Functions with two underscores in front are called compiler intrinsics.
     * They are documented in the compiler user guide, not the IDE or MCU guides.
     * They are a shortcut to insert some assembly code that is not really
     * expressible in plain C/C++. Google "MSP430 Optimizing C/C++ Compiler
     * v18.12.0.LTS" and search for the word "intrinsic" if you want to know
     * more.
     * */

    //Turn off interrupts during initialization
    __disable_interrupt();

    //Stop watchdog timer unless you plan on using it
    WDT_A_hold(WDT_A_BASE);

    // Initializations - see functions for more detail
    Init_GPIO();    //Sets all pins to output low as a default
    Init_PWM();     //Sets up a PWM output
    Init_ADC();     //Sets up the ADC to sample
    Init_Clock();   //Sets up the necessary system clocks
    Init_UART();    //Sets up an echo over a COM port
    Init_LCD();     //Sets up the LaunchPad LCD display

     /*
     * The MSP430 MCUs have a variety of low power modes. They can be almost
     * completely off and turn back on only when an interrupt occurs. You can
     * look up the power modes in the Family User Guide under the Power Management
     * Module (PMM) section. You can see the available API calls in the DriverLib
     * user guide, or see "pmm.h" in the driverlib directory. Unless you
     * purposefully want to play with the power modes, just leave this command in.
     */
    PMM_unlockLPM5(); //Disable the GPIO power-on default high-impedance mode to activate previously configured port settings

    //All done initializations - turn interrupts back on.
    __enable_interrupt();

    int x_dir = 0;
    int y_dir = 0;
    int motor_step_x = 0;
    int motor_step_y = 0;

    showChar('=', pos1);
    showChar('=', pos2);
    showChar('=', pos3);
    showChar('=', pos4);
    showChar('=', pos5);
    showChar('=', pos6);


    bool coordinates_acquired = false;
    int coordinate_arr[10] = {};
    int negPos = 0;

    int x_pos_step = X_POS_INIT;
    int y_pos_step = Y_POS_INIT;
    int cur_x_dif = 0;
    int cur_y_dif = 0;

    while(1) //Do this when you want an infinite loop of code
    {
        int coordinate_counter = 0;
        // Get necessary coordinates
        while(uart_input != (int) 'i');
        while(!coordinates_acquired) {

            if (coordinate_counter == 10) {
                coordinates_acquired = true;
            }

            if (uart_input == (int) ' ') {
                while (uart_input == (int) ' ');
                while (uart_input == (int) '-') {
                    negPos = 1;
                }
                coordinate_arr[coordinate_counter] = uart_input - '0';
                if (negPos) {
                    coordinate_arr[coordinate_counter] = -coordinate_arr[coordinate_counter];
                }
                coordinate_counter ++;
                negPos = 0;
            }

        }
        while(uart_input != (int) 'g');

        //Begin moving motors
        unsigned int coordinate_num = 0;
        for (coordinate_num; coordinate_num < 5; coordinate_num++) {
            if (coordinate_num == 0) {
                cur_x_dif = coordinate_arr[0] - 0;
                cur_y_dif = coordinate_arr[1] - 0;
            } else {
                cur_x_dif = coordinate_arr[2 * coordinate_num] - coordinate_arr[2 * (coordinate_num - 1)];
                cur_y_dif = coordinate_arr[(2 * coordinate_num) + 1] - coordinate_arr[2 * (coordinate_num - 1) + 1];
            }


            x_pos_step = STEP_CONVERSION_CONST * cur_x_dif;
            y_pos_step = STEP_CONVERSION_CONST * cur_y_dif;

            if (x_pos_step >= 0) {
                x_dir = 1;
            } else {
                x_dir = -1;
                x_pos_step = -x_pos_step;
            }

            if (y_pos_step >= 0) {
                y_dir = 1;
            } else {
                y_dir = -1;
                y_pos_step = -y_pos_step;
            }

            int max_step = 0;
            if (x_pos_step >= y_pos_step) {
                max_step = x_pos_step;
            } else {
                max_step = y_pos_step;
            }

            int total_counter = 0;
            while (total_counter <= max_step) {
                motor_step_x = 0;
                motor_step_y = 0;
                int step_counter = 0;
                while (step_counter <= STEP_CONVERSION_CONST) {

                    if (step_counter <= x_pos_step) {
                        motor_step_x = drive_motor_x(x_dir, motor_step_x);
                    }

                    if (step_counter <= y_pos_step) {
                        motor_step_y = drive_motor_y(y_dir, motor_step_y);
                    }
                    step_counter++;
                }
                total_counter += step_counter;
                if (!((GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN6)) && (GPIO_getInputPinValue(GPIO_PORT_P8, GPIO_PIN0)) && (GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN0)) && (GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN2)))) {
                    break;
                }
            }
            display_bar_graph(coordinate_arr[2 * coordinate_num], coordinate_arr[(2 * coordinate_num) + 1]);
            _delay_cycles(1000000);
        }

        coordinates_acquired = false;

    }

    /*
     * You can use the following code if you plan on only using interrupts
     * to handle all your system events since you don't need any infinite loop of code.
     *
     * //Enter LPM0 - interrupts only
     * __bis_SR_register(LPM0_bits);
     * //For debugger to let it know that you meant for there to be no more code
     * __no_operation();
    */

}

/*void recalibrate_motors() {
    int i = 0;
    while ((GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN6) == 1) {
        int step_counter = 0;
        motor_step_x = 0;
        while (step_counter <= STEP_CONVERSION_CONST) {
            motor_step_x = drive_motor_x(1, motor_step_x);
        }

    }
    motor_step_x = 0;
    for (i = 0; i < STEP_CONVERSION_CONST * 3; i++) {
        motor_step_x = drive_motor_x(-1, motor_step_x);
    }

    while ((GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN6) == 1) {
        int step_counter = 0;
        motor_step_y = 0;
        while (step_counter <= STEP_CONVERSION_CONST) {
            motor_step_y = drive_motor_y(1, motor_step_y);
        }
    }
    motor_step_y = 0;
    for (i = 0; i < STEP_CONVERSION_CONST * 3; i++) {
        motor_step_y = drive_motor_y(-1, motor_step_y);
    }

}*/

void display_bar_graph(int x_pos, int y_pos) {
    int i;
    char curChar = ' ';
    showChar(' ', pos1);
    showChar(' ', pos2);
    showChar(' ', pos3);
    showChar(' ', pos4);
    showChar(' ', pos5);
    showChar(' ', pos6);

    for (i = -2; i <= 3; i++) {
        if (x_pos >= i && y_pos >= i) {
            curChar = '=';
        } else if (x_pos >= i) {
            curChar = '-';
        } else if (y_pos >= i) {
            curChar = '_';
        } else {
            break;
        }
        switch (i) {
            case -2:
                showChar(curChar, pos1);
                break;
            case -1:
                showChar(curChar, pos2);
                break;
            case 0:
                showChar(curChar, pos3);
                break;
            case 1:
                showChar(curChar, pos4);
                break;
            case 2:
                showChar(curChar, pos5);
                break;
            case 3:
                showChar(curChar, pos6);
                break;
        }
    }
}

void Init_GPIO(void)
{
    // Set all GPIO pins to output low to prevent floating input and reduce power consumption
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN1|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN1|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    GPIO_setAsInputPin(GPIO_PORT_P8, GPIO_PIN0);
    GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN6);
    GPIO_setAsInputPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN2);

    //Set LaunchPad switches as inputs - they are active low, meaning '1' until pressed
    GPIO_setAsInputPinWithPullUpResistor(SW1_PORT, SW1_PIN);
    GPIO_setAsInputPinWithPullUpResistor(SW2_PORT, SW2_PIN);

    //Set LED1 and LED2 as outputs
    //GPIO_setAsOutputPin(LED1_PORT, LED1_PIN); //Comment if using UART
    GPIO_setAsOutputPin(LED2_PORT, LED2_PIN);
}

/* Clock System Initialization */
void Init_Clock(void)
{
    /*
     * The MSP430 has a number of different on-chip clocks. You can read about it in
     * the section of the Family User Guide regarding the Clock System ('cs.h' in the
     * driverlib).
     */

    /*
     * On the LaunchPad, there is a 32.768 kHz crystal oscillator used as a
     * Real Time Clock (RTC). It is a quartz crystal connected to a circuit that
     * resonates it. Since the frequency is a power of two, you can use the signal
     * to drive a counter, and you know that the bits represent binary fractions
     * of one second. You can then have the RTC module throw an interrupt based
     * on a 'real time'. E.g., you could have your system sleep until every
     * 100 ms when it wakes up and checks the status of a sensor. Or, you could
     * sample the ADC once per second.
     */
    //Set P4.1 and P4.2 as Primary Module Function Input, XT_LF
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN1 + GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);

    // Set external clock frequency to 32.768 KHz
    CS_setExternalClockSource(32768);
    // Set ACLK = XT1
    CS_initClockSignal(CS_ACLK, CS_XT1CLK_SELECT, CS_CLOCK_DIVIDER_1);
    // Initializes the XT1 crystal oscillator
    CS_turnOnXT1LF(CS_XT1_DRIVE_1);
    // Set SMCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_SMCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
    // Set MCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_MCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
}

/* UART Initialization */
void Init_UART(void)
{
    /* UART: It configures P1.0 and P1.1 to be connected internally to the
     * eSCSI module, which is a serial communications module, and places it
     * in UART mode. This let's you communicate with the PC via a software
     * COM port over the USB cable. You can use a console program, like PuTTY,
     * to type to your LaunchPad. The code in this sample just echos back
     * whatever character was received.
     */

    //Configure UART pins, which maps them to a COM port over the USB cable
    //Set P1.0 and P1.1 as Secondary Module Function Input.
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN1, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN0, GPIO_PRIMARY_MODULE_FUNCTION);

    /*
     * UART Configuration Parameter. These are the configuration parameters to
     * make the eUSCI A UART module to operate with a 9600 baud rate. These
     * values were calculated using the online calculator that TI provides at:
     * http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
     */

    //SMCLK = 1MHz, Baudrate = 9600
    //UCBRx = 6, UCBRFx = 8, UCBRSx = 17, UCOS16 = 1
    EUSCI_A_UART_initParam param = {0};
        param.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
        param.clockPrescalar    = 6;
        param.firstModReg       = 8;
        param.secondModReg      = 17;
        param.parity            = EUSCI_A_UART_NO_PARITY;
        param.msborLsbFirst     = EUSCI_A_UART_LSB_FIRST;
        param.numberofStopBits  = EUSCI_A_UART_ONE_STOP_BIT;
        param.uartMode          = EUSCI_A_UART_MODE;
        param.overSampling      = 1;

    if(STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A0_BASE, &param))
    {
        return;
    }

    EUSCI_A_UART_enable(EUSCI_A0_BASE);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

    // Enable EUSCI_A0 RX interrupt
    EUSCI_A_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
}

/* EUSCI A0 UART ISR - Echoes data back to PC host */
#pragma vector=USCI_A0_VECTOR
__interrupt
void EUSCIA0_ISR(void)
{
    uint8_t RxStatus = EUSCI_A_UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, RxStatus);

    if (RxStatus)
    {
        uart_input = EUSCI_A_UART_receiveData(EUSCI_A0_BASE);
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, EUSCI_A_UART_receiveData(EUSCI_A0_BASE));
    }
}

/* PWM Initialization */
void Init_PWM(void)
{
    /*
     * The internal timers (TIMER_A) can auto-generate a PWM signal without needing to
     * flip an output bit every cycle in software. The catch is that it limits which
     * pins you can use to output the signal, whereas manually flipping an output bit
     * means it can be on any GPIO. This function populates a data structure that tells
     * the API to use the timer as a hardware-generated PWM source.
     *
     */
    //Generate PWM - Timer runs in Up-Down mode
    param.clockSource           = TIMER_A_CLOCKSOURCE_SMCLK;
    param.clockSourceDivider    = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    param.timerPeriod           = TIMER_A_PERIOD; //Defined in main.h
    param.compareRegister       = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    param.compareOutputMode     = TIMER_A_OUTPUTMODE_RESET_SET;
    param.dutyCycle             = HIGH_COUNT; //Defined in main.h

    //PWM_PORT PWM_PIN (defined in main.h) as PWM output
    GPIO_setAsPeripheralModuleFunctionOutputPin(PWM_PORT, PWM_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
}

int drive_motor_x(int direction, int curStep) {
    switch(curStep){
    case 0:
        GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN3); //In1
        GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN2); //IN2
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5); //IN3
        GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN1); //IN4
        _delay_cycles(2000);
        break;
    case 1:
        GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN3);
        GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN2);
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5);
        GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN1);
        _delay_cycles(2000);
        break;
    case 2:
        GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN3);
        GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN2);
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5);
        GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN1);
        _delay_cycles(2000);
        break;
    case 3:
        GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN3);
        GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN2);
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5);
        GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN1);
        _delay_cycles(2000);
        break;
    case 4:
        GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN3);
        GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN2);
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);
        GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN1);
        _delay_cycles(2000);
        break;
    case 5:
        GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN3);
        GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN2);
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);
        GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN1);
        _delay_cycles(2000);
        break;
    case 6:
        GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN3);
        GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN2);
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);
        GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN1);
        _delay_cycles(2000);
        break;
    case 7:
        GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN3);
        GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN2);
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);
        GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN1);
        _delay_cycles(2000);
        break;
    }
    if (direction == -1 && curStep == 0) {
        return 7;
    } else if (direction == 1 && curStep == 7) {
        return 0;
    } else {
        return curStep += direction;
    }
}

int drive_motor_y(int direction, int curStep) {
    switch(curStep){
    case 0:
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5); //In1
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4); //IN2
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3); //IN3
        GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN3); //IN4
        _delay_cycles(2000);
        break;
    case 1:
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5);
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4);
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);
        GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN3);
        _delay_cycles(2000);
        break;
    case 2:
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5);
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4);
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);
        GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3);
        _delay_cycles(2000);
        break;
    case 3:
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5);
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4);
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);
        GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3);
        _delay_cycles(2000);
        break;
    case 4:
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5);
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4);
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
        GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3);
        _delay_cycles(2000);
        break;
    case 5:
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN5);
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4);
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
        GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3);
        _delay_cycles(2000);
        break;
    case 6:
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN5);
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4);
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
        GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3);
        _delay_cycles(2000);
        break;
    case 7:
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN5);
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4);
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
        GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN3);
        _delay_cycles(2000);
        break;
    }
    if (direction == -1 && curStep == 0) {
        return 7;
    } else if (direction == 1 && curStep == 7) {
        return 0;
    } else {
        return curStep += direction;
    }
}

void Init_ADC(void)
{
    /*
     * To use the ADC, you need to tell a physical pin to be an analog input instead
     * of a GPIO, then you need to tell the ADC to use that analog input. Defined
     * these in main.h for A9 on P8.1.
     */

    //Set ADC_IN to input direction
    GPIO_setAsPeripheralModuleFunctionInputPin(ADC_IN_PORT, ADC_IN_PIN, GPIO_PRIMARY_MODULE_FUNCTION);

    //Initialize the ADC Module
    /*
     * Base Address for the ADC Module
     * Use internal ADC bit as sample/hold signal to start conversion
     * USE MODOSC 5MHZ Digital Oscillator as clock source
     * Use default clock divider of 1
     */
    ADC_init(ADC_BASE,
             ADC_SAMPLEHOLDSOURCE_SC,
             ADC_CLOCKSOURCE_ADCOSC,
             ADC_CLOCKDIVIDER_1);

    ADC_enable(ADC_BASE);

    /*
     * Base Address for the ADC Module
     * Sample/hold for 16 clock cycles
     * Do not enable Multiple Sampling
     */
    ADC_setupSamplingTimer(ADC_BASE,
                           ADC_CYCLEHOLD_16_CYCLES,
                           ADC_MULTIPLESAMPLESDISABLE);

    //Configure Memory Buffer
    /*
     * Base Address for the ADC Module
     * Use input ADC_IN_CHANNEL
     * Use positive reference of AVcc
     * Use negative reference of AVss
     */
    ADC_configureMemory(ADC_BASE,
                        ADC_IN_CHANNEL,
                        ADC_VREFPOS_AVCC,
                        ADC_VREFNEG_AVSS);

    ADC_clearInterrupt(ADC_BASE,
                       ADC_COMPLETED_INTERRUPT);

    //Enable Memory Buffer interrupt
    ADC_enableInterrupt(ADC_BASE,
                        ADC_COMPLETED_INTERRUPT);
}

//ADC interrupt service routine
#pragma vector=ADC_VECTOR
__interrupt
void ADC_ISR(void)
{
    uint8_t ADCStatus = ADC_getInterruptStatus(ADC_BASE, ADC_COMPLETED_INTERRUPT_FLAG);

    ADC_clearInterrupt(ADC_BASE, ADCStatus);

    if (ADCStatus)
    {
        ADCState = 0; //Not busy anymore
        ADCResult = ADC_getResults(ADC_BASE);
    }
}
