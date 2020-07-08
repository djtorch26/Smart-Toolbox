#include <msp430.h>
#include <driverlib.h>
#include "StopWatchMode.h"
#include "TempSensorMode.h"
#include "hal_LCD.h"

#define STARTUP_MODE         0

volatile unsigned char mode = STARTUP_MODE;
volatile unsigned char stopWatchRunning = 0;
volatile unsigned char tempSensorRunning = 0;
volatile unsigned char S1buttonDebounce = 0;
volatile unsigned char S2buttonDebounce = 0;
volatile unsigned int holdCount = 0;
volatile unsigned int counter = 0;
volatile int centisecond = 0;
Calendar currentTime;

// TimerA0 UpMode Configuration Parameter
Timer_A_initUpModeParam initUpParam_A0 =
{
        TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_1,          // SMCLK/4 = 2MHz
        30000,                                  // 15ms debounce period
        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,    // Enable CCR0 interrupt
        TIMER_A_DO_CLEAR,                       // Clear value
        true                                    // Start Timer
};

// Initialization calls
void Init_GPIO(void);
void Init_Clock(void);

#define ENABLE_PINS 0xFFFE // Required to use inputs and outputs
#define UART_CLK_SEL 0x0080 // Specifies accurate SMCLK clock for UART
#define BR0_FOR_9600 0x34 // Value required to use 9600 baud
#define BR1_FOR_9600 0x00 // Value required to use 9600 baud
#define CLK_MOD 0x4911 // Microcontroller will "clean-up" clock signal

volatile int tape = 0;
volatile int screwdriver = 0;

void select_clock_signals(void)
{
CSCTL0 = 0xA500; // "Password" to access clock calibration registers
CSCTL1 = 0x0046; // Specifies frequency of main clock
CSCTL2 = 0x0133; // Assigns additional clock signals
CSCTL3 = 0x0000; // Use clocks at intended frequency, do not slow them down
}

void assign_pins_to_uart(void)
{
P4SEL1 = 0x00; // 0000 0000
P4SEL0 = BIT3 | BIT2; // 0000 1100
// ^^
// ||
// |+---- 01 assigns P4.2 to UART Transmit (TXD)

// |

// +----- 01 assigns P4.3 to UART Receive (RXD)

}

void use_9600_baud(void)
{
UCA0CTLW0 = UCSWRST; // Put UART into SoftWare ReSeT
UCA0CTLW0 = UCA0CTLW0 | UART_CLK_SEL; // Specifies clock source for UART
UCA0BR0 = BR0_FOR_9600; // Specifies bit rate (baud) of 9600
UCA0BR1 = BR1_FOR_9600; // Specifies bit rate (baud) of 9600
UCA0MCTLW = CLK_MOD; // "Cleans" clock signal
UCA0CTLW0 = UCA0CTLW0 & (~UCSWRST); // Takes UART out of SoftWare ReSeT
}

void lock_control(void)
{
    P1DIR &= 0x0F;                         //input

    P1IE |= 0xF0;                          //enable interrupt
    P1IES |= 0xF0;                         //enable interrupt on falling edge
    P1IFG &= 0x0F;                         //clear IFG

    P4DIR |= BIT7;
    P4OUT &= ~BIT7;                         //lock


}

void timer_setup(void)
{
    TA0CCR0 = 1000;             //100% duty cycle
    TA0CCTL1 = OUTMOD_7;        //set output mode
    TA0CCR1 = 0;                //0% duty cycle
    TA0CTL = TASSEL_2 + MC_1;   //set up timer

    P1SEL0 |= BIT0;              //enable pwm
    P1DIR |= BIT0;

}

unsigned int ADC_value0 = 0;
unsigned int ADC_value1 = 0;
unsigned int weight1 = 0;
unsigned int weight2 = 0;

void ADC_SETUP(void)
{
    P9SEL1 |= BIT1; // Configure P9.1 for ADC
    P9SEL0 |= BIT1;

    P9SEL1 |= BIT2; // Configure P9.2 for ADC
    P9SEL0 |= BIT2;

 #define ADC12_SHT_16 0x0200 // 16 clock cycles for sample and hold
 #define ADC12_ON 0x0010 // Used to turn ADC12 peripheral on
 #define ADC12_SHT_SRC_SEL 0x0200 // Selects source for sample & hold
 #define ADC12_12BIT 0x0020 // Selects 12-bits of resolution
 #define ADC12_P92 0x000A // Use input P9.2 for analog input
#define ADC12_P91  0x0009 //Use input P9.1 for second input
 ADC12CTL0 = ADC12_SHT_16 | ADC12MSC | ADC12_ON ; // Turn on, set sample & hold time
 ADC12CTL1 = ADC12CONSEQ1 + ADC12_SHT_SRC_SEL; // Specify sample & hold clock source
 ADC12CTL2 = ADC12_12BIT; // 12-bit conversion results
 ADC12MCTL0 = ADC12_P92; // P9.2 is analog input
 ADC12MCTL1 = ADC12_P91 | ADC12EOS; //P9.1 second input, end of sequence
 ADC12IER0 |= ADC12IE1; // Enable ADC interrupt on MEM1

}

void main()
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5;

    //Initializations
    Init_GPIO();
    Init_Clock();
    Init_LCD();
    ADC_SETUP(); // Sets up ADC peripheral
    assign_pins_to_uart(); // P4.2 is for TXD, P4.3 is for RXD
    use_9600_baud(); // UART operates at 9600 bits/second
    lock_control();
    timer_setup();


    while(1)
    {
            ADC12CTL0 |= ADC12ENC;
            ADC12CTL0 = ADC12CTL0 | ADC12SC; // Start conversion
            ADC12CTL1 |= ADC12ENC;
            ADC12CTL1 = ADC12CTL1 | ADC12SC; // Start conversion
            ADC_value0 = ADC12MEM0; // Save MEM0
            ADC_value1 = ADC12MEM1; // Save MEM1


                        weight1 = ADC_value0;
                        weight2 = ADC_value1;

                        if((weight1 >= 1700) || (weight2 >= 1700))
                        {
                            tape = 1;
                            screwdriver = 0;
                        }
                        if(weight2 >= 750 && weight2 <= 1700)
                        {
                            screwdriver = 1;
                        }
                        if(weight1 > 750 && weight1 < 1700)
                        {
                            screwdriver = 1;
                        }
                        if(weight1 < 750 && weight2 < 750)
                        {
                            screwdriver = 0;
                            tape = 0;
                        }


        if((~P1IN & 0x02)==0x02)
        {
            Init_Clock();

                        if(tape == 1 && screwdriver == 1)
                        {
                            displayScrollText("TAPE HERE PLIERS HERE");

                        }
                        else if(tape == 0 && screwdriver == 1)
                        {
                            displayScrollText("TAPE NOT HERE PLIERS HERE");

                        }
                        else if(tape == 1 && screwdriver == 0)
                        {
                            displayScrollText("TAPE HERE PLIERS NOT HERE");

                        }
                        else if(tape == 0 && screwdriver == 0)
                        {
                            displayScrollText("TAPE NOT HERE PLIERS NOT HERE");

                        }
        }
        select_clock_signals(); // Assigns microcontroller clock signals

                                if(tape == 1 && screwdriver == 1)
                                {

                                            UCA0TXBUF = 0x0011;
                                }
                                else if(tape == 0 && screwdriver == 1)
                                {

                                            UCA0TXBUF = 0x0001;
                                }
                                else if(tape == 1 && screwdriver == 0)
                                {

                                            UCA0TXBUF = 0x0010;
                                }
                                else if(tape == 0 && screwdriver == 0)
                                {

                                            UCA0TXBUF = 0x0000;
                                }

    if ((P1IN & 0xF0)==0x90) // if button only 2 and 3 are pressed
    {
        P4OUT |= BIT7;                          //unlock
        P1IFG &= 0x0F;                         //clear IFG
        __delay_cycles(10000000);                //delay before locking again
        P4OUT &= ~BIT7;
    }
    if ((P1IN & 0xF0)==0x10 || (P1IN & 0xF0)==0x20 || (P1IN & 0xF0)==0x30 || (P1IN & 0xF0)==0x40 || (P1IN & 0xF0)==0x50 || (P1IN & 0xF0)==0x60 || (P1IN & 0xF0)==0x70 || (P1IN & 0xF0)==0x80 || (P1IN & 0xF0)==0xA0 || (P1IN & 0xF0)==0xB0 || (P1IN & 0xF0)==0xC0 || (P1IN & 0xF0)==0xD0 || (P1IN & 0xF0)==0xE0 || (P1IN & 0xF0)==0x00)
        {
            timer_setup();
            TA0CCR1 = 1000;
            __delay_cycles(3000000);
            TA0CCR1 = 0;
            select_clock_signals(); // Assigns microcontroller clock signals
        }
   }
}


/*
 * GPIO Initialization
 */
void Init_GPIO()
{
    // Set all GPIO pins to output low to prevent floating input and reduce power consumption
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P9, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P9, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    GPIO_setAsInputPin(GPIO_PORT_P3, GPIO_PIN5);

    // Configure button S1 (P1.1) interrupt
    GPIO_selectInterruptEdge(GPIO_PORT_P1, GPIO_PIN1, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);
    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN1);
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1);

    // Configure button S2 (P1.2) interrupt
    GPIO_selectInterruptEdge(GPIO_PORT_P1, GPIO_PIN2, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN2);
    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN2);
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN2);

    // Set P4.1 and P4.2 as Secondary Module Function Input, LFXT.
    GPIO_setAsPeripheralModuleFunctionInputPin(
           GPIO_PORT_PJ,
           GPIO_PIN4 + GPIO_PIN5,
           GPIO_PRIMARY_MODULE_FUNCTION
           );

    // Disable the GPIO power-on default high-impedance mode
    // to activate previously configured port settings
    PMM_unlockLPM5();
}

/*
 * Clock System Initialization
 */
void Init_Clock()
{
    // Set DCO frequency to default 8MHz
    CS_setDCOFreq(CS_DCORSEL_0, CS_DCOFSEL_6);

    // Configure MCLK and SMCLK to default 2MHz
    CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_8);
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_8);

    // Intializes the XT1 crystal oscillator
    CS_turnOnLFXT(CS_LFXT_DRIVE_3);
}

