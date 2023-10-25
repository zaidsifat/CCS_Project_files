// SPWM with CMPA value from Function generator(ADC) for 1phase VSI

// Included Files
#include "F2837xD_device.h"
#include "F2837xD_Examples.h"
#include "F28x_Project.h"
#include "math.h"

//defines.
#define TB_CLK    100e6                 //TBCLK = EPWMCLK = SYSCLK/2 (default); SYSCLK = 200 MHz
#define PWM_CLK   10e3                   // 10kHz carrier frequency/Switching Frequency
int TB_Period=TB_CLK/(2*PWM_CLK);      //TB_Period = TB_CLK/(2*PWM_CLK)
#define RESULTS_BUFFER_SIZE 240        //  200=(20ms/100us)
float makeanalog=0.732e-3;
double VdcHi=2.7;
double VdcLow=2;
double Vdc;

// Globals

int adcvalue1;
int adcvalue2;
Uint16 resultsIndex;
float32 Adcresultsstore1[RESULTS_BUFFER_SIZE];
float32 Adcresultsstore2[RESULTS_BUFFER_SIZE];
float32 Adcresultsstore3[RESULTS_BUFFER_SIZE];
// Function Prototypes
void GPIOConfig(void);
void EPwmConfig(void);
void ConfigureADC(void);
void SetupADCEpwm(void);
void readADCdata(void);
void SaveData(void);
interrupt void adca1_isr(void);

// Main

void main(void)
{
     //1. Initialize System Control: PLL, WatchDog, disable Peripheral Clocks (F2837xD_SysCtrl.c)
      InitSysCtrl();  // SYSCLK = 200 MHz (Using INTOSC2 = 10 MHz)

    //2. Initialize GPIO: [CPU1 only] Disable pin locks, Fill all (6 GPIO ports) registers with zeros.(F2837xD_Gpio.c)
    InitGpio();

    // GPIO muxed Peripherals
    GPIOConfig();

    DINT;
    //3. Clear all __interrupts and initialize PIE vector table Disable CPU __interrupts (F2837xD_device.h)
    //DINT;

    //Initialize PIE control registers to their default state. The default state is all PIE __interrupts disabled and flags are cleared (F2837xD_PieCtrl.c)
    InitPieCtrl();

    // Disable CPU __interrupts and clear all CPU __interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    InitPieVectTable();

    EALLOW;
    PieVectTable.ADCA1_INT=&adca1_isr;    // Function for ADCA interrupt 1
    EDIS;

    //Required Interrupts are re-mapped to ISR functions.

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC =0; // STOP counter TBCTR
    // enable peripheral clocks
    CpuSysRegs.PCLKCR2.bit.EPWM1=1;
  //  CpuSysRegs.PCLKCR2.bit.EPWM2=1;
    CpuSysRegs.PCLKCR13.bit.ADC_A = 1;
    EDIS;
    EPwmConfig();
    ConfigureADC();


    SetupADCEpwm();
    IER |= M_INT1;                      // Enable group 1 interrupts for adc1
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;







    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC =1; // When set PWM time bases of all the PWM modules belonging to the same CPU-Subsystem (as partitioned using their CPUSEL bits) start counting.
    EDIS;

    EINT;
    ERTM;


for(;;);



}



void GPIOConfig(void)
{
    EALLOW; // This is needed to write to EALLOW protected registers

    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1; // Disable pull-up to reduce power consumption
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1; // GPIO0 selected as EPWM1A (pin40) in GPIO and Peripheral Muxing

       GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;
       GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1; // GPIO1 == EPWM1B pin39

       GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;
       GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1; // GPIO2 == EPWM2A pin38

       GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;
       GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1; // GPIO3 == EPWM2B pin37


    EDIS; // This is needed to disable write to EALLOW protected registers
}

void EPwmConfig(void)
{
    EALLOW;
    // TPWM = 2 * TBPRD * TBCLK
    EPwm1Regs.TBPRD = TB_Period;                  // Set Timer period
    //EPwm1Regs.CMPA.bit.CMPA= 1.22*adcvalue;
    //  EPwm1Regs.CMPA.bit.CMPA= dutyratio1*TB_Period;
    EPwm1Regs.TBPHS.bit.TBPHS = 0x0000;           // Set Phase = 0, If TBCTL[PHSEN] = 1, then the TBCTR will be loaded with the TBPHS when a synchronization event occurs.
    EPwm1Regs.TBCTR = 0x0000;                                 // Clear Counter
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;        // Count up-down
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;             // Disable phase shift
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;        // TBCLK = EPWMCLK/(HSPCLKDIV * CLKDIV)
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;

    // Active high complementary PWMs - Setup the dead band // DeadBand = 0.5 us
       EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; // S1=1, S0=1
       EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; // S3=1, S2=0
       EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL; // S5=0, S4=0
       EPwm1Regs.DBRED.bit.DBRED = 100; // RED = DBRED × T_TBCLK // RED = 0.5us; T_TBCLK = EPWMCLK = 100 MHz
       EPwm1Regs.DBFED.bit.DBFED = 100; // FED = DBFED × T_TBCLK // Falling Edge Delay


    //**********ADC Triger setting******************
    EPwm1Regs.ETSEL.bit.SOCAEN  = 0;      // Disable SOC on A group
    EPwm1Regs.ETSEL.bit.SOCASEL = 3;      // Select SOC on zero and TBPRD
    EPwm1Regs.ETSEL.bit.SOCAEN  = 1;      // Enable SOC on A group
    EPwm1Regs.ETPS.bit.SOCAPRD = 1;       // Generate pulse on every 1st event


    EDIS;
}

void ConfigureADC(void)

{

//************write configurations*************************
    EALLOW;
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6;  //set ADCCLK divider to /4  TRM page no 1597
    AdcaRegs.ADCCTL2.bit.RESOLUTION = 0;        // 12-bit resolution
    AdcaRegs.ADCCTL2.bit.SIGNALMODE = 0;        // Single-ended channel conversions (12-bit mode only)
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;      // Occurs at the end of the conversion
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;       //power up the ADC

   // DELAY_US(1000);                    //delay for 1ms to allow ADC time to power up

   EDIS;

}

void SetupADCEpwm(void)
{

Uint16 acqps=30;

//*****Select the channels to convert and end of conversion flag******************

    EALLOW;
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;     //SOC0 will convert pin A0 ADCINA0(pin30)  1=A1 pin(29)
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps;  //sample window is (acqps+1)SYSCLK cycles
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5;       //trigger on ePWM1 SOCA/C

    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 2;          // SOC1 will convert pin A2(pin no 29)
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = acqps;      // SOC1 will use sample duration of 20 SYSCLK cycles
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 5;       // Trigger on ePWM1 SOCA/C

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0;      //end of SOC0 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;        //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;      //make sure INT1 flag is cleared
    EDIS;

}



interrupt void adca1_isr(void)
{
  readADCdata();
  SaveData();
  AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;      // Clear ADC INT1 flag
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;     // Acknowledge PIE group 1 to enable further interrupts

}

void readADCdata(void)
{

adcvalue1=AdcaResultRegs.ADCRESULT0;    // why value in RESULT0 bcz I use SOC0
EPwm1Regs.CMPA.bit.CMPA= 3*1.2*adcvalue1;



void SaveData(void)
{
               Adcresultsstore1[resultsIndex] =makeanalog*(AdcaResultRegs.ADCRESULT0);
               Adcresultsstore2[resultsIndex] =AdcaResultRegs.ADCRESULT0;
               Adcresultsstore3[resultsIndex] =1.2*adcvalue1;

               resultsIndex++;

                if(RESULTS_BUFFER_SIZE <= resultsIndex)
                   {
                       resultsIndex = 0;
                   }
           }



