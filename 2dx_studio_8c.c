// Prawin Premachandran

#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"

#define I2C_MCS_ACK 0x00000008
#define I2C_MCS_DATACK 0x00000008
#define I2C_MCS_ADRACK 0x00000004
#define I2C_MCS_STOP 0x00000004
#define I2C_MCS_START 0x00000002
#define I2C_MCS_ERROR 0x00000002
#define I2C_MCS_RUN 0x00000001
#define I2C_MCS_BUSY 0x00000001
#define I2C_MCR_MFE 0x00000010

#define MAXRETRIES 5 // number of receive attempts before giving up
void I2C_Init(void){ //initializing I2c
SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0; // activate I2C0
SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1; // activate port B
while ((SYSCTL_PRGPIO_R & 0x0002) == 0){}; // ready?

GPIO_PORTB_AFSEL_R |= 0x0C; // 3) enable alt funct on PB2,3       0b00001100
GPIO_PORTB_ODR_R |= 0x08; // 4) enable open drain on PB3 only

GPIO_PORTB_DEN_R |= 0x0C; // 5) enable digital I/O on PB2,3
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & 0xFFFF00FF) + 0x00002200; // TED
I2C0_MCR_R = I2C_MCR_MFE;   // 9) master function enable
I2C0_MTPR_R = 0b0000000000000101000000000111011;// 8) configure for 100 kbps clock
//    I2C0_MTPR_R = 0x3B;                                         // 8) configure for 100 kbps clock
}

uint16_t dev = 0x29; // address of the ToF sensor as an I2C slave peripheral
int status = 0;

// Function Prototypes
void EnableInt(void);
void DisableInt(void);
void WaitForInt(void);
void PortJ_Init(void);
void PortM_Init(void);
void I2C_Init(void);
void scanRoom(void);
void rotate(int num, int dir);
void takeMeasurement(void);
void GPIOJ_IRQHandler(void);

// Interrupt Management
void EnableInt(void){__asm("cpsie i"); }
void DisableInt(void){__asm("cpsid i"); }
void WaitForInt(void){__asm("wfi"); }

// Port Initialization Functions
// Init onboard button (Interrupt Driven Button)
void PortJ_Init(void)
{
SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8; // Activate clock for Port J
while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R8) == 0){};// time for clock stabilize
GPIO_PORTJ_DIR_R &= ~0x02; // Make PJ1 input
GPIO_PORTJ_DEN_R |= 0x02;  // Enable digital I/O on PJ1
GPIO_PORTJ_PCTL_R &= ~0x000000F0; //? Configure PJ1 as GPIO
GPIO_PORTJ_AMSEL_R &= ~0x02;  //??Disable analog functionality on PJ1
GPIO_PORTJ_PUR_R |= 0x02;  // Enable weak pull up resistor on PJ1
// Interrupt Initialization, Arm the GPIO
GPIO_PORTJ_IS_R = 0; //          PJ1 is Edge-sensitive
GPIO_PORTJ_IBE_R = 0; //     PJ1 is not triggered by both edges
GPIO_PORTJ_IEV_R = 0; //     PJ1 is falling edge event
GPIO_PORTJ_ICR_R = 0x02; // Clear interrupt flag by setting proper bit in ICR register
GPIO_PORTJ_IM_R = 0x02; // Arm interrupt on PJ1 by setting proper bit in IM register

// Enable Interrupt in NVIC
NVIC_EN1_R = 0x00080000; //          Enable interrupt 51 in NVIC (which is in Register EN1)
// Set Interrupt Priority Level
NVIC_PRI12_R = 0xA0000000; //          Set interrupt priority to 5
// Enable Global Interrupts
EnableInt(); //          Enable Global Interrupt. lets go!
}

// Stepper Motor Control Initialization
void PortM_Init(void)
{
SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11; // Activate the clock for Port M
while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R11) == 0)
{
}; // Allow time for clock to stabilize

GPIO_PORTM_DIR_R |= 0b1111; // Enable PM0-3 as outputs
GPIO_PORTM_DEN_R |= 0b1111; // Enable PM0-3 digital pins
return;
}

// Application functions
void scanRoom()
{
uint16_t Distance;
uint16_t SignalRate;
uint16_t AmbientRate;
uint16_t SpadNum;
uint8_t RangeStatus;
uint8_t dataReady;

status = VL53L1X_StartRanging(dev); // Must be called to enable the ranging

//  wait till ToF sensor's data is ready
while (dataReady == 0){
status = VL53L1X_CheckForDataReady(dev, &dataReady);
FlashLED4(1); //2nd LSB is 7 --> PF0 primary
VL53L1_WaitMs(dev, 5);
}
dataReady = 0;

// read the data values from ToF sensor
status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
status = VL53L1X_GetDistance(dev, &Distance); // The Measured Distance value
status = VL53L1X_GetSignalRate(dev, &SignalRate);
status = VL53L1X_GetAmbientRate(dev, &AmbientRate);
status = VL53L1X_GetSpadNb(dev, &SpadNum);

FlashLED2(1); //secondary status, since my student # is 400468579, the 2nd LSB is 7 --> PN0 as additional measurement

status = VL53L1X_ClearInterrupt(dev); /* 8 clear interrupt has to be called to enable next interrupt*/

// print the resulted readings to UART
sprintf(printf_buffer, "%u\r\n", Distance);
UART_printf(printf_buffer);
SysTick_Wait10ms(50);

VL53L1X_StopRanging(dev);
}

void rotate(int num, int dir){
	if (dir > 0){
		for (int i = 1; i <= num; i++){
			GPIO_PORTM_DATA_R = 0b0011;
			SysTick_Wait10ms(1);
			GPIO_PORTM_DATA_R = 0b0110;
			SysTick_Wait10ms(1);
			GPIO_PORTM_DATA_R = 0b1100;
			SysTick_Wait10ms(1);
			GPIO_PORTM_DATA_R = 0b1001;
			SysTick_Wait10ms(1);
		}
	}
	else{
		for (int i = 1; i <= num; i++){
			GPIO_PORTM_DATA_R = 0b1001;
			SysTick_Wait10ms(1);
			GPIO_PORTM_DATA_R = 0b1100;
			SysTick_Wait10ms(1);
			GPIO_PORTM_DATA_R = 0b0110;
			SysTick_Wait10ms(1);
			GPIO_PORTM_DATA_R = 0b0011;
			SysTick_Wait10ms(1);
		}
	}
}

void takeMeasurement()
{
FlashAllLEDs(); // scan is starting now
for (int i = 1; i <= 32; i++)
{
rotate(16, 1);
scanRoom();
FlashLED4(1); //primary measurement status of Pf0 (student #400468579)
}
FlashAllLEDs(); //scan complete
// return home
rotate(512, -1);
}

// Interrupt Handler
void GPIOJ_IRQHandler(void)
{
SysTick_Wait10ms(40);
takeMeasurement();
GPIO_PORTJ_ICR_R = 0x02; // Acknowledge flag by setting proper bit in ICR register
}

int main(void)
{
uint8_t byteData, sensorState = 0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, i = 0;
uint16_t wordData;
uint16_t Distance;
uint16_t SignalRate;
uint16_t AmbientRate;
uint16_t SpadNum;
uint8_t RangeStatus;
uint8_t dataReady;

// initialize
PLL_Init();
SysTick_Init();
onboardLEDs_Init();
I2C_Init();
UART_Init();
PortJ_Init();
PortM_Init();
UART_printf("Program Begins\r\n\n");
int mynumber = 1;
sprintf(printf_buffer, "\nRoom Measuring Init %d\r\n", mynumber);
UART_printf(printf_buffer);

// 1 Wait for device ToF booted
while (sensorState == 0){
	status = VL53L1X_BootState(dev, &sensorState);
	SysTick_Wait10ms(10);
}
FlashAllLEDs();
UART_printf("ToF Chip Booted!\r\n Please Wait...\r\n");

status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/

/* 2 Initialize the sensor with the default setting  */
status = VL53L1X_SensorInit(dev);
Status_Check("SensorInit", status);

/* 3 Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */
status = VL53L1X_SetDistanceMode(dev, 2); /* 1=short, 2=long */
status = VL53L1X_SetTimingBudgetInMs(dev, 100); /* in ms possible values [20, 50, 100, 200, 500] */
status = VL53L1X_SetInterMeasurementInMs(dev, 200); /* in ms, IM must be > = TB */

status = VL53L1_RdByte(dev, 0x010F, &byteData);
sprintf(printf_buffer, " Model ID: %u\r\n", byteData);

UART_printf(printf_buffer);

status = VL53L1_RdByte(dev, 0x0110, &byteData); // for module type (0xCC)
sprintf(printf_buffer, "Module type: %u\r\n", byteData);
UART_printf(printf_buffer);

// ID
status = VL53L1X_GetSensorId(dev, &wordData);
sprintf(printf_buffer, "(Model_ID, Module_Type)=0x%x\r\n", wordData);
UART_printf(printf_buffer);

while (1){ //waiting for interrupts inside the loop (runs infinitely
	WaitForInt(); // Call WaitForInt()
}
}
