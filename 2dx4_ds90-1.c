/* 
Saffi Haider, 400255941, haides23

2DX4 Final Project: 3D Spacial Mapping Softtware
Final submission: April 15th, 2021
*/

// Import libraries
#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "vl53l1x_api.h"
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"

// Initialize global variables
uint16_t	dev=0x52;
int scanning = 0;
int status=0;
volatile int IntCount;

// Set isInterrupt to 1 (true) in order to use GPIO interrupt
#define isInterrupt 1 

// Function prototypes
void I2C_Init(void);
void UART_Init(void);
void PortG_Init(void);
void VL53L1X_XSHUT(void);


// Enable interrupts
void EnableInt(void)
{    __asm("    cpsie   i\n");
}

// Disable interrupts
void DisableInt(void)
{    __asm("    cpsid   i\n");
}


// Low power wait
void WaitForInt(void)
{    __asm("    wfi\n");
}
// Counter for interrupt service routine
volatile unsigned long FallingEdges = 0;

// Initialize PJ1 onboard button for use as GPIO interrupt
void ExternalButton_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8; // Activate clock for Port J
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};	// Allow time for clock to stabilize
  FallingEdges = 0;             // Initialize counter
  GPIO_PORTJ_DIR_R &= ~0x02;    // Make PJ1 input

  GPIO_PORTJ_DEN_R |= 0x02; // Enable digital I/O on PJ1
	GPIO_PORTJ_PCTL_R &= ~0x000000F0; // Configure PJ1 as GPIO 
	GPIO_PORTJ_AMSEL_R &= ~0x02; // Disable analog functionality on PJ1		
	GPIO_PORTJ_PUR_R |= 0x02; // Enable weak pull up resistor
  GPIO_PORTJ_IS_R &= ~0x02; // PJ1 is edge-sensitive 
  GPIO_PORTJ_IBE_R &= ~0x02; // PJ1 is not both edges 
  GPIO_PORTJ_IEV_R &= ~0x02; // PJ1 falling edge event 
  GPIO_PORTJ_ICR_R = 0x02; // Clear flag1
  GPIO_PORTJ_IM_R |= 0x02; // Arm interrupt on PJ1
  NVIC_PRI13_R = (NVIC_PRI13_R&0xFF00FFFF)|0x000A0000; // Priority 5
  NVIC_EN1_R |= 0x00080000; // Enable interrupt 67 in NVIC
  EnableInt(); // Enable interrupts now that button is configured
}


// Interrupt service routine to start measurements
void GPIOJ_IRQHandler(void){
	GPIO_PORTJ_ICR_R = 0x02; // acknowledge flag4
	FallingEdges = FallingEdges + 1; // Increment counter
	if (scanning == 0) { // If boolean value of scanning is false, set it to true
		scanning = 1;
	}
	SysTick_Wait10ms(50); // 0.5s delay to prevent double button press
}

void PortE_Init(void){
SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4; // Activate the clock for Port E
while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R4) == 0){}; // Allow time for clock to stabilize
	GPIO_PORTE_DEN_R = 0b11111111; 
	GPIO_PORTE_DIR_R = 0b00001111;
return;
}
void PortM_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11; // Activate the clock for Port M
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){}; // Allow time for clock to stabilize 
	GPIO_PORTM_DIR_R = 0b00000000;
  GPIO_PORTM_DEN_R = 0b00000000;
	return;
}

// Turn on LED D1
void PortN1_Init(void){
SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12; // Activate the clock for Port N
while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R12) == 0){};// Allow time for clock to stabilize
GPIO_PORTN_DIR_R=0b00000010; // Make PN1 output to turn on LED D1
GPIO_PORTN_DEN_R=0b00000010; // Enable PN1
return;
}

void PortK_Init(void){
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R9; // Activate the clock for Port K
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R9) == 0){}; // Allow time for clock to stabilize
	GPIO_PORTK_DIR_R=0xFF; // Set all pins as output to control stepper motor
	GPIO_PORTK_AFSEL_R &= ~0xFF;
	GPIO_PORTK_DEN_R=0xFF; // Enable all pins 
		
	GPIO_PORTK_AMSEL_R &= ~0xFF;    
	return;
}


// Drive the stepper motor in the clockwise direction
void driveStep(int delay) {
	GPIO_PORTK_DATA_R = 0b00001100;
	SysTick_Wait(delay);
	GPIO_PORTK_DATA_R = 0b00000110;
	SysTick_Wait(delay);
	GPIO_PORTK_DATA_R = 0b00000011;
	SysTick_Wait(delay);
	GPIO_PORTK_DATA_R = 0b00001001;
	SysTick_Wait(delay);
}


// Drive the stepper motor in the counterclockwise direction
void reverse(int delay) {
	GPIO_PORTK_DATA_R = 0b00001001;
	SysTick_Wait(delay);
	GPIO_PORTK_DATA_R = 0b00000011;
	SysTick_Wait(delay);
	GPIO_PORTK_DATA_R = 0b00000110;
	SysTick_Wait(delay);
	GPIO_PORTK_DATA_R = 0b00001100;
	SysTick_Wait(delay);
}


// Turns LED D1 on if mode = 1, or off if mode = 0
void ControlLED1(int mode) {
		if (mode) {
			GPIO_PORTN_DATA_R = 0b00000010;
		}
		else {
			GPIO_PORTN_DATA_R = 0b00000000; 
		}
}

int main(void) {
  uint8_t byteData, sensorState = 0;
  uint16_t Distance;
  uint8_t RangeStatus;
  uint8_t dataReady;

	// Call all initialization functions
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	PortE_Init();
	PortM_Init();
	PortN1_Init();
	PortK_Init();
	ExternalButton_Init();
	float angle = 0;
	int count = 0;
	
	while (1) {	
		WaitForInt(); // Remain in low power mode until interrupt triggers data collection
		if (scanning) { // If the interrupt set scanning to true
			DisableInt(); // Disable interrupts to prevent double-press
			SysTick_Wait10ms(10);
			scanning = 0; // Reset the scanning variable to false
			
			// Booting ToF chip
			while(sensorState==0){
				status = VL53L1X_BootState(dev, &sensorState);
				SysTick_Wait10ms(10);
			}
			status = VL53L1X_ClearInterrupt(dev); // Clear interrupt has to be called to enable next interrupt
			// This function must to be called to initialize the sensor with the default setting 
			status = VL53L1X_SensorInit(dev);
			Status_Check("SensorInit", status);

			status = VL53L1X_StartRanging(dev); // This function has to be called to enable the ranging 
			Status_Check("StartRanging", status);

			UART_OutChar('*'); // Start one cycle of python script
			for(int i = 0; i < 512; i++) { // Drive stepper motor for one full rotation
				if (i % 2 == 0) {
					while (dataReady == 0){ // Wait for dataReady flag to be true
						status = VL53L1X_CheckForDataReady(dev, &dataReady);
						VL53L1_WaitMs(dev, 5);
					}

					dataReady = 0; // Reset dataReady flag
					status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
					status = VL53L1X_GetDistance(dev, &Distance);
					ControlLED1(1); // Enable LED since distance measurements are being made
					
					status = VL53L1X_ClearInterrupt(dev); // Clear interrupt has to be called to enable next interrupt
					angle = (float)i/512.0*360.0; // Calculate angle
					sprintf(printf_buffer,"%u, %f\r\n", Distance, angle); // Print to buffer
					UART_printf(printf_buffer); // Output buffer by UART
				}
				driveStep(192000); // Drive stepper motor for 1 step
			}
			ControlLED1(0); // Turn LED off since distance measurements are complete
			for (int i = 0 ; i < 512; i++) {
				reverse(192000); // Reverse motor back to starting position
			}
		}
		EnableInt(); // Enable interrupts for the next measurement
	}
	VL53L1X_StopRanging(dev);
}

#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // Number of receive attempts before giving up

// Initialize I2C communications
void I2C_Init(void){
	SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0; // activate I2C0
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1; // activate port B
	while((SYSCTL_PRGPIO_R&0x0002) == 0){};
	GPIO_PORTB_AFSEL_R |= 0x0C;// 3) enable alt funct on PB2
	GPIO_PORTB_ODR_R |= 0x08;// 4) enable open drain on PB3 only
	GPIO_PORTB_DEN_R |= 0x0C;// 5) enable digital I/O on PB2,3
	GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;
	I2C0_MCR_R = I2C_MCR_MFE; // 9) master function enable
	I2C0_MTPR_R = 0b0000000000000101000000000111011;// 8) configure for 100 kbps clock 
	}
void PortG_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6; // Activate clock for Port G
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};// Allow time for clock to stabilize
	GPIO_PORTG_DIR_R &= 0x00; // Make PG0 in (HiZ)
	GPIO_PORTG_AFSEL_R &= ~0x01;// Disable alt funct on PG0
	GPIO_PORTG_DEN_R |= 0x01;// Enable digital I/O on PG0
	GPIO_PORTG_AMSEL_R &= ~0x01;// Disable analog functionality on PN0
	return;
}


// Active-low shutdown input for the ToF sensor
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01; // Make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110; //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01; // Make PG0 input (HiZ)
    
}
