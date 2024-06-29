#include "tm4c123gh6pm.h"
#include <stdint.h>

//Port B
#define GPIO_PORTB_OUT (*((volatile unsigned long *)0x400053FC)) //bits 7-0 
#define GPIO_PORTB_DIR_R (*((volatile unsigned long *)0x40005400))
#define GPIO_PORTB_AFSEL_R (*((volatile unsigned long *)0x40005420))
#define GPIO_PORTB_DEN_R (*((volatile unsigned long *)0x4000551C))
#define GPIO_PORTB_AMSEL_R (*((volatile unsigned long *)0x40005528))
#define GPIO_PORTB_PCTL_R (*((volatile unsigned long *)0x4000552C))
	
//Port A
#define GPIO_PORTA_IN (*((volatile unsigned long *)0x400043FC)) //bits 3-2 
#define GPIO_PORTA_DATA_R (*((volatile unsigned long *)0x400043FC))
#define GPIO_PORTA_DIR_R  (*((volatile unsigned long *)0x40004400))
#define GPIO_PORTA_AFSEL_R (*((volatile unsigned long *)0x40004420))
#define GPIO_PORTA_PUR_R (*((volatile unsigned long *)0x40004510))
#define GPIO_PORTA_DEN_R (*((volatile unsigned long *)0x4000451C))
#define GPIO_PORTA_LOCK_R (*((volatile unsigned long *)0x40004520))
#define GPIO_PORTA_CR_R (*((volatile unsigned long *)0x40004524))
#define GPIO_PORTA_AMSEL_R (*((volatile unsigned long *)0x40004528))
#define GPIO_PORTA_PCTL_R (*((volatile unsigned long *)0x4000452C))
#define GPIO_PORTA_ODR_R (*((volatile unsigned long *)0x4000450C))
#define GPIO_PORTA_IS_R (*((volatile unsigned long *)0x40004404))
#define GPIO_PORTA_IBE_R (*((volatile unsigned long *)0x40004408))
#define GPIO_PORTA_ICR_R (*((volatile unsigned long *)0x4000441C))
#define GPIO_PORTA_IM_R (*((volatile unsigned long *)0x40004410))
#define NVIC_PRI0_R (*((volatile unsigned long *)0xE000E400))

//Port F
#define GPIO_PORTF_DATA_R (*((volatile unsigned long *)0x400253FC))
#define GPIO_PORTF_DIR_R (*((volatile unsigned long *)0x40025400))
#define GPIO_PORTF_AFSEL_R (*((volatile unsigned long *)0x40025420))
#define GPIO_PORTF_PUR_R (*((volatile unsigned long *)0x40025510))
#define GPIO_PORTF_DEN_R (*((volatile unsigned long *)0x4002551C))
#define GPIO_PORTF_LOCK_R (*((volatile unsigned long *)0x40025520))
#define GPIO_PORTF_CR_R (*((volatile unsigned long *)0x40025524))
#define GPIO_PORTF_AMSEL_R (*((volatile unsigned long *)0x40025528))
#define GPIO_PORTF_PCTL_R (*((volatile unsigned long *)0x4002552C))
#define GPIO_LOCK_KEY           0x4C4F434B

//SysTick Interrupts
#define NVIC_SYS_PRI3_R  (*((volatile unsigned long *)0xE000ED20))
#define NVIC_ST_CTRL_R  (*((volatile unsigned long *)0xE000E010))
#define NVIC_ST_RELOAD_R  (*((volatile unsigned long *)0xE000E014))
#define NVIC_ST_CURRENT_R  (*((volatile unsigned long *)0xE000E018))
#define NVIC_ST_CTRL_COUNT      0x00010000  // Count flag
#define NVIC_ST_CTRL_CLK_SRC    0x00000004  // Clock Source
#define NVIC_ST_CTRL_INTEN      0x00000002  // Interrupt enable
#define NVIC_ST_CTRL_ENABLE     0x00000001  // Counter mode
#define NVIC_ST_RELOAD_M        0x00FFFFFF  // Counter load value
	
#define NVIC_EN0_R (*((volatile unsigned long *)0xE000E100))
#define SYSCTL_RCGC2_R (*((volatile unsigned long *)0x400FE108))
#define IRSENSOR (*((volatile unsigned long *)0x40004010)) 
#define RESET (*((volatile unsigned long *)0x40025004)) 
#define Motor (*((volatile unsigned long *)0x400053FC)) 
#define SYSCTL_RCGC2_GPIOB 0x00000002 //port B Clock Gating Control
#define SYSCTL_RCGC2_GPIOA 0x00000001 //port A Clock Gating Control
#define PD0 (*((volatile unsigned long *)0x40007004))
#define PD1 (*((volatile unsigned long *)0x40007008))
#define PD2 (*((volatile unsigned long *)0x40007010))
#define Stepper (*((volatile unsigned long *)0x4002401C))
#define PF0 (*((volatile unsigned long *)0x40025004))
#define PF4 (*((volatile unsigned long *)0x40025008))

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode

void Ports_Init(void);
void SysTick_Init(void);
void SysTick_Handler (void);
void GPIOPortA_Handler(void);
void Delay(void);
void SysTick_Wait10ms(int delay);

struct State {
 unsigned long Out; 
 unsigned long Time; 
 unsigned long Next[8];}; 
typedef const struct State STyp;
#define Wait 0
#define St1 1
#define St2 2
#define St3 3
#define St4 4
 
STyp FSM[5]={	
{0x00,0x80000,{Wait,Wait,Wait,St1,Wait,St4,Wait,Wait}}, 
{0x39,0x80000,{St4,St2,Wait,St1,St1,St1,St1,St1}},	
{0x6C,0x80000,{St1,St3,Wait,St2,St2,St2,St2,St2}}, 
{0xC6,0x80000,{St2,St4,Wait,St3,St3,St3,St3,St3}}, 
{0x93,0x80000,{St3,St1,Wait,St4,St4,St4,St4,St4}}, 
};

unsigned long i; //state index for iteration 
unsigned long Input; 
unsigned long In;
unsigned long Rep;


void EdgeCounter_Init(void){                          
  DisableInterrupts();
	//Port A
	GPIO_PORTA_IS_R &= ~0x0C;     // (d) edge-sensitive
  GPIO_PORTA_IBE_R &= 0x0C;    
  GPIO_PORTA_ICR_R = 0x0C;      // (e) clear flag
  GPIO_PORTA_IM_R |= 0x0C;      // (f) arm interrupt on PA2-3
  NVIC_PRI1_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00400000; // (g) priority 2
	
	NVIC_EN0_R |= 0x00000011;
	EnableInterrupts();
}

int main(void){ 
	Ports_Init();
	EdgeCounter_Init();
	SysTick_Init();
	i = Wait;
	Rep = 0;
	//GPIO_PORTF_DATA_R = 0x00;
	while(1){
		Motor = FSM[i].Out;
		Delay();
		//In = 3;
		//In = Stepper;
		//In = (4*PD2)+(2*PD1)+PD0;
		if (GPIO_PORTF_DATA_R == 0x01)
		{
			In = 3;
			Rep = (2000)*4.5;
		}
		else if (GPIO_PORTF_DATA_R == 0x10)
		{
			In = 5;
			Rep = 800;
		}
		i = FSM[i].Next[In];
		if (In == 3)
		{
			while(In == 3)
			{
				while (Rep != 0)
				{
					Motor = FSM[i].Out;
					Delay();
					i = FSM[i].Next[1];
					Rep --;
				}
				In = 2;
				i = FSM[i].Next[In]; 
				Motor = FSM[i].Out;
				Delay();
			}
		}
		else if (In == 5)
		{
			while(In == 5)
			{
				while (Rep != 0)
				{
					Motor = FSM[i].Out;
					Delay();
					i = FSM[i].Next[0];
					Rep --;
				}
				In = 2;
				i = FSM[i].Next[In]; 
				Motor = FSM[i].Out;
				Delay();
			}
		}
	 //WaitForInterrupt();
	} 
}

void Ports_Init(void){
	volatile unsigned long delay;
	SYSCTL_RCGC2_R = 0x3B; // 1) Ports A B D 
	delay = SYSCTL_RCGC2_R; // 2) no need to unlock
	
	//Port A
	GPIO_PORTA_DATA_R = 0x400043FC;
	GPIO_PORTA_CR_R = 0x1C;
	GPIO_PORTA_AMSEL_R &= 0x00; // 3) disable analog function on PA3-2 
	GPIO_PORTA_PCTL_R &= 0x00000000; // 4) enable regular GPIO
	GPIO_PORTA_DIR_R &= 0x00; // 5) inputs on PA3-2 
	GPIO_PORTA_AFSEL_R &= 0x00; // 6) regular function on PA3-2 
	GPIO_PORTA_DEN_R |= 0x0C; // 7) enable digital on PA3-2	
	GPIO_PORTA_ODR_R |= 0x38;
	
	//Port F
	GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock GPIO Port F
	GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0
	GPIO_PORTF_AMSEL_R = 0x00;        // 3) disable analog on PF
	GPIO_PORTF_PCTL_R = 0x00000000;   // 4) PCTL GPIO on PF4-0
	GPIO_PORTF_DIR_R = 0x0E;          // 5) PF4,PF0 in, PF3-1 out
	GPIO_PORTF_AFSEL_R = 0x00;        // 6) disable alt funct on PF7-0
	GPIO_PORTF_PUR_R = 0x11;          // enable pull-up on PF0 and PF4
	GPIO_PORTF_DEN_R = 0x1F;          // 7) enable digital I/O on PF4-0
	
	//Port B
	GPIO_PORTB_AMSEL_R &= ~0xFF; 
	GPIO_PORTB_PCTL_R &= ~0x00FFFFFF; // 4) enable regular GPIO
	GPIO_PORTB_DIR_R |= 0xFF; // 5) outputs on PB5-0 
	GPIO_PORTB_AFSEL_R &= ~0xFF; // 6) regular function on PB5-0 
	GPIO_PORTB_DEN_R |= 0xFF; // 7) enable digital on PB5-0

}

void SysTick_Handler() 
{
	In = 2;
	i = FSM[i].Next[In]; 
	Motor = FSM[i].Out;
	Delay();
}
void GPIOPortA_Handler()
{
	In = 2;
	i = FSM[i].Next[In]; 
	Motor = FSM[i].Out;
	Delay();
}

void Delay(){
	volatile uint32_t time;\
	time = 727240*200/(91*200*3);
	while(time){
		time--;
	}
}
void SysTick_Init(void){
  NVIC_ST_CTRL_R = 0;                   // disable SysTick during setup
  NVIC_ST_RELOAD_R = NVIC_ST_RELOAD_M;  // maximum reload value
  NVIC_ST_CURRENT_R = 0;                // any write to current clears it
  NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x00FFFFFF) | 0x60000000; //priority 3
  NVIC_ST_CTRL_R = NVIC_ST_CTRL_ENABLE+NVIC_ST_CTRL_CLK_SRC;
}
// Time delay using busy wait.
// The delay parameter is in units of the core clock. (units of 20 nsec for 50 MHz clock)
void SysTick_Wait(unsigned long delay){
  volatile unsigned long elapsedTime;
  unsigned long startTime = NVIC_ST_CURRENT_R;
  do{
    elapsedTime = (startTime-NVIC_ST_CURRENT_R)&0x00FFFFFF;
  }
  while(elapsedTime <= delay);
}
// Time delay using busy wait.
// This assumes 50 MHz system clock.
void SysTick_Wait10ms(int delay){
  unsigned long i;
  for(i=0; i<delay; i++){
    SysTick_Wait(500000);  // wait 10ms (assumes 50 MHz clock)
  }
}

