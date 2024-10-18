// BACK UP TODO CON UN LED PRENDIENDO Y APAGANDO

#include <stdio.h>
#include <stm32f7xx.h>
#include <string.h>

uint8_t flag = 0, i, cont = 0, flag2 = 0, bandera3 = 0;
unsigned char d;
char name[7] = "Fabian", text[10];
int inf;
uint8_t flagc = 0, ic;
unsigned char dc;
char textc[11];
uint16_t digital;
float voltaje;

#define CD 0x01
#define RH 0x02
#define EMS 0x06
#define DC 0x0F
#define DSr 0x1C
#define DSl 0x18
#define FS 0x28 // 0x38 (8 bits)
#define RAW1 0x80
#define RAW2 0xC0
#define TIME 10
//hola

char detenido8[] = {'D', 'E', 'T','E', 'N', 'I', 'D', 'O'};
char adelante8[] = {'A', 'D', 'E', 'L', 'A', 'N','T','E'};
char derecha7[] = {'D', 'E', 'R','E', 'C', 'H', 'A'};
char izquierda9[] = {'I', 'Z', 'Q', 'U', 'I', 'E','R','D','A'};
char reversa7[] = {'R', 'E', 'V', 'E', 'R', 'S','A'};
char obstaculo9[] = {'O', 'B', 'S', 'T', 'A', 'C','U','L','O'};

int k =0;

extern "C" {
		void SysTick_Handler() {
				k = 0;
		}
		
}

void delay() {
    k = 1;
    while (k == 1);
}

void LCD(unsigned char val) {
    GPIOC->ODR |= 1UL << 8;
    delay();
    GPIOG->ODR = val;
    delay();
    GPIOC->ODR &= ~(1UL << 8);
    delay();
}

void settingsLCD(unsigned char val) {
    GPIOC->ODR &= ~(1UL << 9);
    LCD(val >> 4);
    LCD(val & 0x0f);
}

void writeLCD(unsigned char val) {
    GPIOC->ODR |= 1UL << 9;
    LCD(val >> 4);
    LCD(val & 0x0f);
}

void settings() {
    RCC->AHB1ENR |= 0xFF; // Habilitar GPIO_A-H

    // Pines LCD
    GPIOG->MODER |= 0x55555555; // Salida
    GPIOG->OTYPER |= 0;
    GPIOG->OSPEEDR |= 0xAAAAAAAA;
    GPIOG->PUPDR |= 0xAAAAAAAA;

    GPIOC->MODER |= 0x00050000; // Salida
    GPIOC->OTYPER |= 0;
    GPIOC->OSPEEDR |= 0x000A0000;
    GPIOC->PUPDR |= 0x000A0000;

    // Configuración del botón "User" (en PC13)
    RCC->AHB1ENR |= 0x06; // Habilitar GPIO_B y GPIO_C

    GPIOB->MODER |= 0x10004001;
    GPIOB->OTYPER |= 0;
    GPIOB->OSPEEDR |= 0x20008002;
    GPIOB->PUPDR |= 0x10004001;



    // Systick
    SystemCoreClockUpdate(); // reloj
    SysTick_Config(SystemCoreClock / 1000); // tiempo

    // Configuración LCD
    settingsLCD(0x02);
    settingsLCD(EMS);
    settingsLCD(DC);
    settingsLCD(FS);
    settingsLCD(CD);
}







void Motor_Init(void);
void Motor_Forwarda(void);
void Motor_Backwarda(void);
void Motor_Forwardb(void);
void Motor_Backwardb(void);
void Motor_Stopa(void);
void Motor_Stopb(void);
void Motor_Stop(void);
void led_prueba(void);

void SysTick_Wait(uint32_t n){
    SysTick->LOAD = n - 1; //15999
    SysTick->VAL = 0; //Clean the value of Systick counter
    while (((SysTick->CTRL & 0x00010000) >> 16) == 0); //Check the count flag until it's 1 
}

void SysTick_ms(uint32_t x){
    for (uint32_t i = 0; i < x; i++){//x ms
        SysTick_Wait(16000); //1ms
    }
}

extern "C"{
    void EXTI15_10_IRQHandler(void){
        EXTI->PR |= 1; //Down flag
        if(((GPIOC->IDR & (1<<13)) >> 13) == 1){
            flag = 1;
					inf = 1;
        }
    }

    void USART3_IRQHandler(void){ //Receive interrupt
        if(((USART3->ISR & 0x20) >> 5) == 1){//Received data is ready to be read (flag RXNE = 1)
            d = USART3->RDR;//Read the USART receive buffer 
        }
    }
		void UART4_IRQHandler(){
			if(UART4->ISR&0x20){
				d=UART4->RDR;
			}
		}
		void TIM3_IRQHandler(void) {
        // Limpia el flag de actualización
        TIM3->SR &= ~(1<<0);

        // Código que se ejecutará cada 5 segundos
        // Cambia el estado del LED en PF0
        //GPIOF->ODR ^= (1<<0); // Toggle del LED
				led_prueba();
    }
}

void Motor_Init(void) {
    // Habilitar reloj para GPIOB
    RCC->AHB1ENR |= (1 << 1);
    // Configurar PB4 y PB5 como salidas
    GPIOB->MODER &= ~((0b11 << (4 * 2)) | (0b11 << (5 * 2)));
    GPIOB->MODER |= ((1 << (4 * 2)) | (1 << (5 * 2))); // Modo salida para PB4 y PB5
		GPIOB->MODER &= ~((0b11 << (2 * 7)) | (0b11 << (6 * 2)));
		GPIOB-> MODER |= ((1<<12) | (1<<14)); // PB6 PB7
}

void Motor_Forwarda(void) {
    // IN1 = 1, IN2 = 0 -> Adelante
    GPIOB->ODR |= (1 << 4);  // PB4 en HIGH
    GPIOB->ODR &= ~(1 << 5); // PB5 en LOW
}

void Motor_Backwarda(void) {
    // IN1 = 0, IN2 = 1 -> Atrás
    GPIOB->ODR &= ~(1 << 4); // PB4 en LOW
    GPIOB->ODR |= (1 << 5);  // PB5 en HIGH
}

void Motor_Stopa(void) {
    // IN1 = 0, IN2 = 0 -> Motor detenido
    GPIOB->ODR &= ~((1 << 4) | (1 << 5)); // PB4 y PB5 en LOW
}
void Motor_Forwardb(void) {
    // IN1 = 1, IN2 = 0 -> Adelante
    GPIOB->ODR |= (1 << 6);  // PB6 en HIGH
    GPIOB->ODR &= ~(1 << 7); // PB7 en LOW
}

void Motor_Backwardb(void) {
    // IN1 = 0, IN2 = 1 -> Atrás
    GPIOB->ODR &= ~(1 << 6); // PB6 en LOW
    GPIOB->ODR |= (1 << 7);  // PB7 en HIGH
}
void Motor_Stopb(void) {
    // IN1 = 0, IN2 = 0 -> Motor detenido
    GPIOB->ODR &= ~((1 << 6) | (1 << 7)); // PB6 y PB7 en LOW
	}
void Motor_Stop(void) {
    // IN1 = 0, IN2 = 0 -> Motor detenido
    GPIOB->ODR &= ~((1 << 6) | (1 << 7)); // PB6 y PB7 en LOW
		GPIOB->ODR &= ~((1 << 4) | (1 << 5)); // PB4 y PB5 en LOW
	}
void led_prueba(void){
	GPIOF->ODR ^= (1<<0); // Toggle del LED
	if(bandera3 == 0){
		bandera3 =1;
	}else{
		bandera3=0;
	}


}


int main(){
    //GPIOs
    RCC->AHB1ENR |= 0xFF; //habilita todos los puertos

    GPIOB->MODER &= ~((0b11<<0)|(0b11<<14));
    GPIOB->MODER |= ((1<<0)|(1<<14)); 
    GPIOC->MODER &= ~(0b11<<26);
		// limpiar pin PE13
		GPIOE->MODER &= ~(0b11<<26);
		GPIOE->MODER |= (1<<26);

    GPIOB->OTYPER &= ~((1<<0)|(1<<7));
		GPIOE->OTYPER &= ~(1 << 13);   // Limpiar bit 13 para salida push-pull
    GPIOB->OSPEEDR |= (((1<<1)|(1<<0)|(1<<15)|(1<<14)));
    GPIOC->OSPEEDR |= ((1<<27)|(1<<26));
	  GPIOE->OSPEEDR |= (0b11 << 26); // Configurar PE13 como alta velocidad (11)
    GPIOB->PUPDR &= ~((0b11<<0)|(0b11<<14));
    GPIOC->PUPDR &= ~(0b11<<26);
    GPIOC->PUPDR |= (1<<27);
		GPIOE->PUPDR &= ~(0b11 << 26);  // Sin pull-up ni pull-down (00)
	
		GPIOF->MODER &= ~(0b11<<0); // Configura PF0 como salida
    GPIOF->MODER |= (1<<0);
    GPIOF->OTYPER &= ~(1<<0); // Configura PF0 como push-pull
    GPIOF->OSPEEDR |= (0b11<<0); // Configura la velocidad de PF0
    GPIOF->PUPDR &= ~(0b11<<0); 
	
		settings();
    settingsLCD(CD); // Limpiar la LCD
    settingsLCD(RAW1); // Posición inicial

    
    
		
    

    //Systick
    SysTick->LOAD = 0x00FFFFFF; 
    SysTick->CTRL |= (0b101);

    //Interrupt
    RCC->APB2ENR |= (1<<14); 
    SYSCFG->EXTICR[3] &= ~(0b1111<<4); 
    SYSCFG->EXTICR[3] |= (1<<5); 
    EXTI->IMR |= (1<<13); 
    EXTI->RTSR |= (1<<13);
    NVIC_EnableIRQ(EXTI15_10_IRQn); 
        
    //UART
    RCC->AHB1ENR |= (1<<3); //Enable the GPIOD clock (UART3 is connected on PD9 (RX) and PD8 (TX))
    GPIOD->MODER &= ~((0b11<<18)|(0b11<<16)); //Clear (00) pins PD9 (bits 19:18) and PD8 (bits 17:16)
    GPIOD->MODER |= (1<<19)|(1<<17); //Set (10) pins PD9=RX (bits 19:18) and PD8=TX (bits 17:16) as alternant function
    GPIOD->AFR[1] &= ~((0b1111<<4)|(0b1111<<0)); //Clear (0000) alternant functions for pins PD9 (bits 7:4) and PD8 (bits 3:0)
    GPIOD->AFR[1] |= (0b111<<4)|(0b111<<0); //Set the USART3 (AF7) alternant function for pins PD9=RX (bits 7:4) and PD8=TX (bits 3:0)
    RCC->APB1ENR |= 0xC0000; //Enable the USART3 clock
    USART3->BRR = 0x683; //Set the baud rate on 9600 baud to 16 MHz (HSI)
    USART3->CR1 |= ((1<<5)|(0b11<<2)); //RXNE interrupt enable, transmitter enable and receiver enable
    USART3->CR1 |= (1<<0); //USART enable
    NVIC_EnableIRQ(USART3_IRQn); //Enable the interrupt function on the NVIC module
		
		GPIOA->MODER |= 0xA; // A1 alternante Tx
		GPIOA->AFR[0]|=0x8; //AF8
		GPIOC->MODER |= 0x800000; // C11 Alternate Rx
		GPIOC->AFR[1]|=0x8000;//AF8
		
		UART4->BRR=0x683; //9600 bauds
		UART4->CR1=0x2C; //Tx Rx interrup
		UART4->CR1 |= USART_CR1_UE; //UE
		NVIC_EnableIRQ(UART4_IRQn); //interrupcion

		Motor_Init();
		
		// ADC
    GPIOC->MODER |= (0b11<<0); // Set the bit PC0 (ADC123_IN10) as analog mode        
    RCC->APB2ENR |= (1<<9); // Enable the ADC2 clock 
    ADC2->CR2 |= ((1<<10)|(1<<0)); // Select the EOC bit at the end of each regular conversion and enable the A/D converter
    ADC2->CR1 &= ~(0b11<<24); // Clear the A/D resolution bits 
    ADC2->CR1 |= (1<<24); // Set the A/D resolution on 10 bits (minimum 13 ADCCLK cycles)
    ADC2->SMPR1 |= (1<<0); // 15 ADCCLK cycles on channel 10 (PC0)
    ADC2->SQR3 &= ~(0b11111<<0); // Clear the regular sequence bits 
    ADC2->SQR3 |= (0b1010<<0); // Set the channel 10 on 1st conversion in regular sequence 


		// TIMER
		// Habilita el reloj del TIMER3
    RCC->APB1ENR |= (1<<1);

    // Configura el prescaler
    TIM3->PSC = 16000 - 1; // 16 MHz / 16000 = 1 kHz

    // Configura el valor máximo de cuenta
    TIM3->ARR = 10000 - 1; // 5000 ms = 5 segundos

    // Habilita la interrupción en el update event
    TIM3->DIER |= (1<<0);

    // Habilita el contador
    TIM3->CR1 |= (1<<0);

    // Habilita la interrupción del TIM3 en el NVIC
    //NVIC_EnableIRQ(TIM3_IRQn);

    while(1){
			
//			for (int i = 0; i < 11; i++) {
//        writeLCD(mensaje1[i]);
//    }
			while(bandera3 == 0){
			// Inicia la conversión ADC en el canal 10 (PC0)
        ADC2->CR2 |= (1<<30); // Inicia la conversión ADC
        while(((ADC2->SR & (1<<1)) >> 1) == 0){} // Espera a que la conversión termine
        ADC2->SR &= ~(1<<1); // Limpia el bit EOC
        
        // Lee el valor ADC y calcula el voltaje
        digital = ADC2->DR;
        voltaje = (float)digital * (3.3 / 1023.0);
					if (voltaje > 1.0){
						bandera3 =1;
						NVIC_EnableIRQ(TIM3_IRQn);
					}
			
			}
//			
			// Inicia la conversión ADC en el canal 10 (PC0)
        ADC2->CR2 |= (1<<30); // Inicia la conversión ADC
        while(((ADC2->SR & (1<<1)) >> 1) == 0){} // Espera a que la conversión termine
        ADC2->SR &= ~(1<<1); // Limpia el bit EOC
        
        // Lee el valor ADC y calcula el voltaje
        digital = ADC2->DR;
        voltaje = (float)digital * (3.3 / 1023.0);
				
					
			if(bandera3 == 1){	
					
			if	 (inf == 1 /*|| voltaje >= 1.0*/){
						Motor_Stopb();
						Motor_Stop();
						inf = 0;
						settings();
						settingsLCD(CD); // Limpiar la LCD
						settingsLCD(RAW1); // Posición inicial						
						for (int i = 0; i < 9; i++) {
								writeLCD(obstaculo9[i]);}
						// Delay para evitar que el mensaje cambie constantemente
						for (int i = 0; i < 1000000; i++);
			}
				else if((d  == 'a')){
						// movimiento adelante
            GPIOB->ODR = 0x81;  // Enciende LEDs en PB0 y 
						// Encender el LED
						//GPIOE->ODR |= (1 << 13);   // Poner en 1 el bit 13 (PE13 = HIGH)
						Motor_Forwarda();
						Motor_Forwardb();
						settings();
						settingsLCD(CD); // Limpiar la LCD
						settingsLCD(RAW1); // Posición inicial						
						for (int i = 0; i < 8; i++) {
								writeLCD(adelante8[i]);
						}
					
        }else if(d == 'b'){
						// movimiento atras
            GPIOB->ODR &= ~(1<<7);
						// Apagar el LED
						//GPIOE->ODR &= ~(1 << 13);  // Poner en 0 el bit 13 (PE13 = LOW)
						Motor_Backwarda();
						Motor_Backwardb();
						settings();
						settingsLCD(CD); // Limpiar la LCD
						settingsLCD(RAW1); // Posición inicial						
						for (int i = 0; i < 7; i++) {
								writeLCD(reversa7[i]);}
						//for (volatile uint32_t i = 0; i < 100000; i++);
						//Motor_Stop();
        }else if(d == 'c'){
						//movimiento derecha
            //GPIOB->ODR &= ~(1<<7);
						// Apagar el LED
						//GPIOE->ODR &= ~(1 << 13);  // Poner en 0 el bit 13 (PE13 = LOW)
						Motor_Forwardb();
						Motor_Backwarda();
						settings();
						settingsLCD(CD); // Limpiar la LCD
						settingsLCD(RAW1); // Posición inicial						
						for (int i = 0; i < 7; i++) {
								writeLCD(derecha7[i]);}
        }else if(d  == 'd'){
            GPIOB->ODR &= ~(1<<7);
						// Apagar el LED
						//GPIOE->ODR &= ~(1 << 13);  // Poner en 0 el bit 13 (PE13 = LOW)
						Motor_Forwarda();
						Motor_Backwardb();
						settings();
						settingsLCD(CD); // Limpiar la LCD
						settingsLCD(RAW1); // Posición inicial						
						for (int i = 0; i < 9; i++) {
								writeLCD(izquierda9[i]);}
        }
				else if(d == 'e'){
					inf = 0;
            GPIOB->ODR &= ~(1<<7);
						// Apagar el LED
						//GPIOE->ODR &= ~(1 << 13);  // Poner en 0 el bit 13 (PE13 = LOW)
						Motor_Stopb();
						Motor_Stop();
						settings();
						settingsLCD(CD); // Limpiar la LCD
						settingsLCD(RAW1); // Posición inicial						
						for (int i = 0; i < 8; i++) {
								writeLCD(detenido8[i]);}
				
				}
			
			}else{
			Motor_Stopb();
						Motor_Stop();}
		}
		}
