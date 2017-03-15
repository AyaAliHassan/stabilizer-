
#include "stm32f4_discovery.h"
//#include "pitch_stabilize.h>
#include <stdio.h>
#include <stm32f4xx.h>
#include <stm32f4xx_i2c.h>
#include <stm32f4xx_spi.h>
#include <stm32f4xx_tim.h>
#include <stm32f4xx_usart.h>
#include <math.h>

void Delay(__IO uint32_t time);
extern  __IO uint32_t timingDelay;
/*
*Description
*I2C1 for connection between STM and IMU (interface with imu module )
*USART3 for connection between STM and logger (communication with logger module )
*/


#define WHO_AM_I_MPU9250 0x75 // Should return 0x71

#define I2C_SLV0_REG     0x26

#define I2C_SLV1_REG     0x29

#define I2C_SLV2_REG     0x2C

#define I2C_SLV3_REG     0x2F

#define I2C_SLV4_REG     0x32

#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write

#define  FIFO_R_W  0x74
// Seven-bit device address is 110100 for ADO = 0
#define MPU9250_ADDRESS 0x68  // Device address when ADO = 0  dah howa slave address


void delay(uint16_t count)
{
  int i=0;
  while(i!=count)i++;
  
}
void I2C_init(void){

	    /*
         *Description
         *define varibles of GPIOx& I2Cx*/
	GPIO_InitTypeDef GPIO_InitStruct;
	I2C_InitTypeDef I2C_InitStruct;
        
       
     
        
        
	    /*
         *Description
         *extra to check i2c working*/
        GPIO_InitTypeDef GPIO_Output;     // For some debugging LEDs
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
 
	    /*
           *for timer
         *Description
         *Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
        /* Alternating functions for pins */
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);
    
        GPIO_Output.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15 ;
        GPIO_Output.GPIO_Mode = GPIO_Mode_AF;
        GPIO_Output.GPIO_OType = GPIO_OType_PP;
        GPIO_Output.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_Output.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(GPIOD, &GPIO_Output);
        
        
        
        /*
         *Description
         *enable APB1 peripheral clock for I2Cx*/
	// enable APB1 peripheral clock for I2C1
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
      
	// enable clock for SCL and SDA pins
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	/*
     *Description
     *setup SCL and SDA pins
	 * You can connect the I2C1 functions to two different
	 * pins:
	 * 1. SCL on PB6  
	 * 2. SDA on PB7 
	 */
       
       
	GPIO_InitStruct.GPIO_Pin =GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;			// set pins to alternate function
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		        // set GPIO speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;		  	// set output to open drain --> the line has to be only pulled low, not driven high
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// enable pull up resistors
	GPIO_Init(GPIOB, &GPIO_InitStruct);			        // init GPIOB
      
	
       
       
        /*
         *Description
         *configure I2Cx*/
	// configure I2C1 
        // configure I2C1 
        // Connect I2C1 pins to AF  
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);	// SCL for I2C1
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1); // SDA for I2C1
        
	I2C_InitStruct.I2C_ClockSpeed = 400000; 		// 400 kHz
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;			// I2C mode
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;	        // 50% duty cycle --> standard
	I2C_InitStruct.I2C_OwnAddress1 = 0x00;			// own address, not relevant in master mode
	I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;		// disable acknowledge when reading (can be changed later on)
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // set address length to 7 bit addresses
	I2C_Init(I2C1, &I2C_InitStruct);				// init I2C1
       
        /*
         *Description
         *enable I2Cx*/
	// enable I2C1
	I2C_Cmd(I2C1, ENABLE);
       
      
}

/*
 *Description
 *This function issues a start condition and 
 * transmits the slave address + R/W bit
 * 
 * Parameters:
 * 		I2Cx --> the I2C peripheral e.g. I2C1
 * 		address --> the 7 bit slave address
 * 		direction --> the transmission direction can be:
 * 						I2C_Direction_Tranmitter for Master transmitter mode
 * 						I2C_Direction_Receiver for Master receiver
 */
void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction){
	// wait until I2C1 is not busy any more
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
  
	// Send I2C1 START condition 
	I2C_GenerateSTART(I2Cx, ENABLE);
	  
	// wait for I2C1 EV5 --> Slave has acknowledged start condition
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
		
	// Send slave Address for write 
	  I2C_Send7bitAddress(I2Cx, address, direction);
	 /*hena hoa mstne al ack tgelo mn el slave fe eve6 */ 
	/* wait for I2Cx EV6, check if 
	 * either Slave has acknowledged Master transmitter or
	 * Master receiver mode, depending on the transmission
	 * direction
	 */ 
	if(direction == I2C_Direction_Transmitter){
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	}
	else if(direction == I2C_Direction_Receiver){
		while(!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	}
}

void I2C_start2(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction){
	// wait until I2C1 is not busy any more
	//while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
  
	// Send I2C1 START condition 
	I2C_GenerateSTART(I2Cx, ENABLE);
	  
	// wait for I2C1 EV5 --> Slave has acknowledged start condition
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
		
	// Send slave Address for write 
	  I2C_Send7bitAddress(I2Cx, address, direction);
	 /*hena hoa mstne al ack tgelo mn el slave fe eve6 */ 
	/* wait for I2Cx EV6, check if 
	 * either Slave has acknowledged Master transmitter or
	 * Master receiver mode, depending on the transmission
	 * direction
	 */ 
	if(direction == I2C_Direction_Transmitter){
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	}
	else if(direction == I2C_Direction_Receiver){
		while(!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	}
}

/*
 *Description
 * This function transmits one byte to the slave device
 * Parameters:
 *		I2Cx --> the I2C peripheral e.g. I2C1 
 *		data --> the data byte to be transmitted
 */
void I2C_write(I2C_TypeDef* I2Cx, uint8_t data)
{
	// wait for I2C1 EV8 --> last byte is still being transmitted (last byte in SR, buffer empty), next byte can already be writte
	I2C_SendData(I2Cx, data);
        while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTING));
}

/*
 *Description
 * This function reads one byte from the slave device 
 * and acknowledges the byte (requests another byte)
 */
uint8_t I2C_read_ack(I2C_TypeDef* I2Cx){
	// enable acknowledge of received data
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	// wait until one byte has been received
	
        while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// read data from I2C data register and return data byte
	uint8_t data = (uint8_t)I2Cx->DR;
	return data;
}

/*
 *Description
 * This function reads one byte from the slave device
 * and doesn't acknowledge the received data 
 * after that a STOP condition is transmitted
 */
uint8_t I2C_read_nack(I2C_TypeDef* I2Cx){
	// disable acknowledge of received data
	// nack also generates stop condition after last byte received
	// see reference manual for more info
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	I2C_GenerateSTOP(I2Cx, ENABLE);
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx,  I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// read data from I2C data register and return data byte
	uint8_t data =I2C_ReceiveData(I2Cx);
	return data;
}
/*
 *Description
 *This function issues a stop condition and therefore
 * releases the bus
 */
void I2C_stop(I2C_TypeDef* I2Cx){
	
	// Send I2C1 STOP Condition after last byte has been transmitted
	I2C_GenerateSTOP(I2Cx, ENABLE);
	// wait for I2C1 EV8_2 --> byte has been transmitted
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}
/*
 *Description
 *This function write bytes  from specific register address in imu
 */
void writeByte(uint8_t slave_address, uint8_t reg, uint8_t data)
{
	I2C_start(I2C1, slave_address<<1, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
        I2C_write(I2C1, reg);
	I2C_write(I2C1, data);              // Put data in Tx buffer
	I2C_stop(I2C1);           
}
/*
 *Description
 *This function read byte  from specific register address in imu
 */
uint8_t readByte(uint8_t slave_address , uint8_t reg)
{
        uint8_t received_data;
        I2C_start(I2C1,  slave_address<<1, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
        I2C_write(I2C1, reg);
        I2C_start2(I2C1, slave_address<<1, I2C_Direction_Receiver); // start a transmission in Master receiver mode
	received_data = I2C_read_nack(I2C1); // read one byte and don't request another byte, stop transmission
        return received_data;
}
/*
 *Description
 *This function read bytes  from specific register address in imu
 */
uint8_t * readBytes(uint8_t slave_address, uint8_t reg, uint8_t count, uint8_t * dest)
{  
	I2C_start(I2C1, slave_address<<1, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
        I2C_write(I2C1, reg);
        I2C_start2(I2C1, slave_address<<1, I2C_Direction_Receiver); // start a transmission in Master receiver mode
	uint8_t i = 0;
	while (i!=count) 
        {
          if(i==count-1)
            dest[i++] = I2C_read_nack(I2C1);  //read last value
           else
        dest[i++] = I2C_read_ack(I2C1);    // Put read results in the Rx buffer
        }         
       return dest;
}



 
void init_USART3(uint32_t baudrate){
	
	/* This is a concept that has to do with the libraries provided by ST
	 * to make development easier the have made up something similar to 
	 * classes, called TypeDefs, which actually just define the common
	 * parameters that every peripheral needs to work correctly
	 * 
	 * They make our life easier because we don't have to mess around with 
	 * the low level stuff of setting bits in the correct registers
	 */
	GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStruct; // this is for the USART3 initilization
	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)
        
        
	/* enable APB1 peripheral clock for USART3
	 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	
	/* enable the peripheral clock for the pins used by 
	 * USART3, PB10 for TX and PB11 for RX
	 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	/* This sequence sets up the TX and RX pins 
	 * so they work correctly with the USART3 peripheral
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; // Pins 10 (TX) and 11 (RX) are used ,pin 12 for (CK)
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// this activates the pullup resistors on the IO pins
	GPIO_Init(GPIOB, &GPIO_InitStruct);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers
	
	/* The RX and TX pins are now connected to their AF
	 * so that the USART3 can take over control of the 
	 * pins
	 */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);   //Tx
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);  //Rx
        
	
	/* Now the USART_InitStruct is used to define the 
	 * properties of USART3
	 */
	USART_InitStruct.USART_BaudRate = baudrate;				// the baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART3, &USART_InitStruct);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting
	
	
	/* Here the USART3 receive interrupt is enabled
	 * and the interrupt controller is configured 
	 * to jump to the USART3_IRQHandler() function
	 * if the USART3 receive interrupt occurs
	 */
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);                    // enable the USART3 receive interrupt 
	
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;		 // we want to configure the USART3 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the USART3 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART3 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);							 // the properties are passed to the NVIC_Init function which takes care of the low level stuff	

	// finally this enables the complete USART3 peripheral
	USART_Cmd(USART3, ENABLE);
}

/*test********************************/
void UU_PutChar(USART_TypeDef* USARTx, uint8_t ch)
{
  USARTx->DR = ch;  
  while(!(USARTx->SR & USART_SR_TXE));
}

void UU_PutString(USART_TypeDef* USARTx, uint8_t * str)
{
  while(*str != 0)
  {
    UU_PutChar(USARTx, *str);
    str++;
  }
}

void UU_PutNumber(USART_TypeDef* USARTx, uint32_t x)
{
  char value[10]; //a temp array to hold results of conversion
  int i = 0; //loop index
  
  do
  {
    value[i++] = (char)(x % 10) + '0'; //convert integer to character
    x /= 10;
  } while(x);
  
  while(i) //send data
  {
    UU_PutChar(USARTx, value[--i]); 
  }
}


/**resons for problem of running only in debug mode
*1) rassberry pi takes longer time to power up than stm so stm send data before rasbbery pi was ready to read when u make debug u caused it to work to solve this i added dealy at the begging of stm code
*2) IAR problem ,to solve it i did some configuration
*3) usart.h mkansh enable fe el conf.h driver
*4) u should rebuild the progrem before try it with another device
*/

/*varibles to recive pitch & yaw */
uint8_t p1,p2,y1,y2,p3,y3;
/*varibles*/

short initial_pitch()
{
  
  initial_pitch_flag = 1;
 // I2C_init();
  //I2C_start(I2C1, SLAVE_ADDRESS<<1, I2C_Direction_Receiver);
 // uint8_t temp;
  /*for( int i=0;i<18;i++)
  {
   temp = I2C_read_ack(I2C1);
  
  }*/
   //  pitch_first_1= I2C_read_ack(I2C1);
   //  pitch_first_2= I2C_read_ack(I2C1);
  pitch_first_1 = 11;
  pitch_first_2 = 20;
       //   I2C_stop(I2C1);
  
short concatenated = concatenate (pitch_first_1 ,pitch_first_2);
  printf(" %d\n", concatenated);
  return concatenated  ;
  

}
short not_initial_pitch()
{

short ipitch =concatenate ( pitch_first_1 , pitch_first_2);
uint8_t pitch_1_stabilized  = pitch_first_1;
uint8_t pitch_2_stabilized = pitch_first_2;
uint8_t temp;
 /* for( int i=0;i<18;i++)
  {
   temp = I2C_read_ack(I2C1);
  
  }*/
  
//uint8_t pitch_1_UNstabilized  = I2C_read_ack(I2C1);
//uint8_t pitch_2_UNstabilized  = I2C_read_ack(I2C1);
        //  I2C_stop(I2C1);
uint8_t pitch_1_UNstabilized = 11;
uint8_t pitch_2_UNstabilized  =20;


/*// first part 
if (pitch_1_UNstabilized>pitch_1_stabilized)
pitch_1_stabilized = pitch_1_UNstabilized - (pitch_1_UNstabilized - pitch_1_stabilized);
else if (pitch_1_UNstabilized<pitch_1_stabilized)
{
pitch_1_stabilized= pitch_1_UNstabilized +(pitch_1_stabilized-pitch_1_UNstabilized);
}
else if (pitch_1_UNstabilized==pitch_1_stabilized) ;

// second 

if (pitch_2_UNstabilized>pitch_2_stabilized)
pitch_2_stabilized = pitch_2_UNstabilized - (pitch_2_UNstabilized - pitch_2_stabilized);
else if (pitch_2_UNstabilized<pitch_2_stabilized)
{
pitch_2_stabilized= pitch_2_UNstabilized +(pitch_2_stabilized-pitch_2_UNstabilized);
}
else if (pitch_2_UNstabilized==pitch_2_stabilized) ;

*/





short concatenated =concatenate (pitch_1_UNstabilized,pitch_2_UNstabilized);
  printf("wrong is  %d\n", concatenated);
  printf("right is  %d\n", ipitch);

if (concatenated>ipitch)
{

  concatenated = concatenated - (concatenated - ipitch);
      printf("corrected  is  %d\n", concatenated);

}
else if (concatenated<ipitch)
{
concatenated= concatenated +(ipitch-concatenated);
      printf("corrected  is  %d\n", concatenated);

}
else if (concatenated==ipitch) ;
  printf(" %d\n", concatenated);

return concatenated ; 
}




short stabilize_pitch ()
{
if (initial_pitch_flag==0) return  initial_pitch();
else return not_initial_pitch();



}

//////////////////////////////////////////////////////////
int initial_yaw_flag = 0; 
uint8_t yaw_first_1;
uint8_t yaw_first_2;

short initial_yaw()
{
  
  initial_yaw_flag = 1;
 // I2C_init();
  //I2C_start(I2C1, SLAVE_ADDRESS<<1, I2C_Direction_Receiver);
 // uint8_t temp;
  /*for( int i=0;i<18;i++)
  {
   temp = I2C_read_ack(I2C1);
  
  }*/
   //  yaw_first_1= I2C_read_ack(I2C1);
   //  yaw_first_2= I2C_read_ack(I2C1);
  yaw_first_1 = 11;
  yaw_first_2 = 20;
       //   I2C_stop(I2C1);
  
short concatenated = concatenate (yaw_first_1 ,yaw_first_2);
  printf(" %d\n", concatenated);
  return concatenated  ;
  

}
short not_initial_yaw()
{

short iyaw =concatenate ( yaw_first_1 , yaw_first_2);
uint8_t yaw_1_stabilized  = yaw_first_1;
uint8_t yaw_2_stabilized = yaw_first_2;
uint8_t temp;
 /* for( int i=0;i<18;i++)
  {
   temp = I2C_read_ack(I2C1);
  
  }*/
  
//uint8_t yaw_1_UNstabilized  = I2C_read_ack(I2C1);
//uint8_t yaw_2_UNstabilized  = I2C_read_ack(I2C1);
        //  I2C_stop(I2C1);
uint8_t yaw_1_UNstabilized = 13;
uint8_t yaw_2_UNstabilized  =20;


/*// first part 
if (pitch_1_UNstabilized>pitch_1_stabilized)
pitch_1_stabilized = pitch_1_UNstabilized - (pitch_1_UNstabilized - pitch_1_stabilized);
else if (pitch_1_UNstabilized<pitch_1_stabilized)
{
pitch_1_stabilized= pitch_1_UNstabilized +(pitch_1_stabilized-pitch_1_UNstabilized);
}
else if (pitch_1_UNstabilized==pitch_1_stabilized) ;

// second 

if (pitch_2_UNstabilized>pitch_2_stabilized)
pitch_2_stabilized = pitch_2_UNstabilized - (pitch_2_UNstabilized - pitch_2_stabilized);
else if (pitch_2_UNstabilized<pitch_2_stabilized)
{
yaw_2_stabilized= yaw_2_UNstabilized +(yaw_2_stabilized-pitch_2_UNstabilized);
}
else if (yaw_2_UNstabilized==yaw_2_stabilized) ;

*/





short concatenated =concatenate (yaw_1_UNstabilized,yaw_2_UNstabilized);
  printf("wrong is  %d\n", concatenated);
  printf("right is  %d\n", iyaw);

if (concatenated>iyaw)
{

  concatenated = concatenated - (concatenated - iyaw);
      printf("corrected  is  %d\n", concatenated);

}
else if (concatenated<iyaw)
{
concatenated= concatenated +(iyaw-concatenated);
      printf("corrected  is  %d\n", concatenated);

}
else if (concatenated==iyaw) ;
  printf(" %d\n", concatenated);

return concatenated ; 
}




short stabilize_yaw ()
{
if (initial_yaw_flag==0) return  initial_yaw();
else return not_initial_yaw();



}

//////////////////////////////////////////////////////////



int main(void){

	    /*main*/
	   I2C_init(); // initialize I2C1   peripheral
      while(1)
    {     
           /*recive pitch and yaw  from imu*/
          //law el pitch mslan 260.22 >>>p1=26 , p2=0 ,p3=22
          //law el yaw mslan  252.11 >>>y1=25 ,y2=5 ,y3=11
            p1=readByte(MPU9250_ADDRESS , I2C_SLV0_REG);     
            p2=readByte(MPU9250_ADDRESS , I2C_SLV1_REG);
            p3=readByte(MPU9250_ADDRESS , FIFO_R_W);
            y1=readByte(MPU9250_ADDRESS , I2C_SLV2_REG);
            y2=readByte(MPU9250_ADDRESS , I2C_SLV3_REG);
            y3=readByte(MPU9250_ADDRESS , I2C_SLV4_REG);
            /*end of receiving data from imu*/
            
   
  /*sending data to logger*/
    init_USART3(9600); // initialize USART1 @ 9600 baud
  
      delay(90000);
   UU_PutString(USART3, "120.23,");
     delay(90000);
   U_PutString(USART3, "10.11,");
     delay(90000);
   /* USART_SendData(USART3, 111);   //send 16 bits data 
    delay(90000);
    USART_SendData(USART3, 222);   //send 16 bits data 
    delay(90000);
    USART_SendData(USART3, 133);   //send 16 bits data 
    delay(90000);
    USART_SendData(USART3, 144);   //send 16 bits data 
    delay(90000);
    USART_SendData(USART3, 155);   //send 16 bits data 
    delay(90000);
    USART_SendData(USART3, 166);   //send 16 bits data 
    delay(90000);
    USART_SendData(USART3, 177);   //send 16 bits data 
    delay(90000);
    USART_SendData(USART3, 188);   //send 16 bits data 
    delay(90000);
    USART_SendData(USART3, 199);   //send 16 bits data 
    delay(90000);
    USART_SendData(USART3, 100);   //send 16 bits data 
    delay(90000);*/
    }
    /*end of sending data to logger*/
    
   
	
	/*toaa */
short test = stabilize_yaw();
 short test2 = stabilize_yaw(); 

  
        
	return 0;
}
void Delay(__IO uint32_t time)
{
timingDelay=time;
while(timingDelay!=0);

}
