/*
 * gpio_driver.c
 *
 *  Created on: Dec 26, 2025
 *      Author: wiki
 */
#include "gpio_driver.h"


void GPIO_PeriClockCont(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi)
{
	if(EnorDi)
	{
		switch(pGPIOx)
		{
		case GPIOA:
					GPIOA_PCLK_EN();
		case GPIOB:
					GPIOB_PCLK_EN();
		case GPIOC:
					GPIOC_PCLK_EN();
		case GPIOD:
					GPIOD_PCLK_EN();
		case GPIOE:
					GPIOE_PCLK_EN();
		case GPIOF:
					GPIOF_PCLK_EN();
		case GPIOG:
					GPIOG_PCLK_EN();
		case GPIOH:
					GPIOH_PCLK_EN();
		}

	}else
		switch(pGPIOx)
				{
				case GPIOA:
							GPIOA_PCLK_DI();
				case GPIOB:
							GPIOB_PCLK_DI();
				case GPIOC:
							GPIOC_PCLK_DI();
				case GPIOD:
							GPIOD_PCLK_DI();
				case GPIOE:
							GPIOE_PCLK_DI();
				case GPIOF:
							GPIOF_PCLK_DI();
				case GPIOG:
							GPIOG_PCLK_DI();
				case GPIOH:
							GPIOH_PCLK_DI();
				}
}

void GPIO_Init(GPIO_Handle_t *pGPIO_Handle)
{
	uint32_t temp=0;
	//configure mode
	//speed
	//pupd
	//optype
	//configure Alt functionality
	if(pGPIOHandle->GPIO_PinConfig.Mode <= GPIO_MODE_ANALOG)
	{
		temp= (pGPIOHandle->GPIO_PinConfig.Mode << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));//as in datasheet the register for mode take 2 bits for each pin
		pGPIOHandle->pGPIOx->MODER &= ~(0x03u << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |=temp;

	}else
	{//this part is for the interrupt
		if(pGPIOHandle->GPIO_PinConfig.Mode <= GPIO_MODE_IT_FT)
		{//configure FTSR register
			EXTI->FTSR1 |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR1 &= ~(1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}else if(pGPIOHandle->GPIO_PinConfig.Mode <= GPIO_MODE_IT_RT)
		{//configure RTSR register
			EXTI->RTSR1 |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR1 &= ~(1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}else if(pGPIOHandle->GPIO_PinConfig.Mode <= GPIO_MODE_IT_RTFT)
		{//configure FTSR register//configure RTSR register
			EXTI->RTSR1 |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR1 |= ~(1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		//configure gpio port selection in SYSCFG_EXTCR
		//we are setting that pin of which port is taking the exti line
		//so there are 4 exticr registers such that each of that configures that which port is to be enabled for that exti pins
		uint8_t temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)/4;
		uint8_t temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)%4;
		uint8_t portcode = GPIO_BASEADDRESS_TO_CODE(pGPIOHandle->pGPIOx);
		EXTI->EXTICR[temp1]=  portcode << (temp2*4);

		//enable exti interrupt delivery using IMR
		EXTI->IMR1 |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//enable the interrupt mask register  for that pin
	}
	//speed
	temp=0;
	temp= (pGPIOHandle->GPIO_PinConfig.Speed << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));//as in datasheet the register for mode take 2 bits for each pin
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x03u << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	//configure pull pullup register
	temp=0;
	temp= (pGPIOHandle->GPIO_PinConfig.PuPdControl << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));//as in datasheet the register for mode take 2 bits for each pin
	pGPIOHandle->pGPIOx->PUPDR&= ~(0x03u << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |=temp;
	//configure optype register
	temp=0;
	temp= (pGPIOHandle->GPIO_PinConfig.OPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));//as in datasheet the register for mode take 2 bits for each pin
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x01u << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER |=temp;

	//configure alt functionality
	if (pGPIOHandle->GPIO_PinConfig.Mode <= GPIO_MODE_ANALOG)
	{
		//Alternate function register dived  into two registers high and low
		//low for pin 0 to 7
		//high for pin 8-15
		temp1= pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2= pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xFu << 4*temp2);//each pin has 4 bits

		pGPIOHandle->pGPIOx->AFR[temp1]|= (pGPIOHandle->GPIO_PinConfig.AltFuncType << 4*temp2);//each pin has 4 bits


	}

	}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	switch(pGPIOx)
			{
			case GPIOA:
						GPIOA_REG_RESET();
			case GPIOB:
						GPIOB_REG_RESET();
			case GPIOC:
						GPIOC_REG_RESET();
			case GPIOD:
						GPIOD_REG_RESET();
			case GPIOE:
						GPIOE_REG_RESET();
			case GPIOF:
						GPIOF_REG_RESET();
			case GPIOG:
						GPIOG_REG_RESET();
			case GPIOH:
						GPIOH_REG_RESET();
			}

	}


//read and write
int8_t GPIO_Read_InPin(GPIO_RegDef_t *pGPIOx, uint8_t Pin)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> Pin)& 0x1u);//and with 1lsb to have just the last bit value from idr
	return value;

	}


int16_t GPIO_Read_InPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR );
		return value;
	}


void GPIO_Write_OutPin(GPIO_RegDef_t *pGPIOx, uint8_t Pin, uint8_t value)
{
	if (value==GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (0x1u<<Pin);
	}else
	{
		pGPIOx->ODR &= ~(0x1u<<Pin);
	}
}
void GPIO_Write_OutPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{

	pGPIOx->ODR = value;

}
void GPIO_Toggle_OutPin(GPIO_RegDef_t *pGPIOx, uint8_t Pin)
{
	pGPIOx->ODR ^= (1u<<Pin);//XOR for toggling

	}

//interrupt handling
void GPIO_IRQConfig(uint8_t IRQNumber,uint8_t IRQPriority, uint8_t EnorDi)
{
/*
 *
 * 1pin must be in input configuration
 * 2configure the edge trigger rising falling or rising and falling both
 * 3enable interrupt delevery from peripheral to the processor (to be done on peripheral side)
 * 4identify the irq number on which processor accepts the interrupt from that pin(done by vector table))
 *
 *
 * above are done in init api
 *
 *
 * 5 configure the irq priority for identified irq number (processor side)-to be done in nvic register
 * 6 enable interrupt reception on that IRQ number (processor side)-in nvic enable the irq number as by default all are disabled
 * 7implement irq handler
 */
	//the configuration is here processor side
if(EnorDi==ENABLE)
{
	if(IRQNumber <= 31)
	{
		*NVIC_ISER0 |= (1<<IRQNumber);
	}else if (IRQNumber > 31 && IRQNumber < 64)
	{
		*NVIC_ISER1 |= (1<< IRQNumber % 32);
	}else if (IRQNumber > 64 && IRQNumber < 92)
	{
			*NVIC_ISER3 |= (1<< IRQNumber % 32);
		}
}else
{
	if(IRQNumber <= 31)
	{
		*NVIC_ICER0 |= (1<<IRQNumber);
	}else if (IRQNumber > 31 && IRQNumber < 64)
	{
		*NVIC_ICER1 |= (1<< IRQNumber % 32);
	}else if (IRQNumber > 64 && IRQNumber < 92)
	{
			*NVIC_ICER3 |= (1<< IRQNumber % 32);
		}
}


	}
void GPIO_IRQHandler(uint8_t Pin)
{
	}
