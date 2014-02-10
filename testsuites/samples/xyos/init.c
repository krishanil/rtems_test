/*
 *  COPYRIGHT (c) 1989-2012.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#define CONFIGURE_INIT
#include "system.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <rtems.h>
#include <fcntl.h>
#include <inttypes.h>
#include <rtems/error.h>
#include <rtems/dosfs.h>
#include <ctype.h>
#include <rtems/bdpart.h>
#include <rtems/libcsupport.h>
#include <rtems/fsmount.h>
#include <rtems/ramdisk.h>
#include <rtems/nvdisk.h>
#include <rtems/nvdisk-sram.h>
#include <rtems/shell.h>
#include <rtems/io.h>
#include <bsp/io.h>
#include <bsp/rcc.h>
#include <bsp/irq.h>
#include <bsp/irq-generic.h>

#include <rtems/status-checks.h>


#include "stm32f4xx_gpio.h"
#include "stm32f4xx.h"


const stm32f4_gpio_config stm32f4_led_config_gpio [] = {
	{ \
		{ \
		  .pin_first = STM32F4_GPIO_PIN(3, 12), \
		  .pin_last = STM32F4_GPIO_PIN(3, 12), \
		  .mode = STM32F4_GPIO_MODE_OUTPUT, \
		  .otype = STM32F4_GPIO_OTYPE_PUSH_PULL, \
		  .ospeed = STM32F4_GPIO_OSPEED_100_MHZ, \
		  .pupd = STM32F4_GPIO_NO_PULL, \
		  .output = 0, \
		  .af = 0, \
		  .reserved = 0\
		} \
	},
	{ \
		{ \
		  .pin_first = STM32F4_GPIO_PIN(3, 13), \
		  .pin_last = STM32F4_GPIO_PIN(3, 13), \
		  .mode = STM32F4_GPIO_MODE_OUTPUT, \
		  .otype = STM32F4_GPIO_OTYPE_PUSH_PULL, \
		  .ospeed = STM32F4_GPIO_OSPEED_100_MHZ, \
		  .pupd = STM32F4_GPIO_NO_PULL, \
		  .output = 0, \
		  .af = 0, \
		  .reserved = 0\
		} \
	},
	{ \
		{ \
		  .pin_first = STM32F4_GPIO_PIN(3, 14), \
		  .pin_last = STM32F4_GPIO_PIN(3, 14), \
		  .mode = STM32F4_GPIO_MODE_OUTPUT, \
		  .otype = STM32F4_GPIO_OTYPE_PUSH_PULL, \
		  .ospeed = STM32F4_GPIO_OSPEED_100_MHZ, \
		  .pupd = STM32F4_GPIO_NO_PULL, \
		  .output = 0, \
		  .af = 0, \
		  .reserved = 0\
		} \
	},
	{ \
		{ \
		  .pin_first = STM32F4_GPIO_PIN(3, 15), \
		  .pin_last = STM32F4_GPIO_PIN(3, 15), \
		  .mode = STM32F4_GPIO_MODE_OUTPUT, \
		  .otype = STM32F4_GPIO_OTYPE_PUSH_PULL, \
		  .ospeed = STM32F4_GPIO_OSPEED_100_MHZ, \
		  .pupd = STM32F4_GPIO_NO_PULL, \
		  .output = 0, \
		  .af = 0, \
		  .reserved = 0\
		} \
	},
  STM32F4_GPIO_CONFIG_TERMINAL
};

const stm32f4_gpio_config stm32f4_key_config_gpio [] = {
	{ \
		{ \
		  .pin_first = STM32F4_GPIO_PIN(0, 0), \
		  .pin_last = STM32F4_GPIO_PIN(0, 0), \
		  .mode = STM32F4_GPIO_MODE_INPUT, \
		  .otype = STM32F4_GPIO_OTYPE_PUSH_PULL, \
		  .ospeed = STM32F4_GPIO_OSPEED_100_MHZ, \
		  .pupd = STM32F4_GPIO_NO_PULL, \
		  .output = 0, \
		  .af = 0, \
		  .reserved = 0\
		} \
	},
  STM32F4_GPIO_CONFIG_TERMINAL
};

typedef struct {
	uint32_t memrmp;		/*!< SYSCFG memory remap register,						Address offset: 0x00	  */
	uint32_t pmc;			/*!< SYSCFG peripheral mode configuration register, 	Address offset: 0x04	  */
	uint32_t exticr[4];	/*!< SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14 */
	uint32_t	  reserved[2];	/*!< Reserved, 0x18-0x1C														  */ 
	uint32_t cmpcr;		/*!< SYSCFG Compensation cell control register, 		Address offset: 0x20	  */
} stm32f4_syscfg;

#define STM32F4_SYSCFG 	((volatile stm32f4_syscfg *) (0 + 0x40013800))

typedef struct
{
  uint32_t IMR;    /*!< EXTI Interrupt mask register,            Address offset: 0x00 */
  uint32_t EMR;    /*!< EXTI Event mask register,                Address offset: 0x04 */
  uint32_t RTSR;   /*!< EXTI Rising trigger selection register,  Address offset: 0x08 */
  uint32_t FTSR;   /*!< EXTI Falling trigger selection register, Address offset: 0x0C */
  uint32_t SWIER;  /*!< EXTI Software interrupt event register,  Address offset: 0x10 */
  uint32_t PR;     /*!< EXTI Pending register,                   Address offset: 0x14 */
} stm32f4_exti;

#define STM32F4_EXTI 	((volatile stm32f4_exti *) (0 + 0x40013c00))


void SYSCFG_EXTILineConfig(uint8_t EXTI_PortSourceGPIOx, uint8_t EXTI_PinSourcex)
{
  uint32_t tmp = 0x00;

  tmp = ((uint32_t)0x0F) << (0x04 * (EXTI_PinSourcex & (uint8_t)0x03));
  SYSCFG->EXTICR[EXTI_PinSourcex >> 0x02] &= ~tmp;
  SYSCFG->EXTICR[EXTI_PinSourcex >> 0x02] |= (((uint32_t)EXTI_PortSourceGPIOx) << (0x04 * (EXTI_PinSourcex & (uint8_t)0x03)));

  printk("tmp=0x%x, SYSCFG=0x%x, SYSCFG->EXTICR=0x%x\n", tmp, SYSCFG, SYSCFG->EXTICR);
}

void stm32f4_exti_line_config(unsigned char EXTI_PortSourceGPIOx, unsigned char EXTI_PinSourcex)
{
  uint32_t tmp = 0x00;
  volatile stm32f4_syscfg *syscfg = STM32F4_SYSCFG;

  tmp = ((uint32_t)0x0F) << (0x04 * (EXTI_PinSourcex & (uint8_t)0x03));
  syscfg->exticr[EXTI_PinSourcex >> 0x02] &= ~tmp;
  syscfg->exticr[EXTI_PinSourcex >> 0x02] |= (((uint32_t)EXTI_PortSourceGPIOx) << (0x04 * (EXTI_PinSourcex & (uint8_t)0x03)));

  printk("tmp=0x%x, syscfg=0x%x, syscfg->exticr=0x%x\n", tmp, syscfg, syscfg->exticr);
}

typedef struct {
	unsigned int EXTI_Line;
	unsigned int EXTI_Mode;
	unsigned int EXTI_Trigger;
	unsigned int EXTI_LineCmd;
} EXTI_InitTypeDef;

void EXTI_Init(EXTI_InitTypeDef* EXTI_InitStruct)
{
  uint32_t tmp = 0;
  volatile stm32f4_exti *exti = STM32F4_EXTI;

  tmp = (uint32_t)STM32F4_EXTI;
     
  if (EXTI_InitStruct->EXTI_LineCmd != DISABLE)
  {
    /* Clear EXTI line configuration */
    exti->IMR &= ~EXTI_InitStruct->EXTI_Line;
    exti->EMR &= ~EXTI_InitStruct->EXTI_Line;
    
    tmp += EXTI_InitStruct->EXTI_Mode;

    *(volatile uint32_t *) tmp |= EXTI_InitStruct->EXTI_Line;

    /* Clear Rising Falling edge configuration */
    exti->RTSR &= ~EXTI_InitStruct->EXTI_Line;
    exti->FTSR &= ~EXTI_InitStruct->EXTI_Line;
    
    /* Select the trigger for the selected external interrupts */
    if (EXTI_InitStruct->EXTI_Trigger == 0x10)
    {
      /* Rising Falling edge */
      exti->RTSR |= EXTI_InitStruct->EXTI_Line;
      exti->FTSR |= EXTI_InitStruct->EXTI_Line;
    }
    else
    {
      tmp = (uint32_t)STM32F4_EXTI;
      tmp += EXTI_InitStruct->EXTI_Trigger;

      *(volatile uint32_t *) tmp |= EXTI_InitStruct->EXTI_Line;
    }
  }
  else
  {
    tmp += EXTI_InitStruct->EXTI_Mode;

    /* Disable the selected external lines */
    *(volatile uint32_t *) tmp &= ~EXTI_InitStruct->EXTI_Line;
  }
}

static void stm32f4_key_handler(void *arg)
{
	volatile stm32f4_exti *exti = STM32F4_EXTI;

	exti->PR = 0x00001;
	printk("%s, %d\n", __func__, __LINE__);
}


//void stm32f4_gpio_set_output(int pin, bool set);

static void xyos_menu (void)
{
	bool led_flag = false;
	char inbuf[10];
	rtems_status_code sc = RTEMS_SUCCESSFUL;
	
	//NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;

	printf("   XYOS TEST  \n");

	printf("	 p -> part_table_initialize\n");
	printf("	 f -> mount all disks in fs_table\n");
	printf("	 l -> list	file\n");
	printf("	 r -> read	file\n");
	printf("	 w -> write file\n");
	printf("	 s -> start shell\n");
	printf("	 Enter your selection ==>");

	//SYSCFG_EXTILineConfig(0x00, 0x00);
	
	/*
	* Wait for characters from console terminal
	*/
	stm32f4_gpio_set_config(&stm32f4_led_config_gpio[0]);
	stm32f4_gpio_set_config(&stm32f4_led_config_gpio[1]);
	stm32f4_gpio_set_config(&stm32f4_led_config_gpio[2]);
	stm32f4_gpio_set_config(&stm32f4_led_config_gpio[3]);

	stm32f4_gpio_set_config(&stm32f4_key_config_gpio[0]);

	stm32f4_exti_line_config(0x00, 0x00);
	
	EXTI_InitStructure.EXTI_Line = 0x00001;
	EXTI_InitStructure.EXTI_Mode = 0;
	EXTI_InitStructure.EXTI_Trigger = 0x08;	
	EXTI_InitStructure.EXTI_LineCmd = 1;
	EXTI_Init(&EXTI_InitStructure);
	
	/* Install interrupt handler and disable this vector */
	sc = rtems_interrupt_handler_install(
		STM32F4_IRQ_EXTI0,
		"KEY",
		RTEMS_INTERRUPT_UNIQUE,
		stm32f4_key_handler,
		(void *)0
	);
	bsp_interrupt_vector_disable(STM32F4_IRQ_EXTI0);
	stm32f4_rcc_set_clock(STM32F4_RCC_SYSCFG, true);
	bsp_interrupt_vector_enable(STM32F4_IRQ_EXTI0);
	for (;;) {
	fflush(stdout);

	inbuf[0] = '\0';
	fgets(inbuf,sizeof(inbuf),stdin);
	switch (inbuf[0]) {
	case 's':
	  //GPIO_SetBits(GPIOD, GPIO_Pin_12);
	  led_flag = !led_flag;
	  stm32f4_gpio_set_output(STM32F4_GPIO_D12, led_flag);
	  stm32f4_gpio_set_output(STM32F4_GPIO_D13, led_flag);
	  stm32f4_gpio_set_output(STM32F4_GPIO_D14, led_flag);
	  stm32f4_gpio_set_output(STM32F4_GPIO_D15, led_flag);
	  break;
	case 'k':
		break;
	default:
	  printf("\n");
	  break;
	}

	}
	exit (0);
}

/*
 * RTEMS File Menu Task
 */
static rtems_task
xyos_task (rtems_task_argument ignored)
{
  xyos_menu();
}

/*
 * RTEMS Startup Task
 */
rtems_task
Init (rtems_task_argument ignored)
{
	rtems_name Task_name;
	rtems_id   Task_id;
	rtems_status_code status;

	puts( "\n\n*** xyos startup ***" );

	Task_name = rtems_build_name('X','Y','O','S');

	status = rtems_task_create(
	Task_name, 1, RTEMS_MINIMUM_STACK_SIZE * 2,
	RTEMS_DEFAULT_MODES ,
	RTEMS_FLOATING_POINT | RTEMS_DEFAULT_ATTRIBUTES, &Task_id
	);
	status = rtems_task_start( Task_id, xyos_task, 1 );
	status = rtems_task_delete( RTEMS_SELF );
}


