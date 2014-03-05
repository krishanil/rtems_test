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

/****** STM32F4-Discovery_FW_V1.1.0 ******/
#include <bsp/stm32f4xx_gpio.h>
#include <bsp/stm32f4xx_syscfg.h>
#include <bsp/stm32f4xx_exti.h>
#include "stm32f4xxxx_spi.h"

#include <rtems/bspIo.h>

#include <libchip/serial.h>

#include <bspopts.h>
#include <bsp/usart.h>


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

static void stm32f4_key_handler(void *arg)
{
	if(EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		/* Clear the EXTI line 0 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line0);
		printk("%s, %d\n", __func__, __LINE__);
	}	
}
#define delay(p) rtems_task_wake_after (RTEMS_MICROSECONDS_TO_TICKS (p))

/* Read/Write command */
#define READWRITE_CMD              ((uint8_t)0x80) 
/* Multiple byte read/write command */ 
#define MULTIPLEBYTE_CMD           ((uint8_t)0x40)
#define LIS302DL_WHO_AM_I_ADDR                  0x0F

static void mems_init(void) {
	unsigned char temp = 0;
	unsigned char addr = LIS302DL_WHO_AM_I_ADDR;
	int NumByteToRead = 0x1;

	if(NumByteToRead > 0x01)
  {
    addr |= (uint8_t)(READWRITE_CMD | MULTIPLEBYTE_CMD);
  }
  else
  {
    addr |= (uint8_t)READWRITE_CMD;
  }

	stm32f4_spi_1->ops->init(stm32f4_spi_1);
	delay(30);
	stm32f4_spi_1->ops->send_start(stm32f4_spi_1);
	stm32f4_spi_1->ops->send_addr(stm32f4_spi_1, addr, 1);
	stm32f4_spi_1->ops->read_bytes(stm32f4_spi_1, &temp, 1);
	stm32f4_spi_1->ops->send_stop(stm32f4_spi_1);

	//LIS302DL_Read(&temp, LIS302DL_WHO_AM_I_ADDR, 1);

	printf("\nwho am i is 0x%x\n", temp);
}

static void bluetooth_init(void) {
	//int fd;
	//fd = open("/dev/ttyS1", O_RDWR);

	//printk("fd = 0x%x\n", fd);
}

static void blue_read (void) {
	int c = 0;
	char ch = 0;
	printk("\nblue_read\n");
	const console_fns *con =
    Console_Configuration_Ports [0].pDeviceFns;
	while (1) {
		c = con->deviceRead((int) 0);
		if (c != -1) {
			ch = (char)c;
			rtems_putc(ch);
		}
		//sleep(1);
		//sleep(1);
	}
	exit (0);
}

static void blue_write (void) {
		const console_fns *con =
			Console_Configuration_Ports [0].pDeviceFns;
		int i = 0;
		char buf[4] = {0};
		buf[1] = '\n';
		
		printk("\nblue_write\n");
		while (1) {
			i++;
			buf[0] = i%4;
			con->deviceWrite((int) 0, "AT\n", strlen("AT\n"));
			sleep(1);
		}
	
	exit (0);
}



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

	/* Connect EXTI Line0 to PA0 pin */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
	
	/* Configure EXTI Line0 */
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;	
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
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

	mems_init();
	
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

static rtems_task
uart_read_task (rtems_task_argument ignored)
{
  blue_read();
}

static rtems_task
uart_write_task (rtems_task_argument ignored)
{
	printk("%s, %d\n", __func__, __LINE__);
  blue_write();
}



/*
 * RTEMS Startup Task
 */
rtems_task
Init (rtems_task_argument ignored)
{
	rtems_name Task_name;
	rtems_id   Task_id;
	rtems_name BlueTask_name;
	rtems_id   BlueTask_id;
	rtems_name BlueWTask_name;
	rtems_id   BlueWTask_id;
	rtems_status_code status;

	puts( "\n\n*** xyos startup ***" );

	Task_name = rtems_build_name('X','Y','O','S');
	status = rtems_task_create(
	Task_name, 1, RTEMS_MINIMUM_STACK_SIZE * 2,
	RTEMS_DEFAULT_MODES ,
	RTEMS_FLOATING_POINT | RTEMS_DEFAULT_ATTRIBUTES, &Task_id
	);
	status = rtems_task_start( Task_id, xyos_task, 1 );

	BlueWTask_name = rtems_build_name('W','B','L','U');
	status = rtems_task_create(
	BlueWTask_name, 2, RTEMS_MINIMUM_STACK_SIZE,
	RTEMS_DEFAULT_MODES ,
	RTEMS_FLOATING_POINT | RTEMS_DEFAULT_ATTRIBUTES, &BlueWTask_id
	);
	status = rtems_task_start( BlueWTask_id, uart_write_task, 1 );

	BlueTask_name = rtems_build_name('R','B','L','U');
	status = rtems_task_create(
	BlueTask_name, 3, RTEMS_MINIMUM_STACK_SIZE,
	RTEMS_DEFAULT_MODES ,
	RTEMS_FLOATING_POINT | RTEMS_DEFAULT_ATTRIBUTES, &BlueTask_id
	);
	status = rtems_task_start( BlueTask_id, uart_read_task, 1 );

	status = rtems_task_delete( RTEMS_SELF );
}


