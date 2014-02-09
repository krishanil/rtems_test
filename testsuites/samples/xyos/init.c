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


//void stm32f4_gpio_set_output(int pin, bool set);

static void xyos_menu (void)
{
	bool led_flag = false;
	char inbuf[10];

	printf("   XYOS TEST  \n");

	printf("	 p -> part_table_initialize\n");
	printf("	 f -> mount all disks in fs_table\n");
	printf("	 l -> list	file\n");
	printf("	 r -> read	file\n");
	printf("	 w -> write file\n");
	printf("	 s -> start shell\n");
	printf("	 Enter your selection ==>");
	/*
	* Wait for characters from console terminal
	*/
	stm32f4_gpio_set_config(&stm32f4_led_config_gpio[0]);
	stm32f4_gpio_set_config(&stm32f4_led_config_gpio[1]);
	stm32f4_gpio_set_config(&stm32f4_led_config_gpio[2]);
	stm32f4_gpio_set_config(&stm32f4_led_config_gpio[3]);
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


