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

#include "stm32f4xx_gpio.h"
#include "stm32f4xx.h"


void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  printf("GPIOX=0x%x, GPIO_Pin=0x%x\n", (unsigned int)GPIOx, GPIO_Pin);
  GPIOx->BSRRL = GPIO_Pin;
}

/**
  * @brief  Clears the selected data port bits.
  * @note   This functions uses GPIOx_BSRR register to allow atomic read/modify 
  *         accesses. In this way, there is no risk of an IRQ occurring between
  *         the read and the modify access.
  * @param  GPIOx: where x can be (A..I) to select the GPIO peripheral.
  * @param  GPIO_Pin: specifies the port bits to be written.
  *          This parameter can be any combination of GPIO_Pin_x where x can be (0..15).
  * @retval None
  */
void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  GPIOx->BSRRH = GPIO_Pin;
}
typedef struct {
	uint32_t moder;
	uint32_t otyper;
	uint32_t ospeedr;
	uint32_t pupdr;
	uint32_t idr;
	uint32_t odr;
	uint32_t bsrr;
	uint32_t lckr;
	uint32_t afr [2];
	uint32_t reserved_28 [246];
} stm32f4_gpio;
#define STM32F4_BASE 0x00

#define STM32F4_GPIO_PIN(port, index) ((((port) << 4) | (index)) & 0xff)

#define STM32F4_GPIO_PORT_OF_PIN(pin) (((pin) >> 4) & 0xf)

#define STM32F4_GPIO_INDEX_OF_PIN(pin) ((pin) & 0xf)
#define STM32F4_GPIO_TEST(i) ((STM32F4_BASE + 0x40020000) + (i))

#define STM32F4_GPIO(i) ((volatile stm32f4_gpio *) (STM32F4_BASE + 0x40020000) + (i))


void stm32f4_gpio_set_output_test(int pin, bool set)
{
  int port = STM32F4_GPIO_PORT_OF_PIN(pin);

  //STM32F4_GPIO_PIN(0x0c00, 0) ((0x0c00 0000 | (12)) & 0xff)
  // port = 0x0c00
  volatile stm32f4_gpio *gpio = STM32F4_GPIO(port);
  int test = STM32F4_GPIO_TEST(port);
  // gpio = 0x40020000 + port
  int index = STM32F4_GPIO_INDEX_OF_PIN(pin);
  int set_or_clear_offset = set ? 0 : 16;
  int a, b, c, d;
  //((volatile stm32f4_gpio *) (STM32F4_BASE + 0x40020000) + (i))
  a = ((volatile stm32f4_gpio *) (STM32F4_BASE + 0x40020000));
  b = ((volatile stm32f4_gpio *) (STM32F4_BASE + 0x40020000) + 0x3);
  printk("a=0x%x, b = 0x%x\n", a, b);
  
  //printk("pin = 0x%x, test = 0x%x\n", pin, test);

  printk("gpio=0x%x, port=0x%x, index=0x%x, set_or_clear_offset=0x%x\n", (unsigned int)gpio, port, index, set_or_clear_offset);
  // // GPIOx = 0x40020C00 + BSRRL reg = 0x1000
  gpio->bsrr = 1U << (index + set_or_clear_offset);
}



void stm32f4_gpio_set_output(int pin, bool set);

static void xyos_menu (void)
{
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
  for (;;) {
    fflush(stdout);

    inbuf[0] = '\0';
    fgets(inbuf,sizeof(inbuf),stdin);
    switch (inbuf[0]) {
    case 's':
	  GPIO_SetBits(GPIOD, GPIO_Pin_12);
	  stm32f4_gpio_set_output_test(0x00000030, true);
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


