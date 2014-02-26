/**
 * @file
 *
 * @ingroup stm32f4_libi2c
 *
 * @brief LibI2C bus driver for the SPI.
 */

#include <stdbool.h>

#include <bsp/irq.h>
#include <bsp/io.h>
#include <rtems/libi2c.h>

#include <bsp/rcc.h>
#include <bsp/irq.h>
#include <bsp/irq-generic.h>


#define RTEMS_STATUS_CHECKS_USE_PRINTK

#include <rtems/status-checks.h>

#include <bsp/stm32f4xx_gpio.h>
#include <bsp/stm32f4xx_syscfg.h>
#include <bsp/stm32f4xx_exti.h>
#include <bsp/stm32f4xx_spi.h>
#include <bsp/stm32f4xx_rcc.h>

#define STM32F4_SPI_NUMBER 3

/** @defgroup STM32F4_DISCOVERY_SPI_Macros
  * @{
  */
#define SPI_CS_LOW()       GPIO_ResetBits(GPIOE, GPIO_Pin_3)
#define SPI_CS_HIGH()      GPIO_SetBits(GPIOE, GPIO_Pin_3)
/**
  * @}
  */ 

typedef struct {
  rtems_libi2c_bus_t bus;
  SPI_TypeDef *regs;
  unsigned clock;
  uint32_t idle_char;
} stm32f4_spi_bus_entry;

static rtems_status_code stm32f4_spi_init(rtems_libi2c_bus_t *bus)
{
  //rtems_status_code sc = RTEMS_SUCCESSFUL;
  //rtems_interrupt_level level;
  //stm32f4_spi_bus_entry *e = (stm32f4_spi_bus_entry *) bus;
  //volatile SPI_TypeDef *regs = e->regs;
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;

	//switch (regs) {
		//case SPI1_BASE:
			/* Enable the SPI periph */
  		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

			/* Enable SCK, MOSI and MISO GPIO clocks */
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

			/* Enable CS	GPIO clock */
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

			GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
			GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
			GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
			//break;
		//case SPI2_BASE:
			//RCC_APB2PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
			//break;
		//case SPI3_BASE:
			//RCC_APB2PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
			//break;
		//default :
			//sc = RTEMS_INVALID_ADDRESS;
			//printk("%s, %d, addr error\n", __FILE__, __LINE__);
			//return sc;
			//break;
	//}
	
	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  /* SPI SCK pin configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* SPI  MOSI pin configuration */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* SPI MISO pin configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

	
  /* SPI configuration -------------------------------------------------------*/
  SPI_I2S_DeInit((SPI_TypeDef *)SPI1_BASE);
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_Init((SPI_TypeDef *)SPI1_BASE, &SPI_InitStructure);

	/* Enable SPI1  */
  SPI_Cmd((SPI_TypeDef *)SPI1_BASE, ENABLE);

	
  /* Configure GPIO PIN for Lis Chip select */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOE, &GPIO_InitStructure);

	/* Deselect : Chip Select high */
  GPIO_SetBits(GPIOE, GPIO_Pin_3);

	return RTEMS_SUCCESSFUL;
}

static rtems_status_code stm32f4_spi_send_start(rtems_libi2c_bus_t *bus)
{
	/* Set chip select Low at the start of the transmission */
	SPI_CS_LOW();
  return RTEMS_SUCCESSFUL;
}

static rtems_status_code stm32f4_spi_send_stop(rtems_libi2c_bus_t *bus)
{
	/* Set chip select High at the end of the transmission */ 
  SPI_CS_HIGH();
  return RTEMS_SUCCESSFUL;
}

#define SPI_R_FLAG		0x1
#define SPI_W_FLAG		0x2

#define SPI_FLAG_TIMEOUT ((uint32_t)0x1000)


static int stm32f4_spi_read_write(
  rtems_libi2c_bus_t *bus,
  unsigned char *in,
  const unsigned char *out,
  int n
)
{
  //stm32f4_spi_bus_entry *e = (stm32f4_spi_bus_entry *) bus;
  //volatile SPI_TypeDef *regs = e->regs;
  //int r = 0;
  //int w = 0;
  //int dr = 1;
  //int dw = 1;
  //int m = 0;
  //unsigned char trash = 0;
  //unsigned char idle_char = (unsigned char) e->idle_char;
	unsigned int rw_flag = 0;
	unsigned int time_out = SPI_FLAG_TIMEOUT;
	unsigned char dummy_w = 0;
	unsigned char dummy_r = 0;
	unsigned char *pw = &dummy_w;
	unsigned char *pr = &dummy_r;
	int i = 0;
	

	if (in == NULL && out == NULL)
		return RTEMS_IO_ERROR;

	rw_flag |= (in != NULL) ? SPI_R_FLAG : 0;
	rw_flag |= (out != NULL) ? SPI_W_FLAG : 0;

  while (SPI_I2S_GetFlagStatus((SPI_TypeDef *)SPI1_BASE, SPI_I2S_FLAG_TXE) == RESET)
  {
    if((time_out--) == 0) 
			printk("%s, %d time out\n", __FILE__, __LINE__);
  }

	for (i=0; i<n; i++) {
		if (out != NULL) {
			pw = (unsigned char *)(out + i);
		}
	  /* Send a Byte through the SPI peripheral */
	  SPI_I2S_SendData((SPI_TypeDef *)SPI1_BASE, *pw);
		
		/* Wait to receive a Byte */
		time_out = SPI_FLAG_TIMEOUT;
		while (SPI_I2S_GetFlagStatus((SPI_TypeDef *)SPI1_BASE, SPI_I2S_FLAG_RXNE) == RESET)
		{
			if((time_out--) == 0) 
				printk("%s, %d time out\n", __FILE__, __LINE__);
		}
		if (in != NULL) {
			pr = in + i;
		}
		*pr = SPI_I2S_ReceiveData((SPI_TypeDef *)SPI1_BASE);
	}
	
  return n;
}

static rtems_status_code stm32f4_spi_send_addr(
  rtems_libi2c_bus_t *bus,
  uint32_t addr,
  int rw
)
{	
	uint8_t addr_8 = (uint8_t)addr;
	
	/* Send the Address of the indexed register */
	stm32f4_spi_read_write(bus, NULL, &addr_8, 1);
	
  return RTEMS_SUCCESSFUL;
}

static int stm32f4_spi_read(rtems_libi2c_bus_t *bus, unsigned char *in, int n)
{
  return stm32f4_spi_read_write(bus, in, NULL, n);
}

static int stm32f4_spi_write(
  rtems_libi2c_bus_t *bus,
  unsigned char *out,
  int n
)
{
  return stm32f4_spi_read_write(bus, NULL, out, n);
}

static int stm32f4_spi_ioctl(rtems_libi2c_bus_t *bus, int cmd, void *arg)
{
  int rv = -1;
  //const rtems_libi2c_tfr_mode_t *tm = (const rtems_libi2c_tfr_mode_t *) arg;
  rtems_libi2c_read_write_t *rw = (rtems_libi2c_read_write_t *) arg;
  //rtems_libi2c_read_write_async_t *rwa =
  //  (rtems_libi2c_read_write_async_t *) arg;

  switch (cmd) {
    case RTEMS_LIBI2C_IOCTL_READ_WRITE:
      rv = stm32f4_spi_read_write(bus, rw->rd_buf, rw->wr_buf, rw->byte_cnt);
      break;
    case RTEMS_LIBI2C_IOCTL_READ_WRITE_ASYNC:
      break;
    case RTEMS_LIBI2C_IOCTL_SET_TFRMODE:
      break;
    default:
      rv = -RTEMS_NOT_DEFINED;
      break;
  }

  return rv;
}

static const rtems_libi2c_bus_ops_t stm32f4_spi_ops = {
  .init = stm32f4_spi_init,
  .send_start = stm32f4_spi_send_start,
  .send_stop = stm32f4_spi_send_stop,
  .send_addr = stm32f4_spi_send_addr,
  .read_bytes = stm32f4_spi_read,
  .write_bytes = stm32f4_spi_write,
  .ioctl = stm32f4_spi_ioctl
};

static stm32f4_spi_bus_entry stm32f4_spi_bus_table [STM32F4_SPI_NUMBER] = {
  {
    /* SPI 1 */
    .bus = {
      .ops = &stm32f4_spi_ops,
      .size = sizeof(stm32f4_spi_bus_entry)
    },
    .regs = (SPI_TypeDef *) SPI1_BASE,
    .clock = 0,
    .idle_char = 0xffffffff
  }, {
    /* SPI 2 */
    .bus = {
      .ops = &stm32f4_spi_ops,
      .size = sizeof(stm32f4_spi_bus_entry)
    },
    .regs = (SPI_TypeDef *) SPI2_BASE,
    .clock = 0,
    .idle_char = 0xffffffff
  }, {
    /* SPI 3 */
    .bus = {
      .ops = &stm32f4_spi_ops,
      .size = sizeof(stm32f4_spi_bus_entry)
    },
    .regs = (SPI_TypeDef *) SPI3_BASE,
    .clock = 0,
    .idle_char = 0xffffffff
  }
};

rtems_libi2c_bus_t * const stm32f4_spi_1 =
  (rtems_libi2c_bus_t *) &stm32f4_spi_bus_table [0];

rtems_libi2c_bus_t * const stm32f4_spi_2 =
  (rtems_libi2c_bus_t *) &stm32f4_spi_bus_table [1];

rtems_libi2c_bus_t * const stm32f4_spi_3 =
  (rtems_libi2c_bus_t *) &stm32f4_spi_bus_table [2];

/*=========================================================================*\
| initialization																														|
\*=========================================================================*/

/*=========================================================================*\
| Function: 																																|
\*-------------------------------------------------------------------------*/
rtems_status_code bsp_register_spi
(
/*-------------------------------------------------------------------------*\
| Purpose:																																	|
| 	register SPI bus and devices																						|
+---------------------------------------------------------------------------+
| Input Parameters: 																												|
\*-------------------------------------------------------------------------*/
 void 																	 /* <none>												 */
)
/*-------------------------------------------------------------------------*\
| Return Value: 																														|
| 	 0 or error code																												|
\*=========================================================================*/
{
	int ret_code;
	int spi_busno;

	/*
	 * init I2C library (if not already done)
	 */
	rtems_libi2c_initialize ();

	/*
	 * init port pins used to address/select SPI devices
	 */

	/*
	 * register SPI bus
	 */
	ret_code = rtems_libi2c_register_bus("/dev/spi",
							 stm32f4_spi_1);
	if (ret_code < 0) {
		return -ret_code;
	}
	spi_busno = ret_code;
	/*
	 * FIXME: further drivers, when available
	 */
	return 0;
}



