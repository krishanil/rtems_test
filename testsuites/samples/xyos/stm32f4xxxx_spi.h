/**
 * @file
 *
 * @ingroup stm32f4_libi2c
 *
 * @brief LibI2C bus driver for the SPI.
 */

#ifndef LIBBSP_ARM_STM32F4_SPI_H
#define LIBBSP_ARM_STM32F4_SPI_H

#include <rtems/libi2c.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * @ingroup stm32f4_libi2c
 *
 * @{
 */

extern rtems_libi2c_bus_t * const stm32f4_spi_1;

extern rtems_libi2c_bus_t * const stm32f4_spi_2;

extern rtems_libi2c_bus_t * const stm32f4_spi_3;

rtems_status_code bsp_register_spi (void);

/** @} */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIBBSP_ARM_STM32F4_SPI_H */
