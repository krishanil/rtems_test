/**
 * @file
 * @ingroup stm32f4_exti
 * @brief STM32F4XXXX EXTI support
 */

/*
 * Copyright (c) 2013 Christian Mauderer.  All rights reserved.
 *
 *  embedded brains GmbH
 *  Obere Lagerstr. 30
 *  82178 Puchheim
 *  Germany
 *  <rtems@embedded-brains.de>
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.com/license/LICENSE.
 */

#ifndef LIBBSP_ARM_STM32F4_STM32F4XXXX_EXTI_H
#define LIBBSP_ARM_STM32F4_STM32F4XXXX_EXTI_H

#include <bsp/utility.h>

/**
 * @defgroup stm32f4_exti EXTI Support
 * @ingroup arm_stm32f4
 * @brief STM32F4XXXX EXTI Support
 * @{
 */

typedef struct
{
  uint32_t IMR;    /*!< EXTI Interrupt mask register,            Address offset: 0x00 */
  uint32_t EMR;    /*!< EXTI Event mask register,                Address offset: 0x04 */
  uint32_t RTSR;   /*!< EXTI Rising trigger selection register,  Address offset: 0x08 */
  uint32_t FTSR;   /*!< EXTI Falling trigger selection register, Address offset: 0x0C */
  uint32_t SWIER;  /*!< EXTI Software interrupt event register,  Address offset: 0x10 */
  uint32_t PR;     /*!< EXTI Pending register,                   Address offset: 0x14 */
} stm32f4_exti;

#define STM32F4_EXTI_REG 	((volatile stm32f4_exti *) (0 + 0x40013c00))

/** @} */

#endif /* LIBBSP_ARM_STM32F4_STM32F4XXXX_EXTI_H */
