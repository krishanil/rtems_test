ACLOCAL_AMFLAGS = -I ../../../../aclocal

include $(top_srcdir)/../../../../automake/compile.am

include_bspdir = $(includedir)/bsp

###############################################################################
#                  Data                                                       #
###############################################################################

dist_project_lib_DATA = bsp_specs

noinst_LIBRARIES = libbspstart.a

libbspstart_a_SOURCES = ../shared/start/start.S

project_lib_DATA =
project_lib_DATA += start.$(OBJEXT)
project_lib_DATA += startup/linkcmds

EXTRA_DIST = startup/linkcmds.stm32f4
EXTRA_DIST += startup/linkcmds.stm32f105rc

###############################################################################
#                  Header                                                     #
###############################################################################

include_HEADERS =
include_HEADERS += ../../shared/include/tm27.h
include_HEADERS += include/bsp.h

nodist_include_HEADERS =
nodist_include_HEADERS += ../../shared/include/coverhd.h
nodist_include_HEADERS += include/bspopts.h

nodist_include_bsp_HEADERS =
nodist_include_bsp_HEADERS += ../../shared/include/bootcard.h

include_bsp_HEADERS =
include_bsp_HEADERS += ../../shared/include/utility.h
include_bsp_HEADERS += ../../shared/include/irq-generic.h
include_bsp_HEADERS += ../../shared/include/irq-info.h
include_bsp_HEADERS += ../../shared/include/stackalloc.h
include_bsp_HEADERS += ../../shared/tod.h
include_bsp_HEADERS += ../shared/include/start.h
include_bsp_HEADERS += ../shared/armv7m/include/armv7m-irq.h
include_bsp_HEADERS += include/irq.h
include_bsp_HEADERS += include/usart.h
include_bsp_HEADERS += include/stm32f4.h
include_bsp_HEADERS += include/stm32f10xxx_gpio.h
include_bsp_HEADERS += include/stm32f10xxx_rcc.h
include_bsp_HEADERS += include/stm32f10xxx_exti.h
include_bsp_HEADERS += include/stm32f4xxxx_gpio.h
include_bsp_HEADERS += include/stm32f4xxxx_rcc.h
include_bsp_HEADERS += include/stm32_i2c.h
include_bsp_HEADERS += include/i2c.h
include_bsp_HEADERS += include/stm32_usart.h
include_bsp_HEADERS += include/io.h
include_bsp_HEADERS += include/rcc.h

###############################################################################
#                  STM32F4-Discovery_FW_V1.1.0                                #
###############################################################################
include_bsp_HEADERS += f_include/core_cm4.h
include_bsp_HEADERS += f_include/core_cm4_simd.h
include_bsp_HEADERS += f_include/core_cmFunc.h
include_bsp_HEADERS += f_include/core_cmInstr.h
include_bsp_HEADERS += f_include/misc.h
include_bsp_HEADERS += f_include/stm32f4xx_adc.h
include_bsp_HEADERS += f_include/stm32f4xx_can.h
include_bsp_HEADERS += f_include/stm32f4xx_crc.h
include_bsp_HEADERS += f_include/stm32f4xx_cryp.h
include_bsp_HEADERS += f_include/stm32f4xx_dac.h
include_bsp_HEADERS += f_include/stm32f4xx_dbgmcu.h
include_bsp_HEADERS += f_include/stm32f4xx_dcmi.h
include_bsp_HEADERS += f_include/stm32f4xx_dma.h
include_bsp_HEADERS += f_include/stm32f4xx_exti.h
include_bsp_HEADERS += f_include/stm32f4xx_flash.h
include_bsp_HEADERS += f_include/stm32f4xx_fsmc.h
include_bsp_HEADERS += f_include/stm32f4xx_gpio.h
include_bsp_HEADERS += f_include/stm32f4xx.h
include_bsp_HEADERS += f_include/stm32f4xx_hash.h
include_bsp_HEADERS += f_include/stm32f4xx_i2c.h
include_bsp_HEADERS += f_include/stm32f4xx_iwdg.h
include_bsp_HEADERS += f_include/stm32f4xx_pwr.h
include_bsp_HEADERS += f_include/stm32f4xx_rcc.h
include_bsp_HEADERS += f_include/stm32f4xx_rng.h
include_bsp_HEADERS += f_include/stm32f4xx_rtc.h
include_bsp_HEADERS += f_include/stm32f4xx_sdio.h
include_bsp_HEADERS += f_include/stm32f4xx_spi.h
include_bsp_HEADERS += f_include/stm32f4xx_syscfg.h
include_bsp_HEADERS += f_include/stm32f4xx_tim.h
include_bsp_HEADERS += f_include/stm32f4xx_usart.h
include_bsp_HEADERS += f_include/stm32f4xx_wwdg.h
include_bsp_HEADERS += f_include/system_stm32f4xx.h

###############################################################################
#                  LibBSP                                                     #
###############################################################################

noinst_LIBRARIES += libbsp.a

libbsp_a_SOURCES =
libbsp_a_CPPFLAGS =
libbsp_a_LIBADD =

# Shared
libbsp_a_SOURCES += ../../shared/bootcard.c
libbsp_a_SOURCES += ../../shared/bspclean.c
libbsp_a_SOURCES += ../../shared/bspgetworkarea.c
libbsp_a_SOURCES += ../../shared/bsplibc.c
libbsp_a_SOURCES += ../../shared/bsppost.c
libbsp_a_SOURCES += ../../shared/bsppredriverhook.c
libbsp_a_SOURCES += ../../shared/bsppretaskinghook.c
libbsp_a_SOURCES += ../../shared/gnatinstallhandler.c
libbsp_a_SOURCES += ../../shared/sbrk.c
libbsp_a_SOURCES += ../../shared/src/stackalloc.c

# Startup
libbsp_a_SOURCES += ../shared/startup/bsp-start-memcpy.S
libbsp_a_SOURCES += startup/bspstart.c
libbsp_a_SOURCES += startup/bspstarthook.c
libbsp_a_SOURCES += startup/bspreset.c
libbsp_a_SOURCES += startup/io.c
libbsp_a_SOURCES += startup/rcc.c
libbsp_a_SOURCES += startup/start-config-io.c

# IRQ
libbsp_a_SOURCES += ../../shared/src/irq-default-handler.c
libbsp_a_SOURCES += ../../shared/src/irq-generic.c
libbsp_a_SOURCES += ../../shared/src/irq-info.c
libbsp_a_SOURCES += ../../shared/src/irq-legacy.c
libbsp_a_SOURCES += ../../shared/src/irq-server.c
libbsp_a_SOURCES += ../../shared/src/irq-shell.c
libbsp_a_SOURCES += ../shared/armv7m/irq/armv7m-irq.c
libbsp_a_SOURCES += ../shared/armv7m/irq/armv7m-irq-dispatch.c

# Console
libbsp_a_SOURCES += ../../shared/console.c
libbsp_a_SOURCES += ../../shared/console_control.c
libbsp_a_SOURCES += ../../shared/console_read.c
libbsp_a_SOURCES += ../../shared/console_select.c
libbsp_a_SOURCES += ../../shared/console_write.c
libbsp_a_SOURCES += console/console-config.c
libbsp_a_SOURCES += console/usart.c

# I2C
libbsp_a_SOURCES += i2c/i2c.c
libbsp_a_SOURCES += i2c/i2c-config.c

###############################################################################
#                  STM32F4-Discovery_FW_V1.1.0                                #
###############################################################################
libbsp_a_SOURCES += f_c/misc.c               
libbsp_a_SOURCES += f_c/stm32f4xx_adc.c      
libbsp_a_SOURCES += f_c/stm32f4xx_can.c      
libbsp_a_SOURCES += f_c/stm32f4xx_crc.c      
libbsp_a_SOURCES += f_c/stm32f4xx_cryp_aes.c 
libbsp_a_SOURCES += f_c/stm32f4xx_cryp.c     
libbsp_a_SOURCES += f_c/stm32f4xx_cryp_des.c 
libbsp_a_SOURCES += f_c/stm32f4xx_cryp_tdes.c
libbsp_a_SOURCES += f_c/stm32f4xx_dac.c      
libbsp_a_SOURCES += f_c/stm32f4xx_dbgmcu.c   
libbsp_a_SOURCES += f_c/stm32f4xx_dcmi.c     
libbsp_a_SOURCES += f_c/stm32f4xx_dma.c      
libbsp_a_SOURCES += f_c/stm32f4xx_exti.c     
libbsp_a_SOURCES += f_c/stm32f4xx_flash.c    
libbsp_a_SOURCES += f_c/stm32f4xx_fsmc.c     
libbsp_a_SOURCES += f_c/stm32f4xx_gpio.c     
libbsp_a_SOURCES += f_c/stm32f4xx_hash.c     
libbsp_a_SOURCES += f_c/stm32f4xx_hash_md5.c 
libbsp_a_SOURCES += f_c/stm32f4xx_hash_sha1.c
libbsp_a_SOURCES += f_c/stm32f4xx_i2c.c      
libbsp_a_SOURCES += f_c/stm32f4xx_iwdg.c     
libbsp_a_SOURCES += f_c/stm32f4xx_pwr.c      
libbsp_a_SOURCES += f_c/stm32f4xx_rcc.c      
libbsp_a_SOURCES += f_c/stm32f4xx_rng.c      
libbsp_a_SOURCES += f_c/stm32f4xx_rtc.c      
libbsp_a_SOURCES += f_c/stm32f4xx_sdio.c     
libbsp_a_SOURCES += f_c/stm32f4xx_spi.c      
libbsp_a_SOURCES += f_c/stm32f4xx_syscfg.c   
libbsp_a_SOURCES += f_c/stm32f4xx_tim.c      
libbsp_a_SOURCES += f_c/stm32f4xx_usart.c    
libbsp_a_SOURCES += f_c/stm32f4xx_wwdg.c

# Clock
libbsp_a_SOURCES += ../../shared/clockdrv_shell.h
libbsp_a_SOURCES += ../shared/armv7m/clock/armv7m-clock-config.c

# Timer
libbsp_a_SOURCES += ../../shared/timerstub.c

# Cache
libbsp_a_SOURCES += ../../../libcpu/shared/src/cache_manager.c
libbsp_a_SOURCES += ../../../libcpu/arm/shared/cache/cache_.h
libbsp_a_CPPFLAGS += -I$(srcdir)/../../../libcpu/arm/shared/include

###############################################################################
#                  Special Rules                                              #
###############################################################################

DISTCLEANFILES = include/bspopts.h

include $(srcdir)/preinstall.am
include $(top_srcdir)/../../../../automake/local.am
