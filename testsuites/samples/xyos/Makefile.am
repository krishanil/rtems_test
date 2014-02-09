
rtems_tests_PROGRAMS = xyos
xyos_SOURCES = init.c system.h

dist_rtems_tests_DATA = xyos.scn
dist_rtems_tests_DATA += xyos.doc

include $(RTEMS_ROOT)/make/custom/@RTEMS_BSP@.cfg
include $(top_srcdir)/../automake/compile.am
include $(top_srcdir)/../automake/leaf.am

AM_CPPFLAGS += -I$(top_srcdir)/../support/include


LINK_OBJS = $(xyos_OBJECTS)
LINK_LIBS = $(xyos_LDLIBS)

xyos$(EXEEXT): $(xyos_OBJECTS) $(xyos_DEPENDENCIES)
	@rm -f xyos$(EXEEXT)
	$(make-exe)

include $(top_srcdir)/../automake/local.am
