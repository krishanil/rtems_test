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


