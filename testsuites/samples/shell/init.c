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

static void
notification (int fd, int seconds_remaining, void *arg)
{
  printf(
    "Press any key to start file I/O sample (%is remaining)\n",
    seconds_remaining
  );
}

static void fileio_start_shell(void)
{
  printf("\n =========================\n");
  printf(" starting shell\n");
  printf(" =========================\n");
  rtems_shell_init(
    "SHLL",                          /* task_name */
    RTEMS_MINIMUM_STACK_SIZE * 4,    /* task_stacksize */
    100,                             /* task_priority */
    "/dev/console",                  /* devname */
    false,                           /* forever */
    true,                            /* wait */
    NULL                             /* login */
  );
}


static void fileio_menu (void)
{
  char inbuf[10];

  /*
   * Wait for characters from console terminal
   */
  for (;;) {
    printf(" =========================\n");
    printf(" RTEMS FILE I/O Test Menu \n");
    printf(" =========================\n");
    printf("   p -> part_table_initialize\n");
    printf("   f -> mount all disks in fs_table\n");
    printf("   l -> list  file\n");
    printf("   r -> read  file\n");
    printf("   w -> write file\n");
    printf("   s -> start shell\n");
    printf("   Enter your selection ==>");
    fflush(stdout);

    inbuf[0] = '\0';
    fgets(inbuf,sizeof(inbuf),stdin);
    switch (inbuf[0]) {

    case 's':
      fileio_start_shell ();
      break;
    default:
      printf("Selection `%c` not implemented\n",inbuf[0]);
      break;
    }

  }
  exit (0);
}


/*
 * RTEMS File Menu Task
 */
static rtems_task
fileio_task (rtems_task_argument ignored)
{
  fileio_menu();
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

  puts( "\n\n*** TEST FILE I/O SAMPLE ***" );

  status = rtems_shell_wait_for_input(
    STDIN_FILENO,
    20,
    notification,
    NULL
  );
  if (status == RTEMS_SUCCESSFUL) {
    Task_name = rtems_build_name('F','M','N','U');

    status = rtems_task_create(
      Task_name, 1, RTEMS_MINIMUM_STACK_SIZE * 2,
      RTEMS_DEFAULT_MODES ,
      RTEMS_FLOATING_POINT | RTEMS_DEFAULT_ATTRIBUTES, &Task_id
    );
    directive_failed( status, "create" ); 

    status = rtems_task_start( Task_id, fileio_task, 1 );
    directive_failed( status, "start" ); 

    status = rtems_task_delete( RTEMS_SELF );
    directive_failed( status, "delete" ); 
  } else {
    puts( "*** END OF TEST FILE I/O SAMPLE ***" );

    rtems_test_exit( 0 );
  }
}


#define CONFIGURE_SHELL_COMMANDS_INIT
#define CONFIGURE_SHELL_COMMANDS_ALL
#define CONFIGURE_SHELL_MOUNT_MSDOS
#define CONFIGURE_SHELL_MOUNT_RFS
#define CONFIGURE_SHELL_DEBUGRFS

#include <rtems/shellconfig.h>

