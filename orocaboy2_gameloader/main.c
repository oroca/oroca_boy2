#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <limits.h>
#include "uart.h"
#include "loader.h"
#include "download.h"


#define LOADER_VER "1.0.0"


int main(int argc, char *argv[])
{
  long baud;

  printf("\r\n");
  printf("[ orocaboy2_gameloader ]\r\n");
  printf("  ver : %s\r\n", LOADER_VER);

  switch( argc )
  {
    case _DEF_ARG_CNT :
      errno = 0;
      baud = strtol( argv[ _DEF_ARG_BAUD ], NULL, 10 );
      if( ( errno == ERANGE && ( baud == LONG_MAX || baud == LONG_MIN ) ) || ( errno != 0 && baud == 0 ) || ( baud < 0 ) )
      {
        fprintf( stderr, "  Invalid baud '%s'\n", argv[ _DEF_ARG_BAUD ] );
        exit( 1 );
      }

      download(argc, argv);
      break;

    case _TAG_ARG_CNT :
      downloadAddTagFile(argc, argv);
      break;

    default :
      fprintf( stderr, "  Usage1: orocaboy2_gameloader <addr|hex> <src binary> <dst binary>\n" );
      fprintf( stderr, "  Usage2: orocaboy2_gameloader <port> <baud> <addr|hex> <binary file> <verify>\n" );
      exit(1);
      break;
  }

  return 0;
}
