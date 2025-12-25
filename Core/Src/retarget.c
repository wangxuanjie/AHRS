#include <stdio.h>

__asm(".global __use_no_semihosting\n\t");
struct __FILE { int handle;} ;
//extern FILE __stdout; 

void _sys_exit(int x) 
{ 
    x = x; 
}

void _ttywrch(int ch)
{
    ch = ch;
}

int stdout_putchar(int ch, FILE *f)
{  
    return ch;
}