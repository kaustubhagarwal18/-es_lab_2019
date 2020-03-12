
#include "term_serial.h"

/*------------------------------------------------------------
 * console I/O
 *------------------------------------------------------------
 */
struct termios 	savetty;

void	term_initio()
{
    struct termios tty;

    tcgetattr(0, &savetty);
    tcgetattr(0, &tty);

    tty.c_lflag &= ~(ECHO|ECHONL|ICANON|IEXTEN);
    tty.c_cc[VTIME] = 0;
    tty.c_cc[VMIN] = 0;

    tcsetattr(0, TCSADRAIN, &tty);
}

void	term_exitio()
{
    tcsetattr(0, TCSADRAIN, &savetty);
}

void	term_puts(char *s)
{
    fprintf(stderr,"%s",s);
}

void	term_putchar(char c)
{
    putc(c,stderr);
}

int	term_getchar_nb()
{
    static unsigned char 	line [2];

    if (read(0,line,1)) // note: destructive read
        return (int) line[0];

    return -1;
}

int	term_getchar()
{
    int    c;

    while ((c = term_getchar_nb()) == -1)
        ;
    return c;
}

