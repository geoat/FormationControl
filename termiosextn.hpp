#pragma once
#include <stdio.h>
#include <termios.h>
#include <unistd.h>


struct termios savetty;

void termios_initialize()
{
    struct termios tty;

    tcgetattr(0, &savetty);
    tcgetattr(0, &tty);

    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN);
    tty.c_cc[VTIME] = 0;
    tty.c_cc[VMIN] = 0;

    tcsetattr(0, TCSADRAIN, &tty);
}

void termios_close()
{
    tcsetattr(0, TCSADRAIN, &savetty);
}

void termios_puts(const char *s)
{
    fprintf(stderr, "%s", s);
}

void termios_putchar(char c)
{
    putc(c, stderr);
}

int termios_getchar_nb()
{
    static unsigned char line[2];

    if (read(0, line, 1))
        return (int)line[0];

    return -1;
}

int termios_getchar()
{
    int c;

    while ((c = termios_getchar_nb()) == -1)
        ;
    return c;
}