#ifndef GETCHE_H
#define GETCHE_H

#include <termios.h>
#include <stdio.h>
#include <unistd.h>
static struct termios old_tio;
static struct termios new_tio;

char getch(void);
char getche(void);

#endif