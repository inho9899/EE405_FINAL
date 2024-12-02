#include "getche.h"

// char = getch() Reads 1 character without echo
char getch(void)
{
	char ch = 0;
	tcgetattr(0, &old_tio); // Grab old_tio terminal i/o setting
	new_tio = old_tio; // Copy to new_tio
	new_tio.c_lflag &= ~ICANON; // disable buffered i/o
	new_tio.c_lflag &= ~ECHO; // Set echo mode off
	if (tcsetattr(0, TCSANOW, &new_tio) < 0) perror("tcsetattr ~ICANON");
	// Set new_tio terminal i/o setting
	if (read(0, &ch, 1) < 0) perror ("read()"); // Read one character
	if (tcsetattr(0, TCSADRAIN, &old_tio) < 0) perror ("tcsetattr ICANON");
	// Restore old terminal i/o setting

	return ch;
}
// char = getche() Reads 1 character with echo
char getche(void)
{
	char ch = 0;
	tcgetattr(0, &old_tio); // Grab old_tio terminal i/o setting
	new_tio = old_tio; // Copy to new_tio
	new_tio.c_lflag &= ~ICANON; // disable buffered i/o
	new_tio.c_lflag &= ECHO; // Set echo mode on
	if (tcsetattr(0, TCSANOW, &new_tio) < 0) perror("tcsetattr ~ICANON");
	// Set new_tio terminal i/o setting
	if (read(0, &ch, 1) < 0) perror ("read()"); // Read one character
	if (tcsetattr(0, TCSADRAIN, &old_tio) < 0) perror ("tcsetattr ICANON");
	// Restore old terminal i/o setting

	return ch;
}
