#include <iostream>
#include "getche.h"
#include "determine.hpp"

using namespace std;

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

int main(){
    char c;
    FILE *file;
    while(1){
        c = getch();
        if(c == 'q'){
            FILE* pipe = popen("python3 ../src/cam.py", "r");
            pclose(pipe);
            pair<int, int> res = determine();

            cout << res.first << " " << res.second << endl;

            file = fopen("hello.txt", "wt");
            fprintf(file, "%d", res.first * 3 + res.second + 1);
            fclose(file);
        }
        if(c == 'e'){
            break;
        }
    }
}
