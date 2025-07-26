#include<iostream>
#include<fstream>
#include<cstring>

using namespace std;

char buf[256];

int main(){
    FILE *file = fopen("hello.txt", "wt");
    // fprintf(file, "%d", 1);
    int res;
    if(fgets(buf, sizeof(buf), file) != nullptr){
        res = buf[0]-'0';
        printf("%d\n", res);
    }
}