#include <stdio.h>

int update_number(int needed, int optional);
void print();


int main(){


    int maxi = 9;

    for (int i = 0; i<maxi; i++){
        print();
    }
    return 0;
}


void print(void){
    int number = update_number();
    printf("the new number is: %d\n", number);
}

int update_number(int needed, int optional = 1){
    static int number = 0;
    number++;
    return number;
}