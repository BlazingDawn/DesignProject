#include <stdio.h>
#include <stdlib.h>

int i = 1;
int j = 1;
int counter = 1;
int maxcount = 0;

int main( int argc, char** argv) {


while(i != 0 && j != 0){
scanf("%d %d", &i, &j);
if (i!=0 || j!=0){
if (i<1000000||j<1000000){
int var1 = i;
    int tester = i;
    int var2 = j;
while(var1 <= var2)
{
        counter = 1;
        tester = var1;
while(tester != 1)
{
    if (tester%2 == 1)
    {
        tester = 3*tester+1;
    }
    else
    {
        tester = tester/2;
    }
    counter = counter+1;;
}

var1 = var1 + 1;

if (maxcount<counter)
{
    maxcount = counter;
}

}
printf("%d", i);printf(" ");printf("%d", j);printf(" ");printf("%d\n", maxcount);
}
}
}
return 0;
}
