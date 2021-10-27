#include <stdio.h>

#include "controls/MotorOrientation.h"

int main(int argc, const char **argv) { 
    printf("This is a test\n");

    controls::MotorOrientation o;
    o.numMotors = 100;

    printf("%d\n", o.numMotors);

    return 0;
}