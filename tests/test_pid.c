#include <stdio.h>
#include "pid.h"

int main(void){
    pid_t pid;
    pid_init(&pid, 4.0f, 0.0f, 10.0f, -30.0f, 30.0f);

    float head=0.0f;
    float target = 60.0f;
    float dt= 0.05f;

    float output;
    for(int i=0;i<100;i++){
        output=pid_compute(&pid,target,head);
        head+=output*dt;
        printf("%d head=%.2f  out=%.2f\n", i, head, output);
    }
}