#include "pid.h"
#include "main.h"

void pid_reset(pid_t *pid){
    pid->integral=0.0f;
    pid->last_input=0.0f;
}

void pid_init(pid_t *pid,float kp, float ki, float kd, float out_min, float out_max){
    pid->kp=kp;
    pid->ki=ki;
    pid->kd=kd;
    pid->out_min=out_min;
    pid->out_max=out_max;

    pid_reset(pid);
}

float pid_compute(pid_t *pid, float setpoint, float input){

    float error= setpoint-input;

    float d_input= input-pid->last_input;

pid->integral+=pid->ki*error;

if(pid->integral>pid->out_max){
    pid->integral=pid->out_max;
}
else if(pid->integral<pid->out_min){
    pid->integral=pid->out_min;
}

float output= (pid->kp*error)+pid->integral-(pid->kd*d_input);

if(output>pid->out_max){
    output=pid->out_max;
}
else if(output<pid->out_min){
    output=pid->out_min;
}

pid->last_input=input;

return output;

}
