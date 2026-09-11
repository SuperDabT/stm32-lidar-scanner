#ifndef PID_H
#define PID_H

typedef struct{
    float kp;
    float ki;
    float kd;

    float integral;
    float last_input;

    float out_min;
    float out_max;


} pid_t;

void pid_init(pid_t *pid,float kp, float ki, float kd, float out_min, float out_max);
float pid_compute(pid_t *pid, float setpoint, float input);
void pid_reset(pid_t *pid);

#endif
