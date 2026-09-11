#include "pid.h"

/* Clears accumulated state without touching the gains. Call this on
   re-entering TRACKING — a stale integral from the last lock would
   command a full-speed slew at a target that may be 2 degrees away. */

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

/* One control cycle. setpoint is the target bearing, input is where the
   head is currently pointing, both in degrees. Returns a slew rate in
   degrees per second, which the caller integrates into a servo angle.

   Assumes a fixed call interval — ki and kd are pre-scaled for it, so
   there is no dt term here. Changing the interval means rescaling both. */

float pid_compute(pid_t *pid, float setpoint, float input){

    float error= setpoint-input;

    float d_input= input-pid->last_input;

pid->integral+=pid->ki*error;

/* Clamping the integral as well as the output is the anti-windup fix.
       Clamping only the output looks like it works, but the integral keeps
       growing while the servo sits at a limit, and then has to unwind
       before the head responds again. */

if(pid->integral>pid->out_max){
    pid->integral=pid->out_max;
}
else if(pid->integral<pid->out_min){
    pid->integral=pid->out_min;
}

/* D acts on the measurement, not the error, and subtracts. Detection
       delivers a new bearing every sweep, so an error-based derivative
       would spike on every one of those steps. Watching the input instead
       means a setpoint jump produces no derivative at all. Cost: D also
       resists legitimate pursuit of a moving target. */

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
