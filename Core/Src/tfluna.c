#include "main.h"
#include "usart.h"
#include "tfluna.h"

static volatile uint16_t last_distance=0;
static volatile bool last_valid= false;
static uint8_t rx_byte;
static volatile float last_temp=0.0f;

uint16_t tfluna_distance(void){
    return last_distance;
}

static void tfluna_feed(uint8_t byte){
    static uint8_t frame[9];
    static uint8_t count=0;
    uint8_t sum=0;


if(count==0)
{
if(byte==0x59){
frame[0]=byte;
count=1;
}
//otherwise: discard, stay at 0
}
else if(count==1){
if(byte==0x59){
frame[1]=byte;
count=2;
}
else
{
count=0;
}
}
else {
frame[count]=byte;
count++;
if(count==9){

for(int i=0; i<8; i++){

sum+=frame[i];
}
if(sum==frame[8]){
 uint16_t d= frame[2] | (frame[3] << 8);
 uint16_t amp=frame[4] |(frame[5]<<8);
 uint16_t t_raw= frame[6] |(frame[7]<<8);

 
 last_distance=d;
 last_valid=(amp>=100 ) && (amp!=65535) && (d>=10) && (d<=800);
 last_temp= t_raw/ 8.0f-256.0f;

}
count=0;
}
}
}

bool tfluna_valid(void)
{
return  last_valid;
}

void tfluna_init(void){
    HAL_UART_Receive_IT(&huart1,&rx_byte,1);
}

float tfluna_temperature(void){
    return last_temp;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if(huart->Instance==USART1){
        tfluna_feed(rx_byte);
        HAL_UART_Receive_IT(&huart1,&rx_byte,1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
    if(huart->Instance==USART1){
        // Handle UART error
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

    }
}