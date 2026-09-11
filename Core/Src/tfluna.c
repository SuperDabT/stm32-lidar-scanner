#include "main.h"
#include "usart.h"
#include "tfluna.h"

/* Written in interrupt context, read from the main loop. Without volatile
   the compiler is entitled to cache these in a register, since it cannot
   see that the ISR ever runs. Invisible at -O0, breaks with optimisation.
   rx_byte needs no qualifier: its address escapes into HAL, and it is only
   ever read inside the callback. */
static volatile uint16_t last_distance = 0;
static volatile bool     last_valid    = false;
static volatile float    last_temp     = 0.0f;
static uint8_t           rx_byte;

uint16_t tfluna_distance(void) {
    return last_distance;
}

bool tfluna_valid(void) {
    return last_valid;
}

float tfluna_temperature(void) {
    return last_temp;
}

/* Byte-at-a-time frame parser for the TF-Luna's nine-byte format:

       59 59  Dist_L Dist_H  Amp_L Amp_H  Temp_L Temp_H  Checksum

   One branch per state, each handling every case within it. An earlier
   version mixed "am I hunting for a header?" with "is this byte a header?"
   in a single test; when the byte was not a header the whole branch failed
   and control fell through to the collecting branch, which began assembling
   a frame from the middle of the previous one. */
static void tfluna_feed(uint8_t byte) {
    static uint8_t frame[9];
    static uint8_t count = 0;
    uint8_t sum = 0;

    if (count == 0) {
        /* Hunting for the first header byte. Anything else is discarded. */
        if (byte == 0x59) {
            frame[0] = byte;
            count = 1;
        }
    } else if (count == 1) {
        /* Two header bytes rather than one, because 0x59 occurs in real
           data — a distance of 89 cm produces it. Two in a row is rare
           enough to sync on, and the checksum catches the rest. A miss
           here drops straight back to hunting. */
        if (byte == 0x59) {
            frame[1] = byte;
            count = 2;
        } else {
            count = 0;
        }
    } else {
        /* Collecting the seven remaining bytes. */
        frame[count] = byte;
        count++;

        if (count == 9) {
            /* Checksum is the low eight bits of the sum of bytes 0-7.
               sum is uint8_t, so the truncation happens for free. */
            for (int i = 0; i < 8; i++) {
                sum += frame[i];
            }

            if (sum == frame[8]) {
                uint16_t d     = frame[2] | (frame[3] << 8);   /* little-endian */
                uint16_t amp   = frame[4] | (frame[5] << 8);
                uint16_t t_raw = frame[6] | (frame[7] << 8);

                last_distance = d;

                /* Amplitude below 100 means too little return signal to
                   trust; 65535 means the sensor saturated. A distance of 0
                   means it could not measure, not that nothing is there. */
                last_valid = (amp >= 100) && (amp != 65535)
                             && (d >= 10) && (d <= 800);

                last_temp = t_raw / 8.0f - 256.0f;
            }

            /* Reset whether the checksum passed or not — a bad frame is
               dropped and the parser goes back to hunting for a header. */
            count = 0;
        }
    }
}

void tfluna_init(void) {
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

/* Receive is one-shot, so every byte costs a re-arm to stay listening. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        tfluna_feed(rx_byte);
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}

/* An overrun or framing error aborts the receive, and HAL routes here
   instead of RxCpltCallback. Without re-arming, the sensor goes permanently
   silent and looks like dead hardware. rx_byte is not fed to the parser:
   its contents are unknown at this point, and the parser resynchronises on
   the next header by itself. One lost byte typically costs two or three
   frames, around 20-30 ms. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}