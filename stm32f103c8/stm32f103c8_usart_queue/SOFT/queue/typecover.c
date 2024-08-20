#include "typecover.h"

float uint8_to_float(const uint8_t *data)
{
    float result;
    memcpy(&result, data, sizeof(float));
    return result;
}

int uint8_to_int(const uint8_t *data)
{
    int result;
    memcpy(&result, data, sizeof(int));
    return result;
}

void float_to_uint8(float data, uint8_t *buf)
{
    memcpy(buf, &data, sizeof(float));
}

void int_to_uint8(int data, uint8_t *buf)
{
    memcpy(buf, &data, sizeof(int));
}
