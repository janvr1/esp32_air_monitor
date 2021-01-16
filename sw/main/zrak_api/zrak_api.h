#ifndef JAN_ZRAK_API
#define JAN_ZRAK_API
#include <esp_err.h>

#define ZRAK_API_URL "https://api.zrak.janvr.me/measurements?device_id=%d"

typedef struct zrak_task_params
{
    char *user;
    char *pass;
    int dev_id;
    float T;
    float RH;
    float p;
    float CO2;
    float E;
} zrak_task_params_t;

void zrak_send_measurements(char *user, char *pass, int dev_id, float T, float RH, float p, float CO2, float E);
#endif