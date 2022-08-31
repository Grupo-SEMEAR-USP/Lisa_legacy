#ifndef __WIFI_H__
#define __WIFI_H__

#include <esp_err.h>

#define WIFI_SUCCESS 1 << 0
#define WIFI_FAILURE 1 << 1
#define TCP_SUCCESS 1 << 0
#define TCP_FAILURE 1 << 1
#define MAX_FAILURES 10


esp_err_t connect_wifi(void);
void http_rest_with_url(void);

#endif
