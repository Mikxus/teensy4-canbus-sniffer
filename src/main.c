#include <stdbool.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(canbus_shell, LOG_LEVEL_INF);

int main(void)
{
    while(true)
    {
        LOG_WRN("TEST");
        k_sleep(K_MSEC(1000));
    }
}