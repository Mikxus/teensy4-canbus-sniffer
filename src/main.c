#include <stdbool.h>

#include <zephyr/kernel.h>
#include <zephry/logging/log.h>

int main(void)
{
    while(true)
    {
        LOG_WARN("TEST");
        k_sleep(K_MSEC(1000));
    }
}