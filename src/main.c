#include <stdbool.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/can.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>

#define RX_THREAD_STACK_SIZE 512
#define RX_THREAD_PRIORITY 2

LOG_MODULE_REGISTER(canbus_sniffer, LOG_LEVEL_INF);

/**
 * @brief can device
 */
const struct device *const can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

/**
 * @brief  
 */
CAN_MSGQ_DEFINE(can_rx_que, 20);

const struct can_filter receive_all_filter = {
    .flags = 0,
    .id = 0,
    .mask = 0
};

K_THREAD_STACK_DEFINE(rx_thread_stack, 512);
struct k_thread rx_thread_data;

/**
 * @brief Can message receive thread
 * @note   
 * @param  *arg1: 
 * @param  *arg2: 
 * @param  *arg3: 
 * @retval None
 */
void rx_thread(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);
    struct can_frame frame;
    LOG_INF("rx_thread started\n");

    while(1)
    {
        k_msgq_get(&can_rx_que, &frame, K_FOREVER);

        LOG_INF("id: %d data: ", frame.id);
        sys_be_to_cpu(frame.data, sizeof(frame.data));

        for (int i = 0; i < sizeof(frame.data) / sizeof(uint8_t); i++)
        {
            LOG_INF("%02x", frame.data[i]);
        }
        LOG_INF("\n");
    }
}

int main(void)
{
    k_tid_t rx_thread_tid;
    struct can_timing timing_data;

    if (!device_is_ready(can_dev)) {
        LOG_ERR("CAN: device %s not ready\n", can_dev->name);
        return 0;
    }

    if (can_set_mode(can_dev, CAN_MODE_LISTENONLY))
    {
        LOG_ERR("CAN: Failed to set %s to listen_only mode\n", can_dev->name);
        return 0;
    }

    if (can_calc_timing(can_dev, &timing_data, 500000, 875) > 0)
    {
        LOG_ERR("CAN: sample-point error\n");
        return 0;
    }

    if (can_set_timing(can_dev, &timing_data) != 0)
    {
        LOG_ERR("CAN: failed to set timings\n");
        return 0;
    }

    if (can_start(can_dev))
    {
        LOG_ERR("CAN: Failed to start can on device: %s\n", can_dev->name);
        return 0;
    }

    if (can_add_rx_filter_msgq(can_dev, &can_rx_que, &receive_all_filter) == -ENOSPC)
    {
        LOG_ERR("CAN: error, no filter available\n");
        return 0;
    }

    rx_thread_tid = k_thread_create(&rx_thread_data, &rx_thread_stack,
        K_THREAD_STACK_SIZEOF(rx_thread_stack),
        rx_thread, NULL, NULL, NULL,
        RX_THREAD_PRIORITY, 0, K_NO_WAIT);

    if (!rx_thread_tid) {
        LOG_ERR("CAN: failed to start rx thread\n");
    }
    LOG_INF("init done\n");
}