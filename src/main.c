#include <stdbool.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/can.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/sys/cbprintf.h>
#include <zephyr/random/random.h>

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

        printk("id: %02x data: ", frame.id);
        sys_be_to_cpu(frame.data, sizeof(frame.data));

        for (int i = 0; i < sizeof(frame.data) / sizeof(uint8_t); i++)
        {
            printk("%02x ", frame.data[i]);
        }
        printk("\n");
        k_sleep(K_MSEC(100));
    }
}

void lawicel_send(const struct can_frame *frame)
{
    char buf[32];
    int len = 0;
    int i;

    if (frame->flags & 1) {
        // Extended frame: Tiiiiiiiid[dd..] (T + 8 hex id + 1 hex dlc + data)
        len = snprintfcb(buf, sizeof(buf), "T%08X%1X", frame->id, frame->dlc);
    } else {
        // Standard frame: tiiid[dd..] (t + 3 hex id + 1 hex dlc + data)
        len = snprintfcb(buf, sizeof(buf), "t%03X%1X", frame->id, frame->dlc);
    }

    // Append data bytes
    for (i = 0; i < frame->dlc; i++) {
        len += snprintfcb(buf + len, sizeof(buf) - len, "%02X", frame->data[i]);
    }

    // Append CR
    buf[len++] = '\r';
    buf[len] = 0;

    printk("%s\n", &buf);
}

// SLCAN hex conversion helper
static const char hexval[] = "0123456789ABCDEF";

/**
 * @brief Send CAN frame in SLCAN format over UART (printk)
 * @param frame CAN frame to send
 */
void slcan_send(const struct can_frame *frame, bool timestamp, bool cr)
{
    char command[64];
    int pos = 0;

    // Extended frame
    if (frame->flags & CAN_FRAME_IDE) {
        if (frame->flags & CAN_FRAME_RTR) {
            command[pos++] = 'R';
        } else {
            command[pos++] = 'T';
        }
        // 29-bit ID
        pos += snprintk(&command[pos], sizeof(command) - pos, "%08X", frame->id);
        command[pos++] = hexval[frame->dlc & 0xF];
    } else {
        // Standard frame
        if (frame->flags & CAN_FRAME_RTR) {
            command[pos++] = 'r';
        } else {
            command[pos++] = 't';
        }
        // 11-bit ID
        pos += snprintk(&command[pos], sizeof(command) - pos, "%03X", frame->id & 0x7FF);
        command[pos++] = hexval[frame->dlc & 0xF];
    }

    // Data bytes
    if (!(frame->flags & CAN_FRAME_RTR)) {
        for (int i = 0; i < frame->dlc; i++) {
            command[pos++] = hexval[(frame->data[i] >> 4) & 0xF];
            command[pos++] = hexval[frame->data[i] & 0xF];
        }
    }

    // Optional timestamp (in ms, 16-bit, wraps at 60s)
    if (timestamp) {
        uint32_t time_now = k_uptime_get() % 60000;
        command[pos++] = hexval[(time_now >> 12) & 0xF];
        command[pos++] = hexval[(time_now >> 8) & 0xF];
        command[pos++] = hexval[(time_now >> 4) & 0xF];
        command[pos++] = hexval[time_now & 0xF];
    }

    command[pos++] = '\r';
    command[pos] = '\0';

    printk("%s", command);
    if (cr) {
        printk("\n");
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

    while(1)
    {
        printf("\r");
        printf("\r");
        printf("\r");
        printf("\r");
        printf("\r");
        struct can_frame rand_frame = {0};
        rand_frame.id = sys_rand32_get() & 0x7FF; // 11-bit standard ID
        rand_frame.dlc = (sys_rand32_get() % 9);  // DLC 0-8
        rand_frame.flags = 0; // Standard data frame

        for (int i = 0; i < rand_frame.dlc; i++) {
            rand_frame.data[i] = sys_rand32_get() & 0xFF;
        }

        lawicel_send(&rand_frame);

        k_sleep(K_MSEC(500));
    }


}