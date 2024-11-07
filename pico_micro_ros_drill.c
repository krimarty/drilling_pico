#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/u_int8.h>
#include <std_msgs/msg/int16.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "storage_driver.h"
#include "linear_driver.h"

#define I2C_PORT i2c0

const uint LED_PIN = 25;
struct storage storage;
struct linear linear;


rcl_publisher_t publisher;
std_msgs__msg__Int16 weight;
rcl_subscription_t subscriber;
std_msgs__msg__UInt8 ros_command;


void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    //if (storage_read(&storage) < 0)
    if (linear_read(&linear) < 0)
    {
        weight.data = 69;
        rcl_ret_t ret = rcl_publish(&publisher, &weight, NULL);
    }
    else
    {
    //weight.data = storage.raw;
    weight.data = linear.states;
    rcl_ret_t ret = rcl_publish(&publisher, &weight, NULL);
    }
}

void subscription_callback(const void* msgin)
{
    const std_msgs__msg__UInt8 * msg = (const std_msgs__msg__UInt8 *)msgin;
    printf("Prijaty prikaz: %d\n", msg->data);
    //storage.command = msg->data;
    linear.command = msg->data;
    linear.speed = 69;
    linear_write(&linear);
    //storage_write(&storage);
 }


int main()
{
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico_node", "", &support);
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
        "sample_weight");

    //init subscriber
    rcl_ret_t rc = rclc_subscription_init_default(
        &subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
        "storage_command");

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        timer_callback);

    rclc_executor_init(&executor, &support.context, 2, &allocator); //the number of executors
    rclc_executor_add_timer(&executor, &timer);
    //init executor for subscriber
    rc = rclc_executor_add_subscription(
        &executor, &subscriber, &ros_command,
        &subscription_callback, ON_NEW_DATA);

    //init i2c
    i2c_init(I2C_PORT, 400000);
    gpio_set_function(4, GPIO_FUNC_I2C);
    gpio_set_function(5, GPIO_FUNC_I2C);
    gpio_pull_up(4);
    gpio_pull_up(5);


    gpio_put(LED_PIN, 1);

    weight.data = 0;
    storage_init(&storage);
    linear_init(&linear);

    while (true)
    {
        printf("Zkouska gitu");
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}
