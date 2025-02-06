# frdm_rw612_simple_tcp_echo
Simple TCP echo that works on the FRDM-RW612. It's just a clean version of the SDK example. 

## How to use it
1. In McuXpresso, clone the lwip_ipv4_ipv6_echo_freertos demo project from the FRDM-RW612 SDK version 24.12.00.
2. Copy the simple_tcp_echo.c and .h files into the demo project's source folder.
3. Modify source\lwip_ipv4_ipv6_echo_freertos.c to include simple_tcp_echo.h and in ```main()``` call ```STE_network_interface_init()``` instead of ```sys_thread_new()```.
4. Connect an Ethernet cable between your PC and the FRDM-RW612.
5. Configure your PC's Ethernet adapter to use an static IP address 192.168.0.100. 
6. Build and debug the project with a serial debug terminal open.
7. Use the tcp_echo.py script to test the Echo application.

### Original main:
```C
int main(void)
{
    BOARD_InitHardware();

    /* Initialize lwIP from thread */
    if (sys_thread_new("main", stack_init, NULL, INIT_THREAD_STACKSIZE, INIT_THREAD_PRIO) == NULL)
    {
        LWIP_ASSERT("main(): Task creation failed.", 0);
    }

    vTaskStartScheduler();

    /* Will not get here unless a task calls vTaskEndScheduler ()*/
    return 0;
}
```

### Updated main

```C
int main(void)
{
    BOARD_InitHardware();

    STE_network_interface_init();
    vTaskStartScheduler();

    /* Will not get here unless a task calls vTaskEndScheduler ()*/
    return 0;
}
```
