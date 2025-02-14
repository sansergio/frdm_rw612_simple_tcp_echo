/*
 * simple_tcp_echo.c
 *
 *  Created on: Feb 5, 2025
 *      Author: snico
 */

#include "simple_tcp_echo.h"
#include "lwip/opt.h"

#include "lwip/dhcp.h"
#include "lwip/sys.h"
#include "lwip/netifapi.h"
#include "lwip/prot/dhcp.h"
#include "lwip/netif.h"
#include "lwip/tcpip.h"
#include "lwip/sockets.h"
#include "netif/ethernet.h"
#include "ethernetif.h"

#include "app.h"
/* Must be after include of app.h */
#include "fsl_silicon_id.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief Stack size of the temporary lwIP initialization thread. */
#define INIT_THREAD_STACKSIZE 1024
/*! @brief Priority of the temporary lwIP initialization thread. */
#define INIT_THREAD_PRIO DEFAULT_THREAD_PRIO
/* Ethernet interface Initialization method */
#ifndef EXAMPLE_NETIF_INIT_FN
#define EXAMPLE_NETIF_INIT_FN ethernetif0_init
#endif
/* Number of network physical interfaces used */
#define BOARD_PHY_COUNT 1
/* Max number of TCP clients supported at the same time. */
#define TCP_SERVER_CONNECTIONS_MAX 2
/* Use DHCP instead of static IP */
#define USE_DHCP (0)
/*! @brief Priority of the thread which prints DHCP info. */
#define PRINT_THREAD_PRIO DEFAULT_THREAD_PRIO
/*! @brief Stack size of the thread which prints DHCP info. */
#define PRINT_THREAD_STACKSIZE 512

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* Network interface init task */
static void network_interface_init(void *arg);
/* TCP echo connection task */
static void tcp_listen_for_client_connections(void *arg);
/* TCP Echo server task*/
static void tcp_echo_server_thread(void *arg);
/* TCP echo implementation */
static void echo_loop_tcp(int sck);
/* Set the IP address to socket address structure. */
static int ip_port_to_sockaddr(const char *ip_str,
                               int port,
                               struct sockaddr_in *ipv4,
                               struct sockaddr_in6 *ipv6);
/* Called by the netif module to show link status */
static void linkStatusCallback(struct netif *netif_,
		                       netif_nsc_reason_t reason,
							   const netif_ext_callback_args_t *args);
/* Print DHCP */
static void print_dhcp_state(void *arg);

/*******************************************************************************
 * Variables
 ******************************************************************************/
static phy_handle_t phyHandle;
static netif_ext_callback_t linkStatusCallbackInfo;

/*******************************************************************************
 * Interface Code
 ******************************************************************************/
/* Creates the Network interface initialization task */
void STE_network_interface_init(void)
{
    /* Initialize lwIP from thread */
    if (sys_thread_new("STE_netif_init", network_interface_init, NULL, INIT_THREAD_STACKSIZE, INIT_THREAD_PRIO) == NULL)
    {
        LWIP_ASSERT("STE_network_interface_init(): Task creation failed.", 0);
    }
}

/* Creates the TCP Echo task */
void STE_start_tcp_echo_server(void)
{
    /* Initialize lwIP from thread */
    if (sys_thread_new("STE_tcp_listen", tcp_listen_for_client_connections, NULL, INIT_THREAD_STACKSIZE, INIT_THREAD_PRIO) == NULL)
    {
        LWIP_ASSERT("STE_start_tcp_echo(): Task creation failed.", 0);
    }
}

/*******************************************************************************
 * Static Code
 ******************************************************************************/
/*!
 * @brief Initializes this application's network interface
 *
 * @param arg unused
 */
static void network_interface_init(void *arg)
{
    LWIP_UNUSED_ARG(arg);

    ip4_addr_t netif0_ipaddr, netif0_netmask, netif0_gw;
    static struct netif s_netif0;
    ethernetif_config_t enet0_config = {.phyHandle   = &phyHandle,
                                        .phyAddr     = EXAMPLE_PHY_ADDRESS,
                                        .phyOps      = EXAMPLE_PHY_OPS,
                                        .phyResource = EXAMPLE_PHY_RESOURCE,
                                        .srcClockHz  = EXAMPLE_CLOCK_FREQ,
    };
    (void)SILICONID_ConvertToMacAddr(&enet0_config.macAddress);

    tcpip_init(NULL, NULL);

    LOCK_TCPIP_CORE();
    netif_add_ext_callback(&linkStatusCallbackInfo, linkStatusCallback);
    UNLOCK_TCPIP_CORE();
#if USE_DHCP
    netifapi_netif_add(&s_netif0, NULL, NULL, NULL, &enet0_config, EXAMPLE_NETIF_INIT_FN, tcpip_input);
#else
    IP4_ADDR(&netif0_ipaddr, configIP_ADDR0, configIP_ADDR1, configIP_ADDR2, configIP_ADDR3);
    IP4_ADDR(&netif0_netmask, configNET_MASK0, configNET_MASK1, configNET_MASK2, configNET_MASK3);
    IP4_ADDR(&netif0_gw, configGW_ADDR0, configGW_ADDR1, configGW_ADDR2, configGW_ADDR3);
    netifapi_netif_add(&s_netif0, &netif0_ipaddr, &netif0_netmask, &netif0_gw, &enet0_config, EXAMPLE_NETIF_INIT_FN,
                       tcpip_input);
#endif
    netifapi_netif_set_up(&s_netif0);

    /*
     * Single netif is used, set is as default to avoid
     * the need to append zone indices to link-local IPv6 addresses.
     */
    netifapi_netif_set_default(&s_netif0);

#if !USE_DHCP
    LOCK_TCPIP_CORE();
    netif_create_ip6_linklocal_address(&s_netif0, 1);
    UNLOCK_TCPIP_CORE();
#endif
    struct netif *netif_array[BOARD_PHY_COUNT];
    netif_array[0] = &s_netif0;

    while (ethernetif_wait_linkup_array(netif_array, BOARD_PHY_COUNT, 5000) != ERR_OK)
    {
        PRINTF("PHY Auto-negotiation failed. Please check the cable connection and link partner setting.\r\n");
    }

#if USE_DHCP
    PRINTF("Enabling DHCP!\r\n");
    netifapi_dhcp_start(&s_netif0);

    if (sys_thread_new("print_dhcp", print_dhcp_state, &s_netif0, PRINT_THREAD_STACKSIZE, PRINT_THREAD_PRIO) == NULL)
    {
        LWIP_ASSERT("stack_init(): Task creation failed.", 0);
    }
#else
    PRINTF("************************************************\r\n");
    PRINTF(" Interface name   : %s%d\r\n", netif_default->name, netif_default->num);
    PRINTF(" IPv4 Address     : %s\r\n", ip4addr_ntoa(netif_ip4_addr(netif_default)));
    PRINTF(" IPv4 Subnet mask : %s\r\n", ip4addr_ntoa(netif_ip4_netmask(netif_default)));
    PRINTF(" IPv4 Gateway     : %s\r\n", ip4addr_ntoa(netif_ip4_gw(netif_default)));
    for (int i = 0; i < LWIP_IPV6_NUM_ADDRESSES; i++)
    {
        const char *str_ip = "-";
        if (ip6_addr_isvalid(netif_ip6_addr_state(netif_default, i)))
        {
            str_ip = ip6addr_ntoa(netif_ip6_addr(netif_default, i));
        }
        PRINTF(" IPv6 Address%d    : %s\r\n", i, str_ip);
    }
    PRINTF("************************************************\r\n");

    STE_start_tcp_echo_server();
#endif
    vTaskDelete(NULL);
}

/*!
 * @brief TCP server task for listening for client connections.
 *
 * @param arg unused
 */
static void tcp_listen_for_client_connections(void *arg)
{
    int sck;
    int sck_accepted[TCP_SERVER_CONNECTIONS_MAX];
    int af;
    struct sockaddr_in ipv4;
    struct sockaddr_in6 ipv6;
    int ret;

	PRINTF("TCP echo task strat\r\n");

	af = ip_port_to_sockaddr("0.0.0.0", 57777, &ipv4, &ipv6);

    PRINTF("Creating new socket.\r\n");
    sck = socket(af, SOCK_STREAM, 0);
    if (sck < 0)
    {
        PRINTF("Socket creation failed. (%d)\r\n", sck);
        return;
    }

    ret = bind(sck, (struct sockaddr *)&ipv4, sizeof(ipv4));
    if (ret < 0)
    {
        PRINTF("bind() failed (errno=%d)\r\n", errno);
    }
    else
    {
        PRINTF("Waiting for incoming connection.\r\n");
        listen(sck, 0);
        fcntl(sck, F_SETFL, O_NONBLOCK);

        int accepted_sck_cnt = 0;
        while (1)
        {
            if (accepted_sck_cnt >= TCP_SERVER_CONNECTIONS_MAX)
            {
                // Reached maximum connections.
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }
            sck_accepted[accepted_sck_cnt] = accept(sck, NULL, 0);
            if (sck_accepted[accepted_sck_cnt] < 0)
            {
                // Nothing to accept. Wait 50ms and try it again.
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            else
            {
                PRINTF("\r\nAccepted connection\r\n");

                // Create thread that serves this connection
                sys_thread_t thread =
                    sys_thread_new("tcp_echo_server_thread", tcp_echo_server_thread,
                                   (void *)&(sck_accepted[accepted_sck_cnt]), 1024, DEFAULT_THREAD_PRIO);

                if (thread == NULL)
                {
                    PRINTF("Can not create TCP connection server thread\r\n");
                    close(sck_accepted[accepted_sck_cnt]);
                    sck_accepted[accepted_sck_cnt] = -1;
                }
                else
                {
                    accepted_sck_cnt += 1;
                }
            }
        }
        // Listen thread stopped, clean up connections
        accepted_sck_cnt = 0;
    }

    if (sck != -1)
    {
        close(sck);
        sck = -1;
    }

    vTaskDelete(NULL);

}

/*!
 * @brief TCP server task that echoes received data.
 *
 * @param arg Connected socket
 */
static void tcp_echo_server_thread(void *arg)
{
    int sck = *((int *)arg);

    echo_loop_tcp(sck);

    if (sck != -1)
    {
        close(sck);
        sck = -1;
    }

    vTaskDelete(NULL);
}

/*!
 * @brief TCP echo implementation.
 *
 * @param sck Connected socket
 */
static void echo_loop_tcp(int sck)
{
    int err;
    uint8_t buf[1500];

    PRINTF("\r\nEchoing data.\r\n");
    PRINTF("\r\n");

    struct timeval timeout = {.tv_usec = 50 * 1000, .tv_sec = 0};

    err = lwip_setsockopt(sck, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    if (err)
    {
        PRINTF("Setting socket receive timeout failed (%d).\r\n", err);
        return;
    }

    while (1)
    {
        ssize_t bytes = read(sck, &buf, sizeof(buf));
        if (bytes > 0)
        {
            bytes = write(sck, &buf, bytes);
            if (bytes >= 0)
            {
                PRINTF("%dB sent back.\r\n", bytes);
            }
            else
            {
                PRINTF("write() failed (errno=%d)\r\n", errno);
            }
        }
        else if (errno == EWOULDBLOCK)
        {
            // Timeout is here to allow check if we should continue so call read again.
        }
        else
        {
            PRINTF("Connection terminated. (errno=%d).\r\n", errno);
            return;
        }
    }
}

/*!
 * @brief Set the IP address to socket address structure.
 *
 * @param ip_str string with the IP address
 * @param port   TCP port
 * @param ipv4   sockaddr_in structure to fill if the IP address is IPv4
 * @param ipv6   sockaddr_in structure to fill if the IP address is IPv6
 * @return Address family: AF_INET or AF_INET6 depending of the type of IP address passed.
 */
static int ip_port_to_sockaddr(const char *ip_str,
                               int port,
                               struct sockaddr_in *ipv4,
                               struct sockaddr_in6 *ipv6)
{
    int ret;
    int af;

    /* Convert IP */
    af = AF_INET;
    memset(ipv4, 0, sizeof(struct sockaddr_in));
    ipv4->sin_len    = sizeof(struct sockaddr_in);
    ipv4->sin_family = af;
    ipv4->sin_port   = htons(port);
    ret              = inet_pton(af, ip_str, &ipv4->sin_addr.s_addr);
    if (ret != 1)
    {
        /* Address is not valid IPv4 address. Lets try treat it as IPv6 */
        af = AF_INET6;
        memset(ipv6, 0, sizeof(struct sockaddr_in6));
        ipv6->sin6_len      = sizeof(struct sockaddr_in6);
        ipv6->sin6_family   = af;
        ipv6->sin6_port     = htons(port);
        ipv6->sin6_scope_id = netif_get_index(netif_default);

        LOCK_TCPIP_CORE();
        ret = inet_pton(af, ip_str, &ipv6->sin6_addr.s6_addr);
        UNLOCK_TCPIP_CORE();

        if (ret != 1)
        {
            PRINTF("'%s' is not valid IPv4 nor IPv6 address.\r\n", ip_str);
            return -1;
        }
    }

    return af;
}

/*!
 * @brief Link status callback - prints link status events.
 */
static void linkStatusCallback(struct netif *netif_, netif_nsc_reason_t reason, const netif_ext_callback_args_t *args)
{
    if (reason != LWIP_NSC_LINK_CHANGED)
        return;

    PRINTF("[LINK STATE] netif=%d, state=%s", netif_->num, args->link_changed.state ? "up" : "down");

    if (args->link_changed.state)
    {
        char *speedStr;
        switch (ethernetif_get_link_speed(netif_))
        {
            case kPHY_Speed10M:
                speedStr = "10M";
                break;
            case kPHY_Speed100M:
                speedStr = "100M";
                break;
            case kPHY_Speed1000M:
                speedStr = "1000M";
                break;
            default:
                speedStr = "N/A";
                break;
        }

        char *duplexStr;
        switch (ethernetif_get_link_duplex(netif_))
        {
            case kPHY_HalfDuplex:
                duplexStr = "half";
                break;
            case kPHY_FullDuplex:
                duplexStr = "full";
                break;
            default:
                duplexStr = "N/A";
                break;
        }

        PRINTF(", speed=%s_%s", speedStr, duplexStr);
    }

    PRINTF("\r\n");
}

/*!
 * @brief Prints DHCP status of the interface when it has changed from last status.
 *
 * @param arg pointer to network interface structure
 */
static void print_dhcp_state(void *arg)
{
    struct netif *netif = (struct netif *)arg;
    struct dhcp *dhcp;
    u8_t dhcp_last_state = DHCP_STATE_OFF;

    while (netif_is_up(netif))
    {
        dhcp = netif_dhcp_data(netif);

        if (dhcp == NULL)
        {
            dhcp_last_state = DHCP_STATE_OFF;
        }
        else if (dhcp_last_state != dhcp->state)
        {
            dhcp_last_state = dhcp->state;

            PRINTF(" DHCP state       : ");
            switch (dhcp_last_state)
            {
                case DHCP_STATE_OFF:
                    PRINTF("OFF");
                    break;
                case DHCP_STATE_REQUESTING:
                    PRINTF("REQUESTING");
                    break;
                case DHCP_STATE_INIT:
                    PRINTF("INIT");
                    break;
                case DHCP_STATE_REBOOTING:
                    PRINTF("REBOOTING");
                    break;
                case DHCP_STATE_REBINDING:
                    PRINTF("REBINDING");
                    break;
                case DHCP_STATE_RENEWING:
                    PRINTF("RENEWING");
                    break;
                case DHCP_STATE_SELECTING:
                    PRINTF("SELECTING");
                    break;
                case DHCP_STATE_INFORMING:
                    PRINTF("INFORMING");
                    break;
                case DHCP_STATE_CHECKING:
                    PRINTF("CHECKING");
                    break;
                case DHCP_STATE_BOUND:
                    PRINTF("BOUND");
                    break;
                case DHCP_STATE_BACKING_OFF:
                    PRINTF("BACKING_OFF");
                    break;
                default:
                    PRINTF("%u", dhcp_last_state);
                    assert(0);
                    break;
            }
            PRINTF("\r\n");

            if (dhcp_last_state == DHCP_STATE_BOUND)
            {
                PRINTF("\r\n IPv4 Address     : %s\r\n", ipaddr_ntoa(&netif->ip_addr));
                PRINTF(" IPv4 Subnet mask : %s\r\n", ipaddr_ntoa(&netif->netmask));
                PRINTF(" IPv4 Gateway     : %s\r\n\r\n", ipaddr_ntoa(&netif->gw));
                STE_start_tcp_echo_server();
            }
        }

        sys_msleep(20U);
    }

    vTaskDelete(NULL);
}
