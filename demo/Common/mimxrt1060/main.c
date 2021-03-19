/*
 * Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* Copyright (c) Microsoft Corporation. All rights reserved. */
/* SPDX-License-Identifier: MIT */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>

#include "board.h"

#include "pin_mux.h"
#include <stdbool.h>

/* FreeRTOS Demo Includes */
#include "FreeRTOS.h"
#include "task.h"

#include "lwip/netifapi.h"
#include "lwip/opt.h"
#include "lwip/tcpip.h"
#include "lwip/dhcp.h"
#include "lwip/prot/dhcp.h"
#include "netif/ethernet.h"
#include "enet_ethernetif.h"
#include "fsl_phy.h"

#include "fsl_debug_console.h"

#include "clock_config.h"
#include "fsl_gpio.h"
#include "fsl_iomuxc.h"
#include "fsl_phyksz8081.h"
#include "fsl_enet_mdio.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* MAC address configuration. */
#define configMAC_ADDR                     \
    {                                      \
        0x02, 0x12, 0x13, 0x10, 0x15, 0x25 \
    }

/* Address of PHY interface. */
#define EXAMPLE_PHY_ADDRESS BOARD_ENET0_PHY_ADDRESS

/* MDIO operations. */
#define EXAMPLE_MDIO_OPS enet_ops

/* PHY operations. */
#define EXAMPLE_PHY_OPS phyksz8081_ops

/* ENET clock frequency. */
#define EXAMPLE_CLOCK_FREQ CLOCK_GetFreq(kCLOCK_IpgClk)


#ifndef EXAMPLE_NETIF_INIT_FN
/*! @brief Network interface initialization function. */
#define EXAMPLE_NETIF_INIT_FN ethernetif0_init
#endif /* EXAMPLE_NETIF_INIT_FN */

/* The lpc54018 usb logging driver calls a blocking write function. Since this
 * task is the lowest priority, all of the demo's priorities must be higher than
 * this to run. */
#define mainLOGGING_TASK_PRIORITY        ( tskIDLE_PRIORITY )
#define mainLOGGING_TASK_STACK_SIZE      ( configMINIMAL_STACK_SIZE * 4 )
#define mainLOGGING_QUEUE_LENGTH         ( 16 )

/* The task delay for allowing the lower priority logging task to print out Wi-Fi
 * failure status before blocking indefinitely. */
#define mainLOGGING_WIFI_STATUS_DELAY    pdMS_TO_TICKS( 1000 )

/*******************************************************************************
 * Variables
 ******************************************************************************/

static mdio_handle_t mdioHandle = {.ops = &EXAMPLE_MDIO_OPS};
static phy_handle_t phyHandle   = {.phyAddr = EXAMPLE_PHY_ADDRESS, .mdioHandle = &mdioHandle, .ops = &EXAMPLE_PHY_OPS};
static struct netif netif;
static ip4_addr_t netif_ipaddr, netif_netmask, netif_gw;
static ethernetif_config_t enet_config = {
        .phyHandle  = &phyHandle,
        .macAddress = configMAC_ADDR,
};

static void prvNetworkUp( void );

/*******************************************************************************
 * Code
 ******************************************************************************/

void vLoggingPrintf( const char * pcFormat,
                     ... )
{
    va_list vargs;
    va_start( vargs, pcFormat );
    vprintf( pcFormat, vargs );
    va_end( vargs );
}


int main( void )
{
	gpio_pin_config_t gpio_config = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};

    BOARD_ConfigMPU();
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();
    BOARD_InitModuleClock();

    IOMUXC_EnableMode(IOMUXC_GPR, kIOMUXC_GPR_ENET1TxClkOutputDir, true);

    /* Data cache must be temporarily disabled to be able to use sdram */
    SCB_DisableDCache();

    GPIO_PinInit(GPIO1, 9, &gpio_config);
    GPIO_PinInit(GPIO1, 10, &gpio_config);
    /* pull up the ENET_INT before RESET. */
    GPIO_WritePinOutput(GPIO1, 10, 1);
    GPIO_WritePinOutput(GPIO1, 9, 0);
    delay();
    GPIO_WritePinOutput(GPIO1, 9, 1);
    CRYPTO_InitHardware();

    mdioHandle.resource.csrClock_Hz = EXAMPLE_CLOCK_FREQ;

    vTaskStartScheduler();

    for( ; ; )
    {
    }
}

/*-----------------------------------------------------------*/

void vApplicationDaemonTaskStartupHook( void )
{
    prvNetworkUp();    
     
    /* Demos that use the network are created after the network is
    * up. */
    configPRINTF( ( "---------STARTING DEMO---------\r\n" ) );
    vStartSimpleMQTTDemo();
}

/*-----------------------------------------------------------*/

static void prvNetworkUp( void )
{
    mdioHandle.resource.csrClock_Hz = EXAMPLE_CLOCK_FREQ;

    IP4_ADDR(&netif_ipaddr, 0, 0, 0, 0);
    IP4_ADDR(&netif_netmask, 0, 0, 0, 0);
    IP4_ADDR(&netif_gw, 0, 0, 0, 0);

    tcpip_init(NULL, NULL);

    netifapi_netif_add( &netif, &netif_ipaddr, &netif_netmask, &netif_gw, &enet_config, EXAMPLE_NETIF_INIT_FN,
                        tcpip_input );
    netifapi_netif_set_default(&netif);
    netifapi_netif_set_up(&netif);

    configPRINTF( ("Getting IP address from DHCP ...\n") );
    netifapi_dhcp_start(&netif);

    struct dhcp *dhcp;
    dhcp = netif_dhcp_data(&netif);

    while (dhcp->state != DHCP_STATE_BOUND)
    {
        vTaskDelay(100);
    }

    if ( dhcp->state == DHCP_STATE_BOUND )
    {
        configPRINTF( ("\r\n IPv4 Address     : %u.%u.%u.%u\r\n", ((u8_t *)&netif.ip_addr.addr)[0],
                       ((u8_t *)&netif.ip_addr.addr)[1], ((u8_t *)&netif.ip_addr.addr)[2], ((u8_t *)&netif.ip_addr.addr)[3] ) );
    }
    
    configPRINTF( ("DHCP OK") );
}

/*-----------------------------------------------------------*/

void * pvPortCalloc( size_t xNum,
                     size_t xSize )
{
    void * pvReturn;

    pvReturn = pvPortMalloc( xNum * xSize );

    if( pvReturn != NULL )
    {
        memset( pvReturn, 0x00, xNum * xSize );
    }

    return pvReturn;
}
