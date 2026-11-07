/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * File Name          : ethernetif.h
  * Description        : This file provides initialization code for LWIP
  *                      middleWare.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __ETHERNETIF_H__
#define __ETHERNETIF_H__
#include <stdint.h>
#include "lwip/err.h"
#include "lwip/netif.h"
#include "cmsis_os.h"

/* Within 'USER CODE' section, code will be kept by default at each generation */
/* USER CODE BEGIN 0 */
extern ETH_HandleTypeDef heth;
/* USER CODE END 0 */

/* Exported functions ------------------------------------------------------- */
err_t ethernetif_init(struct netif *netif);

void ethernetif_input(void* argument);
void ethernet_link_thread(void* argument );

void Error_Handler(void);
u32_t sys_jiffies(void);
u32_t sys_now(void);

/* USER CODE BEGIN 1 */

void ethernet_request_recovery(void);

extern volatile uint32_t dbg_eth_rx_alloc_fail_cnt;
extern volatile uint32_t dbg_eth_rx_invalid_free_cnt;
extern volatile uint32_t dbg_eth_rx_alloc_status;
extern volatile uint32_t dbg_eth_rx_pool_in_use;
extern volatile uint32_t dbg_eth_rx_desc_ready;
extern volatile uint32_t dbg_eth_dma_error_cnt;
extern volatile uint32_t dbg_eth_last_dma_error;
extern volatile uint32_t dbg_eth_recovery_attempt_cnt;
extern volatile uint32_t dbg_eth_recovery_success_cnt;
extern volatile uint32_t dbg_eth_recovery_fail_cnt;
extern volatile uint32_t dbg_eth_last_recovery_stage;
extern volatile uint32_t dbg_eth_link_ready;

/* USER CODE END 1 */
#endif
