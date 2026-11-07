/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : ethernetif.c
  * Description        : This file provides code for the configuration
  *                      of the ethernetif.c MiddleWare.
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

/* Includes ------------------------------------------------------------------*/
#include <ethernetif.h>
#include "main.h"
#include "lwip/opt.h"
#include "lwip/timeouts.h"
#include "netif/ethernet.h"
#include "netif/etharp.h"
#include "lwip/ethip6.h"
#include "lan8742.h"
#include <string.h>
#include <stdbool.h>
#include "cmsis_os.h"
#include "lwip/tcpip.h"

/* Within 'USER CODE' section, code will be kept by default at each generation */
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* Private define ------------------------------------------------------------*/
/* The time to block waiting for input. */
#define TIME_WAITING_FOR_INPUT ( portMAX_DELAY )
/* Time to block waiting for transmissions to finish */
#define ETHIF_TX_TIMEOUT (2000U)
/* USER CODE BEGIN OS_THREAD_STACK_SIZE_WITH_RTOS */
/* Stack size of the interface thread */
#define INTERFACE_THREAD_STACK_SIZE ( 2048 )
/* USER CODE END OS_THREAD_STACK_SIZE_WITH_RTOS */
/* Network interface name */
#define IFNAME0 's'
#define IFNAME1 't'

/* ETH Setting  */
#define ETH_DMA_TRANSMIT_TIMEOUT               ( 20U )
#define ETH_TX_BUFFER_MAX             ((ETH_TX_DESC_CNT) * 2U)
#define ETH_DMA_STOP_TIMEOUT_MS       (100U)
#define ETH_RECOVERY_RETRY_MS         (1000U)

/* USER CODE BEGIN 1 */
volatile uintptr_t addr_rx_desc = 0;
volatile uintptr_t addr_tx_desc = 0;
volatile uintptr_t addr_rx_buf  = 0;
/* USER CODE END 1 */

/* Private variables ---------------------------------------------------------*/
/*
@Note: This interface is implemented to operate in zero-copy mode only:
        - Rx Buffers will be allocated from LwIP stack Rx memory pool,
          then passed to ETH HAL driver.
        - Tx Buffers will be allocated from LwIP stack memory heap,
          then passed to ETH HAL driver.

@Notes:
  1.a. ETH DMA Rx descriptors must be contiguous, the default count is 4,
       to customize it please redefine ETH_RX_DESC_CNT in ETH GUI (Rx Descriptor Length)
       so that updated value will be generated in stm32xxxx_hal_conf.h
  1.b. ETH DMA Tx descriptors must be contiguous, the default count is 4,
       to customize it please redefine ETH_TX_DESC_CNT in ETH GUI (Tx Descriptor Length)
       so that updated value will be generated in stm32xxxx_hal_conf.h

  2.a. Rx Buffers number must be between ETH_RX_DESC_CNT and 2*ETH_RX_DESC_CNT
  2.b. Rx Buffers must have the same size: ETH_RX_BUF_SIZE, this value must
       passed to ETH DMA in the init field (heth.Init.RxBuffLen)
  2.c  The RX Ruffers addresses and sizes must be properly defined to be aligned
       to L1-CACHE line size (32 bytes).
*/

/* Data Type Definitions */
typedef enum
{
  RX_ALLOC_OK       = 0x00,
  RX_ALLOC_ERROR    = 0x01
} RxAllocStatusTypeDef;

typedef struct
{
  struct pbuf_custom pbuf_custom;
  uint8_t buff[(ETH_RX_BUF_SIZE + 31) & ~31] __ALIGNED(32);
} RxBuff_t;

/* Memory Pool Declaration */
#define ETH_RX_BUFFER_CNT             12U
LWIP_MEMPOOL_DECLARE(RX_POOL, ETH_RX_BUFFER_CNT, sizeof(RxBuff_t), "Zero-copy RX PBUF pool");

typedef enum
{
  RX_OWNER_DRIVER = 1U,
  RX_OWNER_LWIP = 2U
} RxOwnerTypeDef;

typedef struct
{
  struct pbuf_custom *pbuf;
  uint8_t owner;
} RxBufferTrackTypeDef;

/* Variable Definitions */
static volatile uint8_t RxAllocStatus;
static RxBufferTrackTypeDef g_rx_buffers[ETH_RX_BUFFER_CNT];
static osMutexId_t g_eth_io_mutex;
static osThreadId_t g_eth_input_thread;

volatile uint32_t dbg_eth_rx_alloc_fail_cnt = 0U;
volatile uint32_t dbg_eth_rx_invalid_free_cnt = 0U;
volatile uint32_t dbg_eth_rx_alloc_status = RX_ALLOC_OK;
volatile uint32_t dbg_eth_rx_pool_in_use = 0U;
volatile uint32_t dbg_eth_rx_desc_ready = 0U;
volatile uint32_t dbg_eth_dma_error_cnt = 0U;
volatile uint32_t dbg_eth_last_dma_error = 0U;
volatile uint32_t dbg_eth_recovery_attempt_cnt = 0U;
volatile uint32_t dbg_eth_recovery_success_cnt = 0U;
volatile uint32_t dbg_eth_recovery_fail_cnt = 0U;
volatile uint32_t dbg_eth_last_recovery_stage = 0U;
volatile uint32_t dbg_eth_link_ready = 0U;

#if defined ( __ICCARM__ ) /*!< IAR Compiler */

#pragma location=0x2004c000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x2004c0a0
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x2004c000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x2004c0a0))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */

//ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
//ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT]
  __attribute__((section(".RxDecripSection"))) __attribute__((aligned(32)));

ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT]
  __attribute__((section(".TxDecripSection"))) __attribute__((aligned(32)));

#endif

/* USER CODE BEGIN 2 */
//Replace the above descriptors with this one
//ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT]
//  __attribute__((section(".RxDecripSection"))) __attribute__((aligned(32)));

//ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT]
//  __attribute__((section(".TxDecripSection"))) __attribute__((aligned(32)));

/* MAC phải là static vì HAL giữ con trỏ này sau khi low_level_init() kết thúc */
static uint8_t ETH_MACAddr[6] =
{
    0x00,
    0x80,
    0xE1,
    0x00,
    0x00,
    0x00
};

/* Request từ micro-ROS manager sang Ethernet link thread */
static volatile uint8_t g_eth_force_recovery = 0U;

void ethernet_request_recovery(void)
{
    g_eth_force_recovery = 1U;
}
/* USER CODE END 2 */

osSemaphoreId RxPktSemaphore = NULL;   /* Semaphore to signal incoming packets */
osSemaphoreId TxPktSemaphore = NULL;   /* Semaphore to signal transmit packet complete */

/* Global Ethernet handle */
ETH_HandleTypeDef heth;
ETH_TxPacketConfig TxConfig;

/* Private function prototypes -----------------------------------------------*/
int32_t ETH_PHY_IO_Init(void);
int32_t ETH_PHY_IO_DeInit (void);
int32_t ETH_PHY_IO_ReadReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t *pRegVal);
int32_t ETH_PHY_IO_WriteReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t RegVal);
int32_t ETH_PHY_IO_GetTick(void);

lan8742_Object_t LAN8742;
lan8742_IOCtx_t  LAN8742_IOCtx = {ETH_PHY_IO_Init,
                                  ETH_PHY_IO_DeInit,
                                  ETH_PHY_IO_WriteReg,
                                  ETH_PHY_IO_ReadReg,
                                  ETH_PHY_IO_GetTick};

/* USER CODE BEGIN 3 */

/* USER CODE END 3 */

/* Private functions ---------------------------------------------------------*/
void pbuf_free_custom(struct pbuf *p);

static uint32_t eth_enter_critical(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

static void eth_exit_critical(uint32_t primask)
{
  __set_PRIMASK(primask);
}

static int32_t eth_rx_track_find(const struct pbuf_custom *pbuf)
{
  uint32_t i;

  for (i = 0U; i < ETH_RX_BUFFER_CNT; i++)
  {
    if (g_rx_buffers[i].pbuf == pbuf)
    {
      return (int32_t)i;
    }
  }

  return -1;
}

static bool eth_rx_track_add(struct pbuf_custom *pbuf)
{
  uint32_t i;
  uint32_t primask = eth_enter_critical();

  for (i = 0U; i < ETH_RX_BUFFER_CNT; i++)
  {
    if (g_rx_buffers[i].pbuf == NULL)
    {
      g_rx_buffers[i].pbuf = pbuf;
      g_rx_buffers[i].owner = RX_OWNER_DRIVER;
      dbg_eth_rx_pool_in_use++;
      eth_exit_critical(primask);
      return true;
    }
  }

  eth_exit_critical(primask);
  return false;
}

static bool eth_rx_track_transfer_to_lwip(struct pbuf *p)
{
  bool tracked = true;
  uint32_t primask = eth_enter_critical();

  for (; p != NULL; p = p->next)
  {
    int32_t slot = eth_rx_track_find((const struct pbuf_custom *)p);
    if (slot < 0 || g_rx_buffers[slot].owner != RX_OWNER_DRIVER)
    {
      tracked = false;
      dbg_eth_rx_invalid_free_cnt++;
      continue;
    }

    g_rx_buffers[slot].owner = RX_OWNER_LWIP;
  }

  eth_exit_critical(primask);
  return tracked;
}

static bool eth_rx_is_driver_owned(const struct pbuf *p)
{
  bool driver_owned = false;
  uint32_t primask = eth_enter_critical();
  int32_t slot = eth_rx_track_find((const struct pbuf_custom *)p);

  if (slot >= 0 && g_rx_buffers[slot].owner == RX_OWNER_DRIVER)
  {
    driver_owned = true;
  }
  else
  {
    dbg_eth_rx_invalid_free_cnt++;
  }

  eth_exit_critical(primask);
  return driver_owned;
}

static bool eth_rx_chain_is_driver_owned(const struct pbuf *p)
{
  for (; p != NULL; p = p->next)
  {
    if (!eth_rx_is_driver_owned(p))
    {
      return false;
    }
  }

  return true;
}

static void eth_update_rx_descriptor_count(void)
{
  uint32_t i;
  uint32_t ready = 0U;
  uintptr_t first_buffer = 0U;

  for (i = 0U; i < ETH_RX_DESC_CNT; i++)
  {
    ETH_DMADescTypeDef *desc = (ETH_DMADescTypeDef *)heth.RxDescList.RxDesc[i];
    if (desc != NULL && desc->BackupAddr0 != 0U && desc->DESC2 != 0U)
    {
      ready++;
      if (first_buffer == 0U)
      {
        first_buffer = desc->BackupAddr0;
      }
    }
  }

  dbg_eth_rx_desc_ready = ready;
  addr_rx_buf = first_buffer;
}

static void eth_reclaim_driver_buffers(void)
{
  uint32_t i;

  for (i = 0U; i < ETH_RX_DESC_CNT; i++)
  {
    ETH_DMADescTypeDef *desc = (ETH_DMADescTypeDef *)heth.RxDescList.RxDesc[i];
    if (desc != NULL)
    {
      uint32_t buffer = desc->BackupAddr0;
      desc->BackupAddr0 = 0U;
      desc->BackupAddr1 = 0U;
      desc->DESC2 = 0U;
      if (buffer != 0U)
      {
        struct pbuf *p = (struct pbuf *)(buffer - offsetof(RxBuff_t, buff));
        if (eth_rx_is_driver_owned(p))
        {
          pbuf_free(p);
        }
      }
    }
  }

  struct pbuf *partial_packet = (struct pbuf *)heth.RxDescList.pRxStart;
  heth.RxDescList.pRxStart = NULL;
  heth.RxDescList.pRxEnd = NULL;
  if (partial_packet != NULL && eth_rx_chain_is_driver_owned(partial_packet))
  {
    pbuf_free(partial_packet);
  }

  eth_update_rx_descriptor_count();
}

static bool eth_prepare_rx_descriptors(void)
{
  uint32_t i;

  RxAllocStatus = RX_ALLOC_OK;
  dbg_eth_rx_alloc_status = RX_ALLOC_OK;
  for (i = 0U; i < ETH_RX_DESC_CNT; i++)
  {
    ETH_DMADescTypeDef *desc = (ETH_DMADescTypeDef *)heth.RxDescList.RxDesc[i];
    uint8_t *buffer = NULL;

    if (desc == NULL || desc->BackupAddr0 != 0U || desc->DESC2 != 0U)
    {
      dbg_eth_rx_invalid_free_cnt++;
      eth_reclaim_driver_buffers();
      return false;
    }

    HAL_ETH_RxAllocateCallback(&buffer);
    if (buffer == NULL)
    {
      eth_reclaim_driver_buffers();
      return false;
    }

    desc->BackupAddr0 = (uint32_t)buffer;
    desc->DESC2 = (uint32_t)buffer;
  }

  eth_update_rx_descriptor_count();
  return (dbg_eth_rx_desc_ready == ETH_RX_DESC_CNT &&
          RxAllocStatus == RX_ALLOC_OK);
}

static bool eth_dma_is_stopped(void)
{
  uint32_t dma_status = heth.Instance->DMASR;

  return ((heth.Instance->DMAOMR & (ETH_DMAOMR_ST | ETH_DMAOMR_SR)) == 0U) &&
         ((dma_status & (ETH_DMASR_TPS | ETH_DMASR_RPS)) == 0U);
}

static void eth_release_pending_tx(void)
{
  uint32_t i;

  (void)HAL_ETH_ReleaseTxPacket(&heth);
  for (i = 0U; i < ETH_TX_DESC_CNT; i++)
  {
    uint32_t *packet = heth.TxDescList.PacketAddress[i];
    heth.TxDescList.PacketAddress[i] = NULL;
    if (packet != NULL)
    {
      pbuf_free((struct pbuf *)packet);
    }
  }

  heth.TxDescList.BuffersInUse = 0U;
  heth.TxDescList.releaseIndex = heth.TxDescList.CurTxDesc;
}

static bool eth_stop_and_reclaim_locked(void)
{
  uint32_t start_ms = HAL_GetTick();

  dbg_eth_last_recovery_stage = 1U;
  if (HAL_ETH_Stop_IT(&heth) != HAL_OK &&
      (heth.Instance->DMAOMR & (ETH_DMAOMR_ST | ETH_DMAOMR_SR)) != 0U)
  {
    return false;
  }

  while (!eth_dma_is_stopped())
  {
    if ((HAL_GetTick() - start_ms) >= ETH_DMA_STOP_TIMEOUT_MS)
    {
      return false;
    }
    osDelay(1U);
  }

  HAL_NVIC_DisableIRQ(ETH_IRQn);
  HAL_NVIC_ClearPendingIRQ(ETH_IRQn);
  __HAL_ETH_DMA_DISABLE_IT(&heth, ETH_DMAIER_NISE | ETH_DMAIER_RIE |
                                  ETH_DMAIER_TIE | ETH_DMAIER_FBEIE |
                                  ETH_DMAIER_AISE | ETH_DMAIER_RBUIE);
  eth_release_pending_tx();
  eth_reclaim_driver_buffers();
  RxAllocStatus = RX_ALLOC_OK;
  dbg_eth_rx_alloc_status = RX_ALLOC_OK;
  return true;
}

static bool eth_start_with_link_config(uint32_t speed, uint32_t duplex)
{
  ETH_MACConfigTypeDef mac_config = {0};

  if (g_eth_io_mutex == NULL || RxPktSemaphore == NULL ||
      TxPktSemaphore == NULL || g_eth_input_thread == NULL)
  {
    dbg_eth_last_recovery_stage = 13U;
    return false;
  }

  if (HAL_ETH_GetMACConfig(&heth, &mac_config) != HAL_OK)
  {
    dbg_eth_last_recovery_stage = 2U;
    return false;
  }

  mac_config.DuplexMode = duplex;
  mac_config.Speed = speed;
  if (HAL_ETH_SetMACConfig(&heth, &mac_config) != HAL_OK)
  {
    dbg_eth_last_recovery_stage = 3U;
    return false;
  }

  if (!eth_prepare_rx_descriptors())
  {
    dbg_eth_last_recovery_stage = 4U;
    return false;
  }

  if (HAL_ETH_Start_IT(&heth) != HAL_OK)
  {
    dbg_eth_last_recovery_stage = 5U;
    return false;
  }

  eth_update_rx_descriptor_count();
  if (dbg_eth_rx_desc_ready != ETH_RX_DESC_CNT)
  {
    dbg_eth_last_recovery_stage = 6U;
    return false;
  }

  HAL_NVIC_ClearPendingIRQ(ETH_IRQn);
  HAL_NVIC_EnableIRQ(ETH_IRQn);
  return true;
}

static bool eth_ensure_runtime_resources(struct netif *netif)
{
  osThreadAttr_t attributes = {0};

  if (g_eth_io_mutex == NULL)
  {
    g_eth_io_mutex = osMutexNew(NULL);
  }
  if (RxPktSemaphore == NULL)
  {
    RxPktSemaphore = osSemaphoreNew(1U, 0U, NULL);
  }
  if (TxPktSemaphore == NULL)
  {
    TxPktSemaphore = osSemaphoreNew(1U, 0U, NULL);
  }
  if (g_eth_input_thread == NULL)
  {
    attributes.name = "EthIf";
    attributes.stack_size = INTERFACE_THREAD_STACK_SIZE;
    attributes.priority = osPriorityHigh;
    g_eth_input_thread = osThreadNew(ethernetif_input, netif, &attributes);
  }

  return (g_eth_io_mutex != NULL && RxPktSemaphore != NULL &&
          TxPktSemaphore != NULL && g_eth_input_thread != NULL);
}

static bool eth_recover_link(struct netif *netif, uint32_t speed, uint32_t duplex)
{
  bool success = false;

  dbg_eth_recovery_attempt_cnt++;
  dbg_eth_last_recovery_stage = 0U;
  dbg_eth_link_ready = 0U;
  netif_set_down(netif);
  netif_set_link_down(netif);

  if (g_eth_io_mutex == NULL ||
      osMutexAcquire(g_eth_io_mutex, ETHIF_TX_TIMEOUT) != osOK)
  {
    dbg_eth_last_recovery_stage = 7U;
    goto recovery_done;
  }

  if (!eth_stop_and_reclaim_locked())
  {
    goto release_mutex;
  }

  dbg_eth_last_recovery_stage = 8U;
  if (HAL_ETH_DeInit(&heth) != HAL_OK)
  {
    dbg_eth_last_recovery_stage = 9U;
    goto release_mutex;
  }

  dbg_eth_last_recovery_stage = 10U;
  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    dbg_eth_last_recovery_stage = 11U;
    goto release_mutex;
  }

  if (!eth_start_with_link_config(speed, duplex))
  {
    if ((heth.Instance->DMAOMR & (ETH_DMAOMR_ST | ETH_DMAOMR_SR)) != 0U)
    {
      (void)eth_stop_and_reclaim_locked();
    }
    else
    {
      eth_reclaim_driver_buffers();
    }
    goto release_mutex;
  }

  success = true;

release_mutex:
  osMutexRelease(g_eth_io_mutex);

recovery_done:
  if (success)
  {
    netif_set_up(netif);
    netif_set_link_up(netif);
    dbg_eth_link_ready = 1U;
    dbg_eth_recovery_success_cnt++;
    dbg_eth_last_recovery_stage = 0U;
  }
  else
  {
    netif_set_down(netif);
    netif_set_link_down(netif);
    dbg_eth_recovery_fail_cnt++;
  }

  return success;
}

/**
  * @brief  Ethernet Rx Transfer completed callback
  * @param  handlerEth: ETH handler
  * @retval None
  */
void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *handlerEth)
{
  if (RxPktSemaphore != NULL)
  {
    osSemaphoreRelease(RxPktSemaphore);
  }
}
/**
  * @brief  Ethernet Tx Transfer completed callback
  * @param  handlerEth: ETH handler
  * @retval None
  */
void HAL_ETH_TxCpltCallback(ETH_HandleTypeDef *handlerEth)
{
  if (TxPktSemaphore != NULL)
  {
    osSemaphoreRelease(TxPktSemaphore);
  }
}
/**
  * @brief  Ethernet DMA transfer error callback
  * @param  handlerEth: ETH handler
  * @retval None
  */
void HAL_ETH_ErrorCallback(ETH_HandleTypeDef *handlerEth)
{
  uint32_t dma_error = HAL_ETH_GetDMAError(handlerEth);

  dbg_eth_dma_error_cnt++;
  dbg_eth_last_dma_error = dma_error;

  if((dma_error & ETH_DMASR_RBUS) == ETH_DMASR_RBUS)
  {
     osSemaphoreRelease(RxPktSemaphore);
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/*******************************************************************************
                       LL Driver Interface ( LwIP stack --> ETH)
*******************************************************************************/
/**
 * @brief In this function, the hardware should be initialized.
 * Called from ethernetif_init().
 *
 * @param netif the already initialized lwip network interface structure
 *        for this ethernetif
 */
static void low_level_init(struct netif *netif)
{
  addr_rx_desc = (uintptr_t)DMARxDscrTab;
  addr_tx_desc = (uintptr_t)DMATxDscrTab;

  g_eth_io_mutex = osMutexNew(NULL);

  HAL_StatusTypeDef hal_eth_init_status = HAL_OK;
  uint32_t duplex, speed = 0;
  int32_t PHYLinkState = 0;
  /* Start ETH HAL Init */

  heth.Instance = ETH;
  heth.Init.MACAddr = ETH_MACAddr;
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1536;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  hal_eth_init_status = HAL_ETH_Init(&heth);

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;

  /* End ETH HAL Init */

  /* Initialize the RX POOL */
  LWIP_MEMPOOL_INIT(RX_POOL);

#if LWIP_ARP || LWIP_ETHERNET

  /* set MAC hardware address length */
  netif->hwaddr_len = ETH_HWADDR_LEN;

  /* set MAC hardware address */
  netif->hwaddr[0] =  heth.Init.MACAddr[0];
  netif->hwaddr[1] =  heth.Init.MACAddr[1];
  netif->hwaddr[2] =  heth.Init.MACAddr[2];
  netif->hwaddr[3] =  heth.Init.MACAddr[3];
  netif->hwaddr[4] =  heth.Init.MACAddr[4];
  netif->hwaddr[5] =  heth.Init.MACAddr[5];

  /* maximum transfer unit */
  netif->mtu = ETH_MAX_PAYLOAD;

  /* Accept broadcast address and ARP traffic */
  /* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
  #if LWIP_ARP
    netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;
  #else
    netif->flags |= NETIF_FLAG_BROADCAST;
  #endif /* LWIP_ARP */

  if (!eth_ensure_runtime_resources(netif))
  {
    dbg_eth_last_recovery_stage = 12U;
    netif_set_down(netif);
    netif_set_link_down(netif);
  }

/* USER CODE BEGIN PHY_PRE_CONFIG */

/* USER CODE END PHY_PRE_CONFIG */
  /* Set PHY IO functions */
  LAN8742_RegisterBusIO(&LAN8742, &LAN8742_IOCtx);

  /* Initialize the LAN8742 ETH PHY */
  if(LAN8742_Init(&LAN8742) != LAN8742_STATUS_OK)
  {
    netif_set_link_down(netif);
    netif_set_down(netif);
    return;
  }

  if (hal_eth_init_status == HAL_OK)
  {
    PHYLinkState = LAN8742_GetLinkState(&LAN8742);

    /* Get link state */
    if(PHYLinkState <= LAN8742_STATUS_LINK_DOWN)
    {
      netif_set_link_down(netif);
      netif_set_down(netif);
    }
    else
    {
      switch (PHYLinkState)
      {
      case LAN8742_STATUS_100MBITS_FULLDUPLEX:
        duplex = ETH_FULLDUPLEX_MODE;
        speed = ETH_SPEED_100M;
        break;
      case LAN8742_STATUS_100MBITS_HALFDUPLEX:
        duplex = ETH_HALFDUPLEX_MODE;
        speed = ETH_SPEED_100M;
        break;
      case LAN8742_STATUS_10MBITS_FULLDUPLEX:
        duplex = ETH_FULLDUPLEX_MODE;
        speed = ETH_SPEED_10M;
        break;
      case LAN8742_STATUS_10MBITS_HALFDUPLEX:
        duplex = ETH_HALFDUPLEX_MODE;
        speed = ETH_SPEED_10M;
        break;
      default:
        duplex = ETH_FULLDUPLEX_MODE;
        speed = ETH_SPEED_100M;
        break;
      }

    if (g_eth_io_mutex != NULL &&
        osMutexAcquire(g_eth_io_mutex, ETHIF_TX_TIMEOUT) == osOK)
    {
      if (eth_start_with_link_config(speed, duplex))
      {
        netif_set_up(netif);
        netif_set_link_up(netif);
        dbg_eth_link_ready = 1U;
      }
      else
      {
        if ((heth.Instance->DMAOMR & (ETH_DMAOMR_ST | ETH_DMAOMR_SR)) != 0U)
        {
          (void)eth_stop_and_reclaim_locked();
        }
        else
        {
          eth_reclaim_driver_buffers();
        }
        dbg_eth_link_ready = 0U;
      }
      osMutexRelease(g_eth_io_mutex);
    }

/* USER CODE BEGIN PHY_POST_CONFIG */

/* USER CODE END PHY_POST_CONFIG */
    }

  }
  else
  {
    dbg_eth_link_ready = 0U;
    netif_set_link_down(netif);
    netif_set_down(netif);
  }
#endif /* LWIP_ARP || LWIP_ETHERNET */

/* USER CODE BEGIN LOW_LEVEL_INIT */

/* USER CODE END LOW_LEVEL_INIT */

}

/**
 * @brief This function should do the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
 * @return ERR_OK if the packet could be sent
 *         an err_t value if the packet couldn't be sent
 *
 * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
 *       strange results. You might consider waiting for space in the DMA queue
 *       to become available since the stack doesn't retry to send a packet
 *       dropped because of memory failure (except for the TCP timers).
 */

static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
  uint32_t i = 0U;
  uint32_t tx_start_ms;
  struct pbuf *q = NULL;
  err_t errval = ERR_OK;
  ETH_BufferTypeDef Txbuffer[ETH_TX_DESC_CNT] = {0};

  memset(Txbuffer, 0 , ETH_TX_DESC_CNT*sizeof(ETH_BufferTypeDef));

  for(q = p; q != NULL; q = q->next)
  {
    if(i >= ETH_TX_DESC_CNT)
      return ERR_IF;

    Txbuffer[i].buffer = q->payload;
    Txbuffer[i].len = q->len;

    if(i>0)
    {
      Txbuffer[i-1].next = &Txbuffer[i];
    }

    if(q->next == NULL)
    {
      Txbuffer[i].next = NULL;
    }

    i++;
  }

  if (g_eth_io_mutex == NULL ||
      osMutexAcquire(g_eth_io_mutex, ETHIF_TX_TIMEOUT) != osOK)
  {
    return ERR_IF;
  }
  if (dbg_eth_link_ready == 0U || heth.gState != HAL_ETH_STATE_STARTED)
  {
    osMutexRelease(g_eth_io_mutex);
    return ERR_IF;
  }

  TxConfig.Length = p->tot_len;
  TxConfig.TxBuffer = Txbuffer;
  TxConfig.pData = p;
  tx_start_ms = HAL_GetTick();

  pbuf_ref(p);

  do
  {
    if(HAL_ETH_Transmit_IT(&heth, &TxConfig) == HAL_OK)
    {
      errval = ERR_OK;
    }
    else
    {

      if(HAL_ETH_GetError(&heth) & HAL_ETH_ERROR_BUSY)
      {
        /* Wait for descriptors to become available */
        osStatus_t tx_wait_status =
          osSemaphoreAcquire(TxPktSemaphore, ETHIF_TX_TIMEOUT);
        HAL_ETH_ReleaseTxPacket(&heth);
        if (tx_wait_status != osOK &&
            (HAL_GetTick() - tx_start_ms) >= ETHIF_TX_TIMEOUT)
        {
          pbuf_free(p);
          errval = ERR_IF;
        }
        else
        {
          errval = ERR_BUF;
        }
      }
      else
      {
        /* Other error */
        pbuf_free(p);
        errval =  ERR_IF;
      }
    }
  }while(errval == ERR_BUF);

  osMutexRelease(g_eth_io_mutex);
  return errval;
}

/**
 * @brief Should allocate a pbuf and transfer the bytes of the incoming
 * packet from the interface into the pbuf.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return a pbuf filled with the received packet (including MAC header)
 *         NULL on memory error
   */
static struct pbuf * low_level_input(struct netif *netif)
{
  struct pbuf *p = NULL;

  if (g_eth_io_mutex != NULL &&
      osMutexAcquire(g_eth_io_mutex, ETHIF_TX_TIMEOUT) == osOK)
  {
    if (RxAllocStatus == RX_ALLOC_OK)
    {
      (void)HAL_ETH_ReadData(&heth, (void **)&p);
      eth_update_rx_descriptor_count();
      if (p != NULL)
      {
        (void)eth_rx_track_transfer_to_lwip(p);
      }
    }
    osMutexRelease(g_eth_io_mutex);
  }

  return p;
}

/**
 * @brief This function should be called when a packet is ready to be read
 * from the interface. It uses the function low_level_input() that
 * should handle the actual reception of bytes from the network
 * interface. Then the type of the received packet is determined and
 * the appropriate input function is called.
 *
 * @param netif the lwip network interface structure for this ethernetif
 */
void ethernetif_input(void* argument)
{
  struct pbuf *p = NULL;
  struct netif *netif = (struct netif *) argument;

  for( ;; )
  {
    if (osSemaphoreAcquire(RxPktSemaphore, TIME_WAITING_FOR_INPUT) == osOK)
    {
      do
      {
        p = low_level_input( netif );
        if (p != NULL)
        {
          if (netif->input( p, netif) != ERR_OK )
          {
            pbuf_free(p);
          }
        }
      } while(p!=NULL);
    }
  }
}

#if !LWIP_ARP
/**
 * This function has to be completed by user in case of ARP OFF.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if ...
 */
static err_t low_level_output_arp_off(struct netif *netif, struct pbuf *q, const ip4_addr_t *ipaddr)
{
  err_t errval;
  errval = ERR_OK;

/* USER CODE BEGIN 5 */

/* USER CODE END 5 */

  return errval;

}
#endif /* LWIP_ARP */

/**
 * @brief Should be called at the beginning of the program to set up the
 * network interface. It calls the function low_level_init() to do the
 * actual setup of the hardware.
 *
 * This function should be passed as a parameter to netif_add().
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if the loopif is initialized
 *         ERR_MEM if private data couldn't be allocated
 *         any other err_t on error
 */
err_t ethernetif_init(struct netif *netif)
{
  LWIP_ASSERT("netif != NULL", (netif != NULL));

#if LWIP_NETIF_HOSTNAME
  /* Initialize interface hostname */
  netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */

  /*
   * Initialize the snmp variables and counters inside the struct netif.
   * The last argument should be replaced with your link speed, in units
   * of bits per second.
   */
  // MIB2_INIT_NETIF(netif, snmp_ifType_ethernet_csmacd, LINK_SPEED_OF_YOUR_NETIF_IN_BPS);

  netif->name[0] = IFNAME0;
  netif->name[1] = IFNAME1;
  /* We directly use etharp_output() here to save a function call.
   * You can instead declare your own function an call etharp_output()
   * from it if you have to do some checks before sending (e.g. if link
   * is available...) */

#if LWIP_IPV4
#if LWIP_ARP || LWIP_ETHERNET
#if LWIP_ARP
  netif->output = etharp_output;
#else
  /* The user should write its own code in low_level_output_arp_off function */
  netif->output = low_level_output_arp_off;
#endif /* LWIP_ARP */
#endif /* LWIP_ARP || LWIP_ETHERNET */
#endif /* LWIP_IPV4 */

#if LWIP_IPV6
  netif->output_ip6 = ethip6_output;
#endif /* LWIP_IPV6 */

  netif->linkoutput = low_level_output;

  /* initialize the hardware */
  low_level_init(netif);

  return ERR_OK;
}

/**
  * @brief  Custom Rx pbuf free callback
  * @param  pbuf: pbuf to be freed
  * @retval None
  */
void pbuf_free_custom(struct pbuf *p)
{
  struct pbuf_custom* custom_pbuf = (struct pbuf_custom*)p;
  uint32_t primask = eth_enter_critical();
  int32_t slot = eth_rx_track_find(custom_pbuf);

  if (slot < 0)
  {
    dbg_eth_rx_invalid_free_cnt++;
    eth_exit_critical(primask);
    return;
  }

  g_rx_buffers[slot].pbuf = NULL;
  g_rx_buffers[slot].owner = 0U;
  if (dbg_eth_rx_pool_in_use > 0U)
  {
    dbg_eth_rx_pool_in_use--;
  }
  else
  {
    dbg_eth_rx_invalid_free_cnt++;
  }
  eth_exit_critical(primask);

  LWIP_MEMPOOL_FREE(RX_POOL, custom_pbuf);

  /* If the Rx Buffer Pool was exhausted, signal the ethernetif_input task to
   * call HAL_ETH_GetRxDataBuffer to rebuild the Rx descriptors. */

  if (RxAllocStatus == RX_ALLOC_ERROR)
  {
    RxAllocStatus = RX_ALLOC_OK;
    dbg_eth_rx_alloc_status = RX_ALLOC_OK;
    osSemaphoreRelease(RxPktSemaphore);
  }
}

/* USER CODE BEGIN 6 */

/**
* @brief  Returns the current time in milliseconds
*         when LWIP_TIMERS == 1 and NO_SYS == 1
* @param  None
* @retval Current Time value
*/
u32_t sys_now(void)
{
  return HAL_GetTick();
}

/* USER CODE END 6 */

/**
  * @brief  Initializes the ETH MSP.
  * @param  ethHandle: ETH handle
  * @retval None
  */

void HAL_ETH_MspInit(ETH_HandleTypeDef* ethHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(ethHandle->Instance==ETH)
  {
  /* USER CODE BEGIN ETH_MspInit 0 */

  /* USER CODE END ETH_MspInit 0 */
    /* Enable Peripheral clock */
    __HAL_RCC_ETH_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    /**ETH GPIO Configuration
    PC1     ------> ETH_MDC
    PA1     ------> ETH_REF_CLK
    PA2     ------> ETH_MDIO
    PA7     ------> ETH_CRS_DV
    PC4     ------> ETH_RXD0
    PC5     ------> ETH_RXD1
    PB13     ------> ETH_TXD1
    PG11     ------> ETH_TX_EN
    PG13     ------> ETH_TXD0
    */
    GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|RMII_CRS_DV_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = RMII_TXD1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = RMII_TX_EN_Pin|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(ETH_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(ETH_IRQn);
  /* USER CODE BEGIN ETH_MspInit 1 */

  /* USER CODE END ETH_MspInit 1 */
  }
}

void HAL_ETH_MspDeInit(ETH_HandleTypeDef* ethHandle)
{
  if(ethHandle->Instance==ETH)
  {
  /* USER CODE BEGIN ETH_MspDeInit 0 */

  /* USER CODE END ETH_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ETH_CLK_DISABLE();

    /**ETH GPIO Configuration
    PC1     ------> ETH_MDC
    PA1     ------> ETH_REF_CLK
    PA2     ------> ETH_MDIO
    PA7     ------> ETH_CRS_DV
    PC4     ------> ETH_RXD0
    PC5     ------> ETH_RXD1
    PB13     ------> ETH_TXD1
    PG11     ------> ETH_TX_EN
    PG13     ------> ETH_TXD0
    */
    HAL_GPIO_DeInit(GPIOC, RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1|GPIO_PIN_2|RMII_CRS_DV_Pin);

    HAL_GPIO_DeInit(RMII_TXD1_GPIO_Port, RMII_TXD1_Pin);

    HAL_GPIO_DeInit(GPIOG, RMII_TX_EN_Pin|GPIO_PIN_13);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(ETH_IRQn);

  /* USER CODE BEGIN ETH_MspDeInit 1 */

  /* USER CODE END ETH_MspDeInit 1 */
  }
}

/*******************************************************************************
                       PHI IO Functions
*******************************************************************************/
/**
  * @brief  Initializes the MDIO interface GPIO and clocks.
  * @param  None
  * @retval 0 if OK, -1 if ERROR
  */
int32_t ETH_PHY_IO_Init(void)
{
  /* We assume that MDIO GPIO configuration is already done
     in the ETH_MspInit() else it should be done here
  */

  /* Configure the MDIO Clock */
  HAL_ETH_SetMDIOClockRange(&heth);

  return 0;
}

/**
  * @brief  De-Initializes the MDIO interface .
  * @param  None
  * @retval 0 if OK, -1 if ERROR
  */
int32_t ETH_PHY_IO_DeInit (void)
{
  return 0;
}

/**
  * @brief  Read a PHY register through the MDIO interface.
  * @param  DevAddr: PHY port address
  * @param  RegAddr: PHY register address
  * @param  pRegVal: pointer to hold the register value
  * @retval 0 if OK -1 if Error
  */
int32_t ETH_PHY_IO_ReadReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t *pRegVal)
{
  if(HAL_ETH_ReadPHYRegister(&heth, DevAddr, RegAddr, pRegVal) != HAL_OK)
  {
    return -1;
  }

  return 0;
}

/**
  * @brief  Write a value to a PHY register through the MDIO interface.
  * @param  DevAddr: PHY port address
  * @param  RegAddr: PHY register address
  * @param  RegVal: Value to be written
  * @retval 0 if OK -1 if Error
  */
int32_t ETH_PHY_IO_WriteReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t RegVal)
{
  if(HAL_ETH_WritePHYRegister(&heth, DevAddr, RegAddr, RegVal) != HAL_OK)
  {
    return -1;
  }

  return 0;
}

/**
  * @brief  Get the time in millisecons used for internal PHY driver process.
  * @retval Time value
  */
int32_t ETH_PHY_IO_GetTick(void)
{
  return HAL_GetTick();
}

/**
  * @brief  Check the ETH link state then update ETH driver and netif link accordingly.
  * @retval None
  */
static bool eth_get_link_config(int32_t phy_state, uint32_t *speed, uint32_t *duplex)
{
  switch (phy_state)
  {
    case LAN8742_STATUS_100MBITS_FULLDUPLEX:
      *speed = ETH_SPEED_100M;
      *duplex = ETH_FULLDUPLEX_MODE;
      return true;
    case LAN8742_STATUS_100MBITS_HALFDUPLEX:
      *speed = ETH_SPEED_100M;
      *duplex = ETH_HALFDUPLEX_MODE;
      return true;
    case LAN8742_STATUS_10MBITS_FULLDUPLEX:
      *speed = ETH_SPEED_10M;
      *duplex = ETH_FULLDUPLEX_MODE;
      return true;
    case LAN8742_STATUS_10MBITS_HALFDUPLEX:
      *speed = ETH_SPEED_10M;
      *duplex = ETH_HALFDUPLEX_MODE;
      return true;
    default:
      return false;
  }
}

void ethernet_link_thread(void* argument)
{
  struct netif *netif = (struct netif *)argument;
  uint32_t speed = ETH_SPEED_100M;
  uint32_t duplex = ETH_FULLDUPLEX_MODE;
  uint32_t last_recovery_ms = HAL_GetTick() - ETH_RECOVERY_RETRY_MS;
  uint8_t link_up_stable_count = 0U;
  bool recovery_pending = false;

  for (;;)
  {
    if (!eth_ensure_runtime_resources(netif))
    {
      dbg_eth_last_recovery_stage = 12U;
      osDelay(100U);
      continue;
    }

    int32_t phy_state = LAN8742_GetLinkState(&LAN8742);
    bool phy_up = phy_state > LAN8742_STATUS_LINK_DOWN;
    bool link_up = netif_is_link_up(netif);

    if (g_eth_force_recovery != 0U)
    {
      g_eth_force_recovery = 0U;
      recovery_pending = true;
    }

    if (!phy_up)
    {
      link_up_stable_count = 0U;
      if (link_up)
      {
        netif_set_down(netif);
        netif_set_link_down(netif);
        dbg_eth_link_ready = 0U;

        if (g_eth_io_mutex != NULL &&
            osMutexAcquire(g_eth_io_mutex, ETHIF_TX_TIMEOUT) == osOK)
        {
          if (!eth_stop_and_reclaim_locked())
          {
            dbg_eth_recovery_fail_cnt++;
          }
          osMutexRelease(g_eth_io_mutex);
        }
        else
        {
          dbg_eth_recovery_fail_cnt++;
          dbg_eth_last_recovery_stage = 7U;
        }
      }
    }
    else
    {
      if (!link_up)
      {
        if (link_up_stable_count < 5U)
        {
          link_up_stable_count++;
        }
        if (link_up_stable_count >= 5U)
        {
          recovery_pending = true;
        }
      }
      else if (!recovery_pending)
      {
        link_up_stable_count = 0U;
      }

      if (recovery_pending &&
          (link_up || link_up_stable_count >= 5U) &&
          (HAL_GetTick() - last_recovery_ms) >= ETH_RECOVERY_RETRY_MS)
      {
        last_recovery_ms = HAL_GetTick();
        if (eth_get_link_config(phy_state, &speed, &duplex) &&
            eth_recover_link(netif, speed, duplex))
        {
          recovery_pending = false;
          link_up_stable_count = 0U;
        }
        else
        {
          link_up_stable_count = 0U;
        }
      }
    }

    osDelay(100U);
  }
}

void HAL_ETH_RxAllocateCallback(uint8_t **buff)
{
/* USER CODE BEGIN HAL ETH RxAllocateCallback */
  struct pbuf_custom *p = LWIP_MEMPOOL_ALLOC(RX_POOL);
  if (p)
  {
    /* Get the buff from the struct pbuf address. */
    *buff = (uint8_t *)p + offsetof(RxBuff_t, buff);
    p->custom_free_function = pbuf_free_custom;
    /* Initialize the struct pbuf.
    * This must be performed whenever a buffer's allocated because it may be
    * changed by lwIP or the app, e.g., pbuf_free decrements ref. */
    pbuf_alloced_custom(PBUF_RAW, 0, PBUF_REF, p, *buff, ETH_RX_BUF_SIZE);
    if (!eth_rx_track_add(p))
    {
      dbg_eth_rx_alloc_fail_cnt++;
      RxAllocStatus = RX_ALLOC_ERROR;
      dbg_eth_rx_alloc_status = RX_ALLOC_ERROR;
      *buff = NULL;
      LWIP_MEMPOOL_FREE(RX_POOL, p);
    }
  }
  else
  {
    dbg_eth_rx_alloc_fail_cnt++;
    RxAllocStatus = RX_ALLOC_ERROR;
    dbg_eth_rx_alloc_status = RX_ALLOC_ERROR;
    *buff = NULL;
  }
/* USER CODE END HAL ETH RxAllocateCallback */
}

void HAL_ETH_RxLinkCallback(void **pStart, void **pEnd, uint8_t *buff, uint16_t Length)
{
/* USER CODE BEGIN HAL ETH RxLinkCallback */

  struct pbuf **ppStart = (struct pbuf **)pStart;
  struct pbuf **ppEnd = (struct pbuf **)pEnd;
  struct pbuf *p = NULL;

  /* Get the struct pbuf from the buff address. */
  p = (struct pbuf *)(buff - offsetof(RxBuff_t, buff));
  p->next = NULL;
  p->tot_len = 0;
  p->len = Length;

  /* Chain the buffer. */
  if (!*ppStart)
  {
    /* The first buffer of the packet. */
    *ppStart = p;
  }
  else
  {
    /* Chain the buffer to the end of the packet. */
    (*ppEnd)->next = p;
  }
  *ppEnd  = p;

  /* Update the total length of all the buffers of the chain. Each pbuf in the chain should have its tot_len
   * set to its own length, plus the length of all the following pbufs in the chain. */
  for (p = *ppStart; p != NULL; p = p->next)
  {
    p->tot_len += Length;
  }

  /* Invalidate data cache because Rx DMA's writing to physical memory makes it stale. */
  SCB_InvalidateDCache_by_Addr((uint32_t *)buff, Length);

/* USER CODE END HAL ETH RxLinkCallback */
}

void HAL_ETH_TxFreeCallback(uint32_t * buff)
{
/* USER CODE BEGIN HAL ETH TxFreeCallback */

  pbuf_free((struct pbuf *)buff);

/* USER CODE END HAL ETH TxFreeCallback */
}

/* USER CODE BEGIN 8 */

/* USER CODE END 8 */
