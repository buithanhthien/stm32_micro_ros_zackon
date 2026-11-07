# Ethernet and micro-ROS recovery

## Source and incident evidence

The firmware build source list in `tools/sources.mk` compiles
`Core/Src/ethernetif.c`. `LWIP/Target/ethernetif.c` is not the active source.

The earlier on-target observations were an exhausted RX pool
(`RxAllocStatus=RX_ALLOC_ERROR`), four RX descriptors with null buffer
addresses, and a DMA fatal bus error. Those observations establish the failed
runtime state, but do not establish that the currently flashed firmware matches
the source or any ELF in this workspace.

At the time this change was made, `/odomfromSTM32` returned an odometry sample.
`ros2 topic info --verbose` showed the publisher `stm32_odom_node`, which
confirms an active micro-ROS session through the running Agent. No firmware was
read from or written to the MCU.

## Recovery ownership rules

- `ethernet_link_thread` serializes recovery with the RX and TX paths using
  `g_eth_io_mutex`. Recovery first marks the netif down, stops DMA, waits for the
  DMA process state to stop, disables/clears the ETH IRQ, then handles pending
  buffers.
- RX buffers still in descriptors or an incomplete HAL packet belong to the
  driver and are reclaimed. Completed pbufs have their ownership transferred
  to lwIP before the input path releases the mutex; recovery never reclaims
  those buffers.
- Outstanding TX pbuf references are released after DMA is stopped, including
  packets still recorded in HAL's TX descriptor list.
- `RX_POOL` is initialized only once. Recovery reuses it and requires every RX
  descriptor buffer to be allocated before `HAL_ETH_Start_IT` can be called.
- A failed recovery leaves the netif/link down and retries no sooner than once
  per second. It does not report ready based only on `HAL_ETH_Start_IT` status.

## Read-only diagnostics

The following volatile symbols can be inspected through a debugger/SWD session
without resetting the MCU:

| Symbol | Meaning |
| --- | --- |
| `dbg_eth_link_ready` | 1 only after all RX descriptors passed validation and the link was raised |
| `dbg_eth_rx_desc_ready` | RX descriptors with non-null buffer addresses |
| `dbg_eth_rx_pool_in_use` | Tracked RX pool objects still allocated |
| `dbg_eth_rx_alloc_status` | 0: allocations okay; 1: last RX allocation failed |
| `dbg_eth_rx_alloc_fail_cnt` | RX allocation failures |
| `dbg_eth_rx_invalid_free_cnt` | RX ownership/tracking violations; expected to remain 0 |
| `dbg_eth_dma_error_cnt`, `dbg_eth_last_dma_error` | DMA error callback count and last status bits |
| `dbg_eth_recovery_attempt_cnt` | Link reinitialization attempts |
| `dbg_eth_recovery_success_cnt`, `dbg_eth_recovery_fail_cnt` | Successful/failed reinitializations |
| `dbg_eth_last_recovery_stage` | 0: idle/success; 1: stopping; 2-6: MAC/RX/start checks; 7: mutex acquisition; 8-11: HAL deinit/init; 12: runtime resources; 13: start preconditions |
| `dbg_eth_recovery_cnt` | Existing micro-ROS manager count of recovery requests after Agent ping failures |
| `g_uros_connected` | Existing micro-ROS entity/session state, set by the manager task |

No periodic diagnostic logging was added, so recovery telemetry does not add
load to the control tasks.

## Verification and hardware checklist

The candidate was built with `make -C Debug -j4 main-build` using the installed
STM32 GNU 14.3 toolchain. The build succeeded; the changed Ethernet translation
unit emitted no warnings. The output measured 256044 bytes text, 9484 data, and
240476 BSS. ELF and BIN checksums are recorded in the handoff.

Build outputs:

- ELF: [`MotorControlwithMicroROS-ethernet-recovery-20261006.elf`](../build-linux/candidates/MotorControlwithMicroROS-ethernet-recovery-20261006.elf),
  SHA-256 `038a8f103e7be1515e269f20bb0482875c21c8dd46addbb651c99d7f9d486972`
- BIN: [`MotorControlwithMicroROS-ethernet-recovery-20261006.bin`](../build-linux/candidates/MotorControlwithMicroROS-ethernet-recovery-20261006.bin),
  SHA-256 `8e0e256e83fbf9504735b8bff501ef9bc55690a88723892ef634e22efe8d369c`

The candidate includes the pre-existing, uncommitted changes in `Core/Src/motor.c`,
`Core/Inc/motor.h`, and `Core/Src/freertos.c` as well as this Ethernet change.
Those files were not modified as part of this task. Hardware recovery testing
is deliberately not performed. At the final read-only check,
`/odomfromSTM32` still had an active `stm32_odom_node` publisher and the Agent
process was running. No reset or flash operation was performed.

After explicit authorization and after confirming the image on the MCU, test:

1. Repeat forced recovery several times; verify `dbg_eth_rx_pool_in_use` returns
   to its pre-test baseline, `dbg_eth_rx_invalid_free_cnt` stays zero, and
   `dbg_eth_rx_desc_ready == ETH_RX_DESC_CNT` whenever `dbg_eth_link_ready` is 1.
2. Restart the Agent, then unplug/reconnect Ethernet; verify odometry returns
   without resetting the MCU and that the Agent/session state follows it.
3. Request recovery during RX and TX traffic; verify DMA is stopped before any
   driver-owned pbuf is reclaimed, and no descriptor is started with a null
   buffer.
4. Observe odometry and motor control throughout; do not change the topic,
   address, command timeout, or control behavior for these tests.

Before flashing any candidate, read and archive the MCU image and checksum if
the attached programmer supports it. The workspace ELFs/BINs are candidates,
not evidence of the currently flashed image. Keep the verified pre-change image
as the rollback image; do not use an unverified workspace BIN as a rollback.

`build-linux/backups/stm32-20261002-112154.bin` is a historical rollback
candidate that `build-linux/restore-test.log` records as successfully
programmed and verified (SHA-256:
`cf8d483abd5a11ea463915d6263fcc782fcde8f4b33e53e3188040a145b05706`). It is
not evidence of the MCU's current image.
