/*
 * QEMU Educational GPGPU Device
 *
 * Copyright (c) 2024-2025
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/pci/pci.h"
#include "hw/pci/msi.h"
#include "hw/pci/msix.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "exec/cpu-common.h"

#include "gpgpu.h"
#include "gpgpu_core.h"

static uint64_t gpgpu_ctrl_read(void *opaque, hwaddr addr, unsigned size)
{
    GPGPUState *s = GPGPU(opaque);
    uint64_t ret = 0;

    switch (addr) {
    /* 设备信息寄存器组 */
    case GPGPU_REG_DEV_ID:
        ret = GPGPU_DEV_ID_VALUE;
        break;
    case GPGPU_REG_DEV_VERSION:
        ret = GPGPU_DEV_VERSION_VALUE;
        break;
    case GPGPU_REG_DEV_CAPS:
        /* §5.1: [23:16]=WARP_SIZE, [15:8]=WARPS_PER_CU, [7:0]=NUM_CUS */
        ret = (s->warp_size << 16) | (s->warps_per_cu << 8) | s->num_cus;
        break;
    case GPGPU_REG_VRAM_SIZE_LO:
        ret = (uint32_t)(s->vram_size & 0xFFFFFFFF);
        break;
    case GPGPU_REG_VRAM_SIZE_HI:
        ret = (uint32_t)(s->vram_size >> 32);
        break;

    /* 全局控制寄存器组 */
    case GPGPU_REG_GLOBAL_CTRL:
        ret = s->global_ctrl;
        break;
    case GPGPU_REG_GLOBAL_STATUS:
        ret = s->global_status;
        break;
    case GPGPU_REG_ERROR_STATUS:
        ret = s->error_status;
        break;

    /* 中断控制寄存器组 */
    case GPGPU_REG_IRQ_ENABLE:
        ret = s->irq_enable;
        break;
    case GPGPU_REG_IRQ_STATUS:
        ret = s->irq_status;
        break;
    case GPGPU_REG_IRQ_ACK:
        ret = 0; /* 写1清除寄存器，读返回0 */
        break;

    /* 内核分发寄存器组 */
    case GPGPU_REG_KERNEL_ADDR_LO:
        ret = (uint32_t)(s->kernel.kernel_addr & 0xFFFFFFFF);
        break;
    case GPGPU_REG_KERNEL_ADDR_HI:
        ret = (uint32_t)(s->kernel.kernel_addr >> 32);
        break;
    case GPGPU_REG_KERNEL_ARGS_LO:
        ret = (uint32_t)(s->kernel.kernel_args & 0xFFFFFFFF);
        break;
    case GPGPU_REG_KERNEL_ARGS_HI:
        ret = (uint32_t)(s->kernel.kernel_args >> 32);
        break;
    case GPGPU_REG_GRID_DIM_X:
        ret = s->kernel.grid_dim[0];
        break;
    case GPGPU_REG_GRID_DIM_Y:
        ret = s->kernel.grid_dim[1];
        break;
    case GPGPU_REG_GRID_DIM_Z:
        ret = s->kernel.grid_dim[2];
        break;
    case GPGPU_REG_BLOCK_DIM_X:
        ret = s->kernel.block_dim[0];
        break;
    case GPGPU_REG_BLOCK_DIM_Y:
        ret = s->kernel.block_dim[1];
        break;
    case GPGPU_REG_BLOCK_DIM_Z:
        ret = s->kernel.block_dim[2];
        break;
    case GPGPU_REG_SHARED_MEM_SIZE:
        ret = s->kernel.shared_mem_size;
        break;
    case GPGPU_REG_DISPATCH:
        ret = 0;
        break;

    /* DMA 引擎寄存器组 */
    case GPGPU_REG_DMA_SRC_LO:
        ret = (uint32_t)(s->dma.src_addr & 0xFFFFFFFF);
        break;
    case GPGPU_REG_DMA_SRC_HI:
        ret = (uint32_t)(s->dma.src_addr >> 32);
        break;
    case GPGPU_REG_DMA_DST_LO:
        ret = (uint32_t)(s->dma.dst_addr & 0xFFFFFFFF);
        break;
    case GPGPU_REG_DMA_DST_HI:
        ret = (uint32_t)(s->dma.dst_addr >> 32);
        break;
    case GPGPU_REG_DMA_SIZE:
        ret = s->dma.size;
        break;
    case GPGPU_REG_DMA_CTRL:
        ret = s->dma.ctrl;
        break;
    case GPGPU_REG_DMA_STATUS:
        ret = s->dma.status;
        break;

    /* SIMT 线程上下文寄存器组 */
    case GPGPU_REG_THREAD_ID_X:
        ret = s->simt.thread_id[0];
        break;
    case GPGPU_REG_THREAD_ID_Y:
        ret = s->simt.thread_id[1];
        break;
    case GPGPU_REG_THREAD_ID_Z:
        ret = s->simt.thread_id[2];
        break;
    case GPGPU_REG_BLOCK_ID_X:
        ret = s->simt.block_id[0];
        break;
    case GPGPU_REG_BLOCK_ID_Y:
        ret = s->simt.block_id[1];
        break;
    case GPGPU_REG_BLOCK_ID_Z:
        ret = s->simt.block_id[2];
        break;
    case GPGPU_REG_WARP_ID:
        ret = s->simt.warp_id;
        break;
    case GPGPU_REG_LANE_ID:
        ret = s->simt.lane_id;
        break;

    /* 同步寄存器组 */
    case GPGPU_REG_BARRIER:
        ret = 0;
        break;
    case GPGPU_REG_THREAD_MASK:
        ret = s->simt.thread_mask;
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "GPGPU: unknown ctrl read at offset 0x%" HWADDR_PRIx "\n",
                      addr);
        break;
    }

    return ret;
}

static void gpgpu_ctrl_write(void *opaque, hwaddr addr, uint64_t val,
                             unsigned size)
{
    GPGPUState *s = GPGPU(opaque);

    switch (addr) {
    /* 设备信息寄存器组 — 只读，忽略写入 */
    case GPGPU_REG_DEV_ID:
    case GPGPU_REG_DEV_VERSION:
    case GPGPU_REG_DEV_CAPS:
    case GPGPU_REG_VRAM_SIZE_LO:
    case GPGPU_REG_VRAM_SIZE_HI:
        break;

    /* 全局控制寄存器组 */
    case GPGPU_REG_GLOBAL_CTRL:
        s->global_ctrl = val;
        /* §6.5 软复位: global_ctrl 清零，状态恢复 READY，错误/中断/SIMT/内核/DMA 清零 */
        if (val & GPGPU_CTRL_RESET) {
            s->global_ctrl = 0;
            s->global_status = GPGPU_STATUS_READY;
            s->error_status = 0;
            s->irq_enable = 0;
            s->irq_status = 0;
            memset(&s->kernel, 0, sizeof(s->kernel));
            memset(&s->dma, 0, sizeof(s->dma));
            memset(&s->simt, 0, sizeof(s->simt));
        }
        break;
    case GPGPU_REG_GLOBAL_STATUS:
        /* 只读，忽略写入 */
        break;
    case GPGPU_REG_ERROR_STATUS:
        /* 写 1 清除 */
        s->error_status &= ~val;
        break;

    /* 中断控制寄存器组 */
    case GPGPU_REG_IRQ_ENABLE:
        s->irq_enable = val;
        break;
    case GPGPU_REG_IRQ_STATUS:
        /* 只读，忽略写入 */
        break;
    case GPGPU_REG_IRQ_ACK:
        /* 写 1 清除中断状态 */
        s->irq_status &= ~val;
        break;

    /* 内核分发寄存器组 */
    case GPGPU_REG_KERNEL_ADDR_LO:
        s->kernel.kernel_addr = (s->kernel.kernel_addr & 0xFFFFFFFF00000000ULL) | val;
        break;
    case GPGPU_REG_KERNEL_ADDR_HI:
        s->kernel.kernel_addr = (s->kernel.kernel_addr & 0xFFFFFFFFULL) | ((uint64_t)val << 32);
        break;
    case GPGPU_REG_KERNEL_ARGS_LO:
        s->kernel.kernel_args = (s->kernel.kernel_args & 0xFFFFFFFF00000000ULL) | val;
        break;
    case GPGPU_REG_KERNEL_ARGS_HI:
        s->kernel.kernel_args = (s->kernel.kernel_args & 0xFFFFFFFFULL) | ((uint64_t)val << 32);
        break;
    case GPGPU_REG_GRID_DIM_X:
        s->kernel.grid_dim[0] = val;
        break;
    case GPGPU_REG_GRID_DIM_Y:
        s->kernel.grid_dim[1] = val;
        break;
    case GPGPU_REG_GRID_DIM_Z:
        s->kernel.grid_dim[2] = val;
        break;
    case GPGPU_REG_BLOCK_DIM_X:
        s->kernel.block_dim[0] = val;
        break;
    case GPGPU_REG_BLOCK_DIM_Y:
        s->kernel.block_dim[1] = val;
        break;
    case GPGPU_REG_BLOCK_DIM_Z:
        s->kernel.block_dim[2] = val;
        break;
    case GPGPU_REG_SHARED_MEM_SIZE:
        s->kernel.shared_mem_size = val;
        break;
    case GPGPU_REG_DISPATCH:
        /* §8.2: 写任意值触发内核执行，需通过验证检查 */
        if (val) {
            bool valid = true;

            /* 检查 1: 设备必须已使能 */
            if (!(s->global_ctrl & GPGPU_CTRL_ENABLE)) {
                s->error_status |= GPGPU_ERR_INVALID_CMD;
                valid = false;
            }

            /* 检查 2: 设备不能正忙 */
            if (s->global_status & GPGPU_STATUS_BUSY) {
                s->error_status |= GPGPU_ERR_INVALID_CMD;
                valid = false;
            }

            /* 检查 3: 所有 Grid 维度必须 > 0 */
            if (s->kernel.grid_dim[0] == 0 ||
                s->kernel.grid_dim[1] == 0 ||
                s->kernel.grid_dim[2] == 0) {
                s->error_status |= GPGPU_ERR_INVALID_CMD;
                valid = false;
            }

            /* 检查 4: 所有 Block 维度必须 > 0 */
            if (s->kernel.block_dim[0] == 0 ||
                s->kernel.block_dim[1] == 0 ||
                s->kernel.block_dim[2] == 0) {
                s->error_status |= GPGPU_ERR_INVALID_CMD;
                valid = false;
            }

            /* 检查 5: 内核地址不能越界 */
            if (s->kernel.kernel_addr >= s->vram_size) {
                s->error_status |= GPGPU_ERR_INVALID_CMD;
                valid = false;
            }

            if (valid) {
                int exec_ret;

                s->global_status &= ~GPGPU_STATUS_READY;
                s->global_status |= GPGPU_STATUS_BUSY;

                /* §12: 调用 SIMT 执行引擎 */
                exec_ret = gpgpu_core_exec_kernel(s);

                if (exec_ret == 0) {
                    s->global_status &= ~GPGPU_STATUS_BUSY;
                    s->global_status |= GPGPU_STATUS_READY;
                    if (s->irq_enable & GPGPU_IRQ_KERNEL_DONE) {
                        s->irq_status |= GPGPU_IRQ_KERNEL_DONE;
                    }
                } else {
                    /* §12.4: 超时或其他执行错误 */
                    s->global_status &= ~GPGPU_STATUS_BUSY;
                    s->global_status |= GPGPU_STATUS_ERROR;
                    s->error_status |= GPGPU_ERR_KERNEL_FAULT;
                    if (s->irq_enable & GPGPU_IRQ_ERROR) {
                        s->irq_status |= GPGPU_IRQ_ERROR;
                    }
                }
            }
        }
        break;

    /* DMA 引擎寄存器组 */
    case GPGPU_REG_DMA_SRC_LO:
        s->dma.src_addr = (s->dma.src_addr & 0xFFFFFFFF00000000ULL) | val;
        break;
    case GPGPU_REG_DMA_SRC_HI:
        s->dma.src_addr = (s->dma.src_addr & 0xFFFFFFFFULL) | ((uint64_t)val << 32);
        break;
    case GPGPU_REG_DMA_DST_LO:
        s->dma.dst_addr = (s->dma.dst_addr & 0xFFFFFFFF00000000ULL) | val;
        break;
    case GPGPU_REG_DMA_DST_HI:
        s->dma.dst_addr = (s->dma.dst_addr & 0xFFFFFFFFULL) | ((uint64_t)val << 32);
        break;
    case GPGPU_REG_DMA_SIZE:
        s->dma.size = val;
        break;
    case GPGPU_REG_DMA_CTRL:
        /* §9.2: [2]=IRQ_EN, [1]=DIR, [0]=START */
        s->dma.ctrl = val;
        if (val & GPGPU_DMA_START) {
            uint64_t host_addr, vram_offset;

            /*
             * §9.4 方向:
             *   DIR=0: Host→VRAM, SRC=主机物理地址, DST=VRAM偏移
             *   DIR=1: VRAM→Host, SRC=VRAM偏移, DST=主机物理地址
             */
            if (val & GPGPU_DMA_DIR_FROM_VRAM) {
                host_addr = s->dma.dst_addr;
                vram_offset = s->dma.src_addr;
            } else {
                host_addr = s->dma.src_addr;
                vram_offset = s->dma.dst_addr;
            }

            /* VRAM 越界检查 */
            if (vram_offset + s->dma.size > s->vram_size ||
                s->dma.size == 0) {
                s->dma.status = GPGPU_DMA_ERROR;
                s->dma.ctrl &= ~GPGPU_DMA_START;
                s->error_status |= GPGPU_ERR_DMA_FAULT;
                if (s->irq_enable & GPGPU_IRQ_ERROR) {
                    s->irq_status |= GPGPU_IRQ_ERROR;
                }
                break;
            }

            /* 分配临时缓冲区 */
            uint8_t *dma_buf = g_malloc(s->dma.size);

            s->dma.status = GPGPU_DMA_BUSY;

            if (val & GPGPU_DMA_DIR_FROM_VRAM) {
                /* VRAM → Host: 从 VRAM 读取，写入主机物理地址 */
                memcpy(dma_buf, s->vram_ptr + vram_offset, s->dma.size);
                cpu_physical_memory_write(host_addr, dma_buf, s->dma.size);
            } else {
                /* Host → VRAM: 从主机物理地址读取，写入 VRAM */
                cpu_physical_memory_read(host_addr, dma_buf, s->dma.size);
                memcpy(s->vram_ptr + vram_offset, dma_buf, s->dma.size);
            }

            g_free(dma_buf);

            /* §9.4: 1ms 虚拟定时器延迟后触发完成 */
            timer_mod(s->dma_timer,
                      qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 1);
        }
        break;
    case GPGPU_REG_DMA_STATUS:
        /* 只读，忽略写入 */
        break;

    /* SIMT 线程上下文寄存器组 */
    case GPGPU_REG_THREAD_ID_X:
        s->simt.thread_id[0] = val;
        break;
    case GPGPU_REG_THREAD_ID_Y:
        s->simt.thread_id[1] = val;
        break;
    case GPGPU_REG_THREAD_ID_Z:
        s->simt.thread_id[2] = val;
        break;
    case GPGPU_REG_BLOCK_ID_X:
        s->simt.block_id[0] = val;
        break;
    case GPGPU_REG_BLOCK_ID_Y:
        s->simt.block_id[1] = val;
        break;
    case GPGPU_REG_BLOCK_ID_Z:
        s->simt.block_id[2] = val;
        break;
    case GPGPU_REG_WARP_ID:
        s->simt.warp_id = val;
        break;
    case GPGPU_REG_LANE_ID:
        s->simt.lane_id = val;
        break;

    /* 同步寄存器组 */
    case GPGPU_REG_BARRIER:
        /* 写任意值触发 barrier */
        if (val) {
            s->simt.barrier_count++;
            if (s->simt.barrier_count >= s->simt.barrier_target) {
                s->simt.barrier_count = 0;
                s->simt.barrier_active = false;
            }
        }
        break;
    case GPGPU_REG_THREAD_MASK:
        s->simt.thread_mask = val;
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "GPGPU: unknown ctrl write at offset 0x%" HWADDR_PRIx
                      " val 0x%" PRIx64 "\n", addr, val);
        break;
    }
}

static const MemoryRegionOps gpgpu_ctrl_ops = {
    .read = gpgpu_ctrl_read,
    .write = gpgpu_ctrl_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static uint64_t gpgpu_vram_read(void *opaque, hwaddr addr, unsigned size)
{
    GPGPUState *s = GPGPU(opaque);
    uint64_t ret = 0;

    if (addr + size > s->vram_size) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "GPGPU: VRAM read out of bounds: addr=0x%" HWADDR_PRIx
                      " size=%u\n", addr, size);
        return 0;
    }

    memcpy(&ret, s->vram_ptr + addr, size);
    return ret;
}

static void gpgpu_vram_write(void *opaque, hwaddr addr, uint64_t val,
                             unsigned size)
{
    GPGPUState *s = GPGPU(opaque);

    if (addr + size > s->vram_size) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "GPGPU: VRAM write out of bounds: addr=0x%" HWADDR_PRIx
                      " size=%u\n", addr, size);
        return;
    }

    memcpy(s->vram_ptr + addr, &val, size);
}

static const MemoryRegionOps gpgpu_vram_ops = {
    .read = gpgpu_vram_read,
    .write = gpgpu_vram_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

static uint64_t gpgpu_doorbell_read(void *opaque, hwaddr addr, unsigned size)
{
    (void)opaque;
    (void)addr;
    (void)size;
    return 0;
}

static void gpgpu_doorbell_write(void *opaque, hwaddr addr, uint64_t val,
                                 unsigned size)
{
    (void)opaque;
    (void)addr;
    (void)val;
    (void)size;
}

static const MemoryRegionOps gpgpu_doorbell_ops = {
    .read = gpgpu_doorbell_read,
    .write = gpgpu_doorbell_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

/* §9.4: DMA 完成处理 — 由 1ms 虚拟定时器触发 */
static void gpgpu_dma_complete(void *opaque)
{
    GPGPUState *s = GPGPU(opaque);

    s->dma.status &= ~GPGPU_DMA_BUSY;
    s->dma.status |= GPGPU_DMA_COMPLETE;
    s->dma.ctrl &= ~GPGPU_DMA_START;

    /* 检查 DMA_CTRL.IRQ_EN 以及全局 IRQ_ENABLE.DMA_DONE */
    if ((s->dma.ctrl & GPGPU_DMA_IRQ_ENABLE) &&
        (s->irq_enable & GPGPU_IRQ_DMA_DONE)) {
        s->irq_status |= GPGPU_IRQ_DMA_DONE;
        /* TODO: 触发 MSI-X 中断向量 1 */
    }
}

/* TODO: Implement kernel completion handler */
static void gpgpu_kernel_complete(void *opaque)
{
    (void)opaque;
}

static void gpgpu_realize(PCIDevice *pdev, Error **errp)
{
    GPGPUState *s = GPGPU(pdev);
    uint8_t *pci_conf = pdev->config;

    pci_config_set_interrupt_pin(pci_conf, 1);

    s->vram_ptr = g_malloc0(s->vram_size);
    if (!s->vram_ptr) {
        error_setg(errp, "GPGPU: failed to allocate VRAM");
        return;
    }

    /* BAR 0: control registers */
    memory_region_init_io(&s->ctrl_mmio, OBJECT(s), &gpgpu_ctrl_ops, s,
                          "gpgpu-ctrl", GPGPU_CTRL_BAR_SIZE);
    pci_register_bar(pdev, 0,
                     PCI_BASE_ADDRESS_SPACE_MEMORY |
                     PCI_BASE_ADDRESS_MEM_TYPE_64,
                     &s->ctrl_mmio);

    /* BAR 2: VRAM */
    memory_region_init_io(&s->vram, OBJECT(s), &gpgpu_vram_ops, s,
                          "gpgpu-vram", s->vram_size);
    pci_register_bar(pdev, 2,
                     PCI_BASE_ADDRESS_SPACE_MEMORY |
                     PCI_BASE_ADDRESS_MEM_TYPE_64 |
                     PCI_BASE_ADDRESS_MEM_PREFETCH,
                     &s->vram);

    /* BAR 4: doorbell registers */
    memory_region_init_io(&s->doorbell_mmio, OBJECT(s), &gpgpu_doorbell_ops, s,
                          "gpgpu-doorbell", GPGPU_DOORBELL_BAR_SIZE);
    pci_register_bar(pdev, 4,
                     PCI_BASE_ADDRESS_SPACE_MEMORY,
                     &s->doorbell_mmio);

    if (msix_init(pdev, GPGPU_MSIX_VECTORS,
                  &s->ctrl_mmio, 0, 0xFE000,
                  &s->ctrl_mmio, 0, 0xFF000,
                  0, errp)) {
        g_free(s->vram_ptr);
        return;
    }

    msi_init(pdev, 0, 1, true, false, errp);

    s->dma_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, gpgpu_dma_complete, s);
    s->kernel_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL,
                                   gpgpu_kernel_complete, s);

    s->global_status = GPGPU_STATUS_READY;
}

static void gpgpu_exit(PCIDevice *pdev)
{
    GPGPUState *s = GPGPU(pdev);

    timer_free(s->dma_timer);
    timer_free(s->kernel_timer);
    g_free(s->vram_ptr);
    msix_uninit(pdev, &s->ctrl_mmio, &s->ctrl_mmio);
    msi_uninit(pdev);
}

static void gpgpu_reset(DeviceState *dev)
{
    GPGPUState *s = GPGPU(dev);

    s->global_ctrl = 0;
    s->global_status = GPGPU_STATUS_READY;
    s->error_status = 0;
    s->irq_enable = 0;
    s->irq_status = 0;
    memset(&s->kernel, 0, sizeof(s->kernel));
    memset(&s->dma, 0, sizeof(s->dma));
    memset(&s->simt, 0, sizeof(s->simt));
    timer_del(s->dma_timer);
    timer_del(s->kernel_timer);
    if (s->vram_ptr) {
        memset(s->vram_ptr, 0, s->vram_size);
    }
}

static const Property gpgpu_properties[] = {
    DEFINE_PROP_UINT32("num_cus", GPGPUState, num_cus,
                       GPGPU_DEFAULT_NUM_CUS),
    DEFINE_PROP_UINT32("warps_per_cu", GPGPUState, warps_per_cu,
                       GPGPU_DEFAULT_WARPS_PER_CU),
    DEFINE_PROP_UINT32("warp_size", GPGPUState, warp_size,
                       GPGPU_DEFAULT_WARP_SIZE),
    DEFINE_PROP_UINT64("vram_size", GPGPUState, vram_size,
                       GPGPU_DEFAULT_VRAM_SIZE),
};

static const VMStateDescription vmstate_gpgpu = {
    .name = "gpgpu",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_PCI_DEVICE(parent_obj, GPGPUState),
        VMSTATE_UINT32(global_ctrl, GPGPUState),
        VMSTATE_UINT32(global_status, GPGPUState),
        VMSTATE_UINT32(error_status, GPGPUState),
        VMSTATE_UINT32(irq_enable, GPGPUState),
        VMSTATE_UINT32(irq_status, GPGPUState),
        VMSTATE_END_OF_LIST()
    }
};

static void gpgpu_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *pc = PCI_DEVICE_CLASS(klass);

    pc->realize = gpgpu_realize;
    pc->exit = gpgpu_exit;
    pc->vendor_id = GPGPU_VENDOR_ID;
    pc->device_id = GPGPU_DEVICE_ID;
    pc->revision = GPGPU_REVISION;
    pc->class_id = GPGPU_CLASS_CODE;

    device_class_set_legacy_reset(dc, gpgpu_reset);
    dc->desc = "Educational GPGPU Device";
    dc->vmsd = &vmstate_gpgpu;
    device_class_set_props(dc, gpgpu_properties);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo gpgpu_type_info = {
    .name          = TYPE_GPGPU,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(GPGPUState),
    .class_init    = gpgpu_class_init,
    .interfaces    = (InterfaceInfo[]) {
        { INTERFACE_PCIE_DEVICE },
        { }
    },
};

static void gpgpu_register_types(void)
{
    type_register_static(&gpgpu_type_info);
}

type_init(gpgpu_register_types)
