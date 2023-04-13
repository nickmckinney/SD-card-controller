#include <stdint.h>
#include <memory.h>
#include "mmc.h"

#define MMC_INPUT_FREQ 50000000
#define SDCARD_REG_BASE 0x00002400

// the one and only MMC device
struct mmc mmc;

// IP core registers
struct sdcard_core {
    uint32_t argument;               // 0x00
    uint32_t command;                // 0x04
    const uint32_t response[4];      // 0x08 - 0x14
    uint32_t data_timeout;           // 0x18
    uint32_t control;                // 0x1C
    uint32_t cmd_timeout;            // 0x20
    uint32_t clock_divider;          // 0x24
    uint32_t reset;                  // 0x28
    const uint32_t voltage;          // 0x2C
    const uint32_t capabilities;     // 0x30
    uint32_t cmd_int_status;       // 0x34
    uint32_t cmd_int_enable;       // 0x38
    uint32_t data_int_status;      // 0x3C
    uint32_t data_int_enable;      // 0x40
    uint32_t blk_size;               // 0x44
    uint32_t blk_count;              // 0x48
    uint32_t reserved[5];            // 0x4C - 0x5C
    uint32_t dst_src_addr;           // 0x60
};

volatile struct sdcard_core * sdcard_regs = (struct sdcard_core *) SDCARD_REG_BASE;

static void sdcard_set_bus_width(uint32_t width) {
    sdcard_regs->control = (width == 4) ? 1 : 0;
}

static void sdcard_set_clock(uint32_t dev_clock, uint32_t output_clock) {
    uint32_t divider = dev_clock / (output_clock * 2) - 1;

    // need to assert reset while changing the clock frequency
    sdcard_regs->reset = 1;
    sdcard_regs->clock_divider = divider;
    sdcard_regs->reset = 0;
}

static int sdcard_wait_for_cmd(struct mmc_cmd *cmd) {
    // TODO: use an interrupt for this instead?
    uint32_t status;

    while((status = sdcard_regs->cmd_int_status) == 0)
        ;

    sdcard_regs->cmd_int_status = 0;

    if (status & 0x02) {
        return -1;
    }

    cmd->response[0] = sdcard_regs->response[0];
    if (cmd->resp_type & MMC_RSP_136) {
        cmd->response[1] = sdcard_regs->response[1];
        cmd->response[2] = sdcard_regs->response[2];
        cmd->response[3] = sdcard_regs->response[3];
    }

    return 0;
}

static int sdcard_wait_for_data_finish() {
    // TODO: use an interrupt for this instead?
    uint32_t status;

    while((status = sdcard_regs->data_int_status) == 0)
        ;

    sdcard_regs->data_int_status = 0;

    if (status & 0x02) {
        return -1;
    }

    return 0;
}

static int sdcard_send_cmd(struct mmc * ignored, struct mmc_cmd *cmd, struct mmc_data *data) {
    uint32_t command = cmd->cmdidx << 8;

    if (cmd->resp_type & MMC_RSP_PRESENT) {
        if (cmd->resp_type & MMC_RSP_136) {
            command |= 0x2;
        } else {
            command |= 0x1;
        }
    }
    if (cmd->resp_type & MMC_RSP_BUSY) {
        command |= 0x4;
    }
    if (cmd->resp_type & MMC_RSP_CRC) {
        command |= 0x8;
    }
    if (cmd->resp_type & MMC_RSP_OPCODE) {
        command |= 0x10;
    }
    if (data && data->blocks && ((data->flags & MMC_DATA_READ) || (data->flags & MMC_DATA_WRITE))) {
        if (data->flags & MMC_DATA_READ) {
            sdcard_regs->dst_src_addr = (uint32_t) data->dest;
            command |= 0x20;
        }
        if (data->flags & MMC_DATA_WRITE) {
            sdcard_regs->dst_src_addr = (uint32_t) data->src;
            command |= 0x40;
        }

        sdcard_regs->blk_size = data->blocksize - 1;
        sdcard_regs->blk_count = data->blocks - 1;
    }

    sdcard_regs->command = command;

    // setting argument register is what triggers starting the command
    sdcard_regs->argument = cmd->cmdarg;

    if (sdcard_wait_for_cmd(cmd) < 0) {
        return -1;
    }
    if (data && data->blocks) {
        if (sdcard_wait_for_data_finish() < 0) {
            return -1;
        }
    }

    return 0;
}

static void sdcard_set_ios(struct mmc * ignored) {
    sdcard_set_bus_width(mmc.bus_width);
    if (mmc.clock != 0) {
        sdcard_set_clock(MMC_INPUT_FREQ, mmc.clock);
    }
}

static int sdcard_priv_init(struct mmc * mmc) {
    sdcard_regs->cmd_timeout = 0x7FFF;
    sdcard_regs->cmd_int_enable = 0;
    sdcard_regs->data_int_enable = 0;
    sdcard_regs->cmd_int_status = 0;
    sdcard_regs->data_int_status = 0;

    sdcard_set_clock(MMC_INPUT_FREQ, MMC_INPUT_FREQ / 2);

    return 0;
}

static uint32_t get_mmc_voltage() {
    uint32_t mV = sdcard_regs->voltage;

    return MMC_VDD_32_33 | MMC_VDD_33_34;
    /*if (mV < 1900) { return MMC_VDD_165_195; }
    if (mV < 2050) { return MMC_VDD_20_21; }
    if (mV < 2150) { return MMC_VDD_20_21 | MMC_VDD_21_22; }
    if (mV < 2250) { return MMC_VDD_21_22 | MMC_VDD_22_23; }
    if (mV < 2350) { return MMC_VDD_22_23 | MMC_VDD_23_24; }
    if (mV < 2450) { return MMC_VDD_23_24 | MMC_VDD_24_25; }
    if (mV < 2550) { return MMC_VDD_24_25 | MMC_VDD_25_26; }
    if (mV < 2650) { return MMC_VDD_25_26 | MMC_VDD_26_27; }
    if (mV < 2750) { return MMC_VDD_26_27 | MMC_VDD_27_28; }
    if (mV < 2850) { return MMC_VDD_27_28 | MMC_VDD_28_29; }
    if (mV < 2950) { return MMC_VDD_28_29 | MMC_VDD_29_30; }
    if (mV < 3050) { return MMC_VDD_29_30 | MMC_VDD_30_31; }
    if (mV < 3150) { return MMC_VDD_30_31 | MMC_VDD_31_32; }
    if (mV < 3250) { return MMC_VDD_31_32 | MMC_VDD_32_33; }
    if (mV < 3350) { return MMC_VDD_32_33 | MMC_VDD_33_34; }
    if (mV < 3450) { return MMC_VDD_33_34 | MMC_VDD_34_35; }
    if (mV < 3550) { return MMC_VDD_34_35 | MMC_VDD_35_36; }
    return MMC_VDD_35_36;*/
}

void sdcard_startup() {
	memset(&mmc, 0, sizeof(struct mmc));

    mmc.send_cmd = sdcard_send_cmd;
    mmc.set_ios = sdcard_set_ios;
    mmc.init = sdcard_priv_init;
    mmc.getcd = NULL;
    mmc.f_max = MMC_INPUT_FREQ / 2;
    mmc.f_min = MMC_INPUT_FREQ / 6;
    mmc.voltages = get_mmc_voltage();
    mmc.host_caps = MMC_MODE_4BIT;
    mmc.b_max = 256;
}

int sdcard_init() {
    return mmc_init(&mmc);
}

uint32_t sdcard_block_read(uint32_t start, uint32_t blkcnt, void *dst) {
    return mmc_bread(&mmc, start, blkcnt, dst);
}
