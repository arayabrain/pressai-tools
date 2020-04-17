// SPDX-License-Identifier: GPL-2.0-only
#ifndef _ALTERA_DMA_CMD_H
#define _ALTERA_DMA_CMD_H

#define ALTERA_DMA_DRIVER_VERSION "1.00"

#define ALTERA_DMA_DID 0xE003
#define ALTERA_DMA_VID 0x1172

#define ALTERA_CMD_START_DMA            1
#define ALTERA_CMD_ENA_DIS_READ         2
#define ALTERA_CMD_ENA_DIS_WRITE        3
#define ALTERA_CMD_ENA_DIS_SIMUL        4
#define ALTERA_CMD_MODIFY_NUM_DWORDS    5
#define ALTERA_CMD_MODIFY_NUM_DESC      6
#define ALTERA_CMD_ONCHIP_OFFCHIP		7
#define ALTERA_LOOP                     8
#define ALTERA_CMD_READ_STATUS          9
#define ALTERA_EXIT                     10
#define ALTERA_CMD_WAIT_ACK             11
#define ALTERA_CMD_ALLOC_RP_BUFFER      12
#define ALTERA_CMD_RAND					13

#define ALTERA_CMD_FPGA_RESET_ON      	14
#define ALTERA_CMD_FPGA_RESET_OFF      	15
#define ALTERA_CMD_FIRMWARE_UPDATE      16
#define ALTERA_CMD_WRITE_DMA_END        17
#define ALTERA_CMD_READ_DMA_FIRM        18
#define ALTERA_CMD_READ_DMA_IMG         19
#define ALTERA_CMD_READ_DMA_RESIZE      20
#define ALTERA_CMD_READ_DMA_DL_SUB      21
#define ALTERA_CMD_DMA_ADRS_OFFSET      22
#define ALTERA_CMD_READ_DMA             23
#define ALTERA_CMD_WRITE_DMA            24

#include <linux/ioctl.h>

#define ALTERA_IOC_MAGIC   0x66
#define ALTERA_IOCX_START            _IO(ALTERA_IOC_MAGIC, ALTERA_CMD_START_DMA)
#define ALTERA_IOCX_ALLOC_RP_BUFFER  _IO(ALTERA_IOC_MAGIC, ALTERA_CMD_ALLOC_RP_BUFFER)

#define CMD_WAIT_ACK                 _IO(ALTERA_IOC_MAGIC, ALTERA_CMD_WAIT_ACK)
#define CMD_WRITE_DMA_END          	 _IO(ALTERA_IOC_MAGIC, ALTERA_CMD_WRITE_DMA_END)
#define CMD_FPGA_RESET_ON          	 _IO(ALTERA_IOC_MAGIC, ALTERA_CMD_FPGA_RESET_ON)
#define CMD_FPGA_RESET_OFF           _IO(ALTERA_IOC_MAGIC, ALTERA_CMD_FPGA_RESET_OFF)
#define CMD_FIRMWARE_UPDATE          _IO(ALTERA_IOC_MAGIC, ALTERA_CMD_FIRMWARE_UPDATE)
#define CMD_READ_DMA_FIRM            _IO(ALTERA_IOC_MAGIC, ALTERA_CMD_READ_DMA_FIRM)
#define CMD_READ_DMA_IMG             _IO(ALTERA_IOC_MAGIC, ALTERA_CMD_READ_DMA_IMG)
#define CMD_READ_DMA_RESIZE          _IO(ALTERA_IOC_MAGIC, ALTERA_CMD_READ_DMA_RESIZE)
#define CMD_READ_DMA_DL_SUB          _IO(ALTERA_IOC_MAGIC, ALTERA_CMD_READ_DMA_DL_SUB)
#define CMD_DMA_ADRS_OFFSET          _IO(ALTERA_IOC_MAGIC, ALTERA_CMD_DMA_ADRS_OFFSET)
#define CMD_READ_DMA                 _IO(ALTERA_IOC_MAGIC, ALTERA_CMD_READ_DMA)
#define CMD_WRITE_DMA                _IO(ALTERA_IOC_MAGIC, ALTERA_CMD_WRITE_DMA)

#ifndef __KERNEL__

#include <sys/ioctl.h>

#endif

struct dma_cmd {
    int cmd;
    int bank;
    int usr_buf_size;
    char *buf;
};

struct dma_status {
    char run_write;
    char run_read;
    char run_simul;
    int length_transfer;
    int altera_dma_num_dwords;
    int altera_dma_descriptor_num;
    struct timeval write_time;
    struct timeval read_time;
    struct timeval simul_time;
    char pass_read;
    char pass_write;
    char pass_simul;
    char read_eplast_timeout;
    char write_eplast_timeout;
    int offset;
    char onchip;
    char rand;
};

#endif /* _ALTERA_DMA_CMD_H */
