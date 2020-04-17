// SPDX-License-Identifier: GPL-2.0-only
#include <linux/time.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/pci.h>
#include <linux/cdev.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/atomic.h>
#include <linux/random.h>
#include "altera_dma_cmd.h"
#include "altera_dma.h"
#include <linux/unistd.h>
#include <linux/uaccess.h>

#define TIMEOUT 0x2000000

static int bank = 0;
static int fpga_ack = 0;
static u32 dma_read_base = 0;
static u32 dma_read_size = 0;
static u64 dma_adr_offset = 0;

static u32 dma_read_base_addr = 0;
static u32 dma_write_base_addr = 0;
bool dma_read_flag = false;
bool dma_write_flag = false;

struct timeval trg_tv1;
struct timeval trg_tv2;
struct timeval trg_diff;

static long altera_dma_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{

  printk(KERN_DEBUG "CMD = %d\n", cmd);

  struct altera_pcie_dma_bookkeep *bk_ptr = filp->private_data;
  switch (cmd)
  {

  case CMD_READ_DMA:
    // READ_DMA準備
    printk(KERN_DEBUG "CMD_READ_DMA\n");
    dma_read_flag = true;
    dma_read_base_addr = arg;
    break;
  case CMD_WRITE_DMA:
    // WRITE_DMA準備
    printk(KERN_DEBUG "CMD_WRITE_DMA\n");
    dma_write_flag = true;
    dma_write_base_addr = arg;
    break;
  case CMD_FPGA_RESET_ON:
    // FPGAリセット
    iowrite32(cpu_to_le32(0), (u32 *)(bk_ptr->bar[BAR] + BAR2_PCIEXTREG_FDONE));
    printk(KERN_DEBUG "CMD_FPGA_RESET_ON\n");
    break;
  case CMD_FPGA_RESET_OFF:
    // FPGAリセット解除
    iowrite32(cpu_to_le32(1), (u32 *)(bk_ptr->bar[BAR] + BAR2_PCIEXTREG_FDONE));
    printk(KERN_DEBUG "CMD_FPGA_RESET_OFF\n");
    break;
  case CMD_FIRMWARE_UPDATE:
    firmware_update(bk_ptr, arg);
    printk(KERN_DEBUG "CMD_FIRMWARE_UPDATE\n");
    break;
  case CMD_WRITE_DMA_END:

    printk(KERN_DEBUG "CMD_WRITE_DMA_END\n");

    // 転送完了割り込み発生
    iowrite32(cpu_to_le32(bank << 16), (u32 *)(bk_ptr->bar[BAR] + BAR2_PCIEXTREG_TXEND));

    // 転送時間計測終了
    do_gettimeofday(&trg_tv2);
    diff_timeval(&trg_diff, &trg_tv2, &trg_tv1);
    printk(KERN_DEBUG "Trig Time Interval = %d\n", trg_diff.tv_usec + trg_diff.tv_sec * 1000000);
    trg_tv1 = trg_tv2;

    // 転送毎に面を切替
    if (bank == 0)
    {
      bank = 1;
    }
    else
    {
      bank = 0;
    }

    break;
  case CMD_READ_DMA_FIRM:
    dma_read_base = PRG_MEM_BASE;
    dma_read_size = PRG_MEM_SIZE;
    printk(KERN_DEBUG "CMD_READ_DMA_FIRM\n");
    break;
  case CMD_READ_DMA_IMG:
    dma_read_base = IMG_MEM_BASE;
    dma_read_size = IMG_MEM_SIZE;
    printk(KERN_DEBUG "CMD_READ_DMA_IMG\n");
    break;
  case CMD_READ_DMA_RESIZE:
    dma_read_base = RESIZE_MEM_BASE;
    dma_read_size = RESIZE_MEM_SIZE;
    printk(KERN_DEBUG "CMD_READ_DMA_RESIZE\n");
    break;
  case CMD_READ_DMA_DL_SUB:
    dma_read_base = DL_SUB_MEM_BASE;
    dma_read_size = DL_SUB_MEM_SIZE;
    printk(KERN_DEBUG "CMD_READ_DMA_DL_SUB\n");
    break;
  case CMD_DMA_ADRS_OFFSET:
    dma_adr_offset = arg;
    printk(KERN_DEBUG "CMD_DMA_ADRS_OFFSET\n");
    break;
  case CMD_WAIT_ACK:
    wait_event_interruptible(bk_ptr->wait_q, (fpga_ack != 0));
    fpga_ack = 0;
    break;
  default:
    return -EINVAL;
  }
  return 0;
}

// ファームウェアアップデート
static int firmware_update(struct altera_pcie_dma_bookkeep *bk_ptr, unsigned long arg)
{

  int rc, max_num, num, i, j;
  struct timeval tv1;
  struct timeval tv2;
  struct timeval diff;

  char *buf = arg;

  // エラー初期値
  rc = 0;

  // 繰り返し回数計算(端数は切上)
  max_num = DMA_MAX_BYTE;
  num = (DMA_MAX_BYTE >= PRG_MEM_SIZE) ? 1 : (int)((PRG_MEM_SIZE + max_num - 1) / (max_num));

  //	printk(KERN_DEBUG "DMA WRITE MAX BYTE = %d\n",max_num);
  //	printk(KERN_DEBUG "DMA WRITE NUM      = %d\n",num);

  // 転送時間計測開始
  do_gettimeofday(&tv1);

  // DMA転送
  for (i = 0; i < num; i++)
  {

    // ユーザ空間のデータをカーネルの空間にコピー
    if (copy_from_user(bk_ptr->rp_rd_buffer_virt_addr, buf + max_num * i, max_num))
    {
      printk(KERN_DEBUG "copy_from_user error\n");
      rc = -EFAULT;
      return rc;
    }

    //		for(j=0;j<10;j++){
    //			printk(KERN_DEBUG "DATA = %d\n",*(bk_ptr->rp_rd_buffer_virt_addr+j));
    //		}

    // DMA転送(PC->FPGA)
    rc = dma_pc_to_fpga(bk_ptr, (u64)(PRG_MEM_BASE + max_num * i), (u32)max_num);

    if (rc != 0)
    {
      break;
    }
  }

  // 転送時間計測終了
  do_gettimeofday(&tv2);
  diff_timeval(&diff, &tv2, &tv1);
  bk_ptr->dma_status.read_time = diff;

  printk(KERN_DEBUG "Firm Ware Time Interval = %d\n", diff.tv_usec + diff.tv_sec * 1000000);

  // 完了割込
  wake_up(&bk_ptr->wait_q);

  return rc;
}

// ALTERA_DMAリード
ssize_t altera_dma_read(struct file *file, char __user *buf, size_t count, loff_t *pos)
{
  if (dma_read_flag)
  {
    struct altera_pcie_dma_bookkeep *bk_ptr = file->private_data;
    int rc, max_num, num, i, j;
    struct timeval tv1;
    struct timeval tv2;
    struct timeval diff;

    // 繰り返し回数計算(端数は切上)
    max_num = DMA_MAX_BYTE;
    num = (DMA_MAX_BYTE >= count) ? 1 : (int)((count + max_num - 1) / (max_num));

    printk(KERN_DEBUG "DMA READ BASE     = %x\n", dma_read_base_addr);
    printk(KERN_DEBUG "DMA READ SIZE     = %x\n", count);

    // 転送時間計測開始
    do_gettimeofday(&tv1);

    // DMA転送
    for (i = 0; i < num; i++)
    {

      // DMA転送(FPGA->PC)
      rc = dma_fpga_to_pc(bk_ptr, (u64)(dma_read_base_addr + max_num * i), (u32)max_num);

      if (rc != 0)
      {
        break;
      }

      // カーネルの空間のデータをユーザ空間にコピー
      if (copy_to_user(buf + max_num * i, bk_ptr->rp_wr_buffer_virt_addr, max_num))
      {
        printk(KERN_DEBUG "copy_to_user error\n");
        rc = -EFAULT;
        return rc;
      }

      //		for(j=0;j<10;j++){
      //			printk(KERN_DEBUG "DATA = %d\n",*(bk_ptr->rp_wr_buffer_virt_addr+j));
      //		}
    }

    // 転送時間計測終了
    do_gettimeofday(&tv2);
    diff_timeval(&diff, &tv2, &tv1);
    bk_ptr->dma_status.read_time = diff;

    printk(KERN_DEBUG "DMA Read Time Interval = %d\n", diff.tv_usec + diff.tv_sec * 1000000);

    // 完了割込
    wake_up(&bk_ptr->wait_q);

    dma_read_flag = false;
    return rc;
  }

  struct altera_pcie_dma_bookkeep *bk_ptr = file->private_data;
  int rc, max_num, num, i, j;
  int read_bank = 0;
  struct timeval tv1;
  struct timeval tv2;
  struct timeval diff;

  // 繰り返し回数計算(端数は切上)
  max_num = DMA_MAX_BYTE;
  num = (DMA_MAX_BYTE >= count) ? 1 : (int)((count + max_num - 1) / (max_num));

  //	printk(KERN_DEBUG "DMA READ MAX BYTE = %d\n",max_num);
  //	printk(KERN_DEBUG "DMA READ NUM      = %d\n",num);
  printk(KERN_DEBUG "DMA READ BASE     = %x\n", dma_read_base);
  printk(KERN_DEBUG "DMA READ SIZE     = %x\n", dma_read_size);
  printk(KERN_DEBUG "DMA READ OFFSET   = %x\n", dma_adr_offset);

  // リード面生成
  if (dma_read_base == PRG_MEM_BASE)
  { // ファーム領域リード時はBANK管理なし
    read_bank = 0;
  }
  else
  { // ファーム領域以外のリード時はBANK情報を読み出し
    read_bank = (ioread32((u32 *)(bk_ptr->bar[BAR] + BAR2_PCIEXTREG_TXEND)) & 0x00020000) >> 17;
  }
  printk(KERN_DEBUG "DMA READ BANK     = %d\n", read_bank);

  // リード対象ベースアドレス、サイズ選択

  // 転送時間計測開始
  do_gettimeofday(&tv1);

  // DMA転送
  for (i = 0; i < num; i++)
  {

    // DMA転送(FPGA->PC)
    rc = dma_fpga_to_pc(bk_ptr, (u64)(dma_read_base + dma_adr_offset + (read_bank * dma_read_size) + max_num * i), (u32)max_num);

    if (rc != 0)
    {
      break;
    }

    // カーネルの空間のデータをユーザ空間にコピー
    if (copy_to_user(buf + max_num * i, bk_ptr->rp_wr_buffer_virt_addr, max_num))
    {
      printk(KERN_DEBUG "copy_to_user error\n");
      rc = -EFAULT;
      return rc;
    }

    //		for(j=0;j<10;j++){
    //			printk(KERN_DEBUG "DATA = %d\n",*(bk_ptr->rp_wr_buffer_virt_addr+j));
    //		}
  }

  // 転送時間計測終了
  do_gettimeofday(&tv2);
  diff_timeval(&diff, &tv2, &tv1);
  bk_ptr->dma_status.read_time = diff;

  printk(KERN_DEBUG "DMA Read Time Interval = %d\n", diff.tv_usec + diff.tv_sec * 1000000);

  // 完了割込
  wake_up(&bk_ptr->wait_q);

  return rc;
}

// ALTERA_DMAライト
ssize_t altera_dma_write(struct file *file, char __user *buf, size_t count, loff_t *pos)
{
  if (dma_write_flag)
  {
    struct altera_pcie_dma_bookkeep *bk_ptr = file->private_data;
    int rc, max_num, num, i, j;
    struct timeval tv1;
    struct timeval tv2;
    struct timeval diff;

    // エラー初期値
    rc = 0;

    // 繰り返し回数計算(端数は切上)
    max_num = DMA_MAX_BYTE;
    num = (DMA_MAX_BYTE >= count) ? 1 : (int)((count + max_num - 1) / (max_num));

    printk(KERN_DEBUG "DMA WRITE BASE     = %x\n", dma_write_base_addr);
    printk(KERN_DEBUG "DMA WRITE SIZE     = %x\n", count);

    // 転送時間計測開始
    do_gettimeofday(&tv1);

    // DMA転送
    for (i = 0; i < num; i++)
    {

      // ユーザ空間のデータをカーネルの空間にコピー
      if (copy_from_user(bk_ptr->rp_rd_buffer_virt_addr, buf + max_num * i, max_num))
      {
        printk(KERN_DEBUG "copy_from_user error\n");
        rc = -EFAULT;
        return rc;
      }

      // DMA転送(PC->FPGA)
      rc = dma_pc_to_fpga(bk_ptr, (u64)(dma_write_base_addr + max_num * i), (u32)max_num);

      if (rc != 0)
      {
        break;
      }
    }

    // 転送時間計測終了
    do_gettimeofday(&tv2);
    diff_timeval(&diff, &tv2, &tv1);
    bk_ptr->dma_status.read_time = diff;

    printk(KERN_DEBUG "DMA Write Time Interval = %d\n", diff.tv_usec + diff.tv_sec * 1000000);

    // 完了割込
    wake_up(&bk_ptr->wait_q);

    dma_write_flag = false;
    return rc;
  }

  struct altera_pcie_dma_bookkeep *bk_ptr = file->private_data;
  int rc, max_num, num, i, j;
  struct timeval tv1;
  struct timeval tv2;
  struct timeval diff;

  // エラー初期値
  rc = 0;

  // 繰り返し回数計算(端数は切上)
  max_num = DMA_MAX_BYTE;
  num = (DMA_MAX_BYTE >= count) ? 1 : (int)((count + max_num - 1) / (max_num));

  //	printk(KERN_DEBUG "DMA WRITE MAX BYTE = %d\n",max_num);
  //	printk(KERN_DEBUG "DMA WRITE NUM      = %d\n",num);
  printk(KERN_DEBUG "DMA WRITE BASE     = %x\n", IMG_MEM_BASE);
  printk(KERN_DEBUG "DMA WRITE SIZE     = %x\n", IMG_MEM_SIZE);
  printk(KERN_DEBUG "DMA WRITE OFFSET   = %x\n", dma_adr_offset);
  printk(KERN_DEBUG "DMA WRITE BANK     = %x\n", bank);

  // 転送時間計測開始
  do_gettimeofday(&tv1);

  // DMA転送
  for (i = 0; i < num; i++)
  {

    // ユーザ空間のデータをカーネルの空間にコピー
    if (copy_from_user(bk_ptr->rp_rd_buffer_virt_addr, buf + max_num * i, max_num))
    {
      printk(KERN_DEBUG "copy_from_user error\n");
      rc = -EFAULT;
      return rc;
    }

    //		for(j=0;j<10;j++){
    //			printk(KERN_DEBUG "DATA = %d\n",*(bk_ptr->rp_rd_buffer_virt_addr+j));
    //		}

    // DMA転送(PC->FPGA)
    rc = dma_pc_to_fpga(bk_ptr, (u64)(IMG_MEM_BASE + dma_adr_offset + (bank * IMG_MEM_SIZE) + max_num * i), (u32)max_num);

    if (rc != 0)
    {
      break;
    }
  }

  // 転送時間計測終了
  do_gettimeofday(&tv2);
  diff_timeval(&diff, &tv2, &tv1);
  bk_ptr->dma_status.read_time = diff;

  printk(KERN_DEBUG "DMA Write Time Interval = %d\n", diff.tv_usec + diff.tv_sec * 1000000);

  // 完了割込
  wake_up(&bk_ptr->wait_q);

  return rc;
}

// DMA転送(FPGA->PC)※FPGAからみてライト転送
static int dma_fpga_to_pc(struct altera_pcie_dma_bookkeep *bk_ptr, u64 fpga_adr, u32 len)
{

  // ローカル変数
  dma_addr_t rp_wr_buffer_bus_addr = bk_ptr->rp_wr_buffer_bus_addr;
  int i, j;
  u32 last_id, write_127;
  u32 timeout;

  int rc = 0;

  // 初期化
  atomic_set(&bk_ptr->status, 1);
  bk_ptr->dma_status.pass_write = 0;
  bk_ptr->dma_status.write_eplast_timeout = 0;
  bk_ptr->dma_status.altera_dma_num_dwords = ALTERA_DMA_NUM_DWORDS; // 転送サイズは8KB単位に固定

  // ディスクリプタ数設定(端数は切り上げ)
  //  bk_ptr->dma_status.altera_dma_descriptor_num = ALTERA_DMA_DESCRIPTOR_NUM;
  bk_ptr->dma_status.altera_dma_descriptor_num = (ALTERA_DMA_NUM_DWORDS >= (len / 4)) ? 1 : (int)(((len / 4) + ALTERA_DMA_NUM_DWORDS - 1) / ALTERA_DMA_NUM_DWORDS);

  //	printk(KERN_DEBUG "altera_dma_num_dwords     = %d\n",bk_ptr->dma_status.altera_dma_num_dwords);
  //	printk(KERN_DEBUG "altera_dma_descriptor_num = %d\n",bk_ptr->dma_status.altera_dma_descriptor_num);

  timeout = TIMEOUT;

  write_127 = 0;

  // 最終ID値読み出し
  last_id = ioread32((u32 *)(bk_ptr->bar[0] + DESC_CTRLLER_BASE + ALTERA_LITE_DMA_WR_LAST_PTR));
  //	printk(KERN_DEBUG "Read ID = %08x\n", last_id);

  // 転送完了フラグ初期化
  set_lite_table_header((struct lite_dma_header *)bk_ptr->lite_table_wr_cpu_virt_addr);

  wmb();

  // ディスクリプタ設定
  for (i = 0; i < bk_ptr->dma_status.altera_dma_descriptor_num; i++)
  {
    //		set_write_desc(&bk_ptr->lite_table_wr_cpu_virt_addr->descriptors[i], (u64)fpga_adr+(i*MAX_NUM_DWORDS*4), (dma_addr_t)rp_wr_buffer_bus_addr, bk_ptr->dma_status.altera_dma_num_dwords, i);
    set_write_desc(&bk_ptr->lite_table_wr_cpu_virt_addr->descriptors[i],
                   (u64)fpga_adr + (i * bk_ptr->dma_status.altera_dma_num_dwords * 4),
                   (dma_addr_t)rp_wr_buffer_bus_addr + (i * bk_ptr->dma_status.altera_dma_num_dwords * 4),
                   bk_ptr->dma_status.altera_dma_num_dwords,
                   i);
    //		set_write_desc(&bk_ptr->lite_table_wr_cpu_virt_addr->descriptors[i], (u64)fpga_adr, (dma_addr_t)rp_wr_buffer_bus_addr, bk_ptr->dma_status.altera_dma_num_dwords, i);
  }

  // ディスクリプタテーブルのPC側の先頭アドレスを通知
  // Program source write descriptor table lower 32-bit in RC into register thru bar0
  iowrite32((dma_addr_t)bk_ptr->lite_table_wr_bus_addr, bk_ptr->bar[0] + DESC_CTRLLER_BASE + ALTERA_LITE_DMA_WR_RC_LOW_SRC_ADDR);
  iowrite32(((dma_addr_t)bk_ptr->lite_table_wr_bus_addr) >> 32, bk_ptr->bar[0] + DESC_CTRLLER_BASE + ALTERA_LITE_DMA_WR_RC_HIGH_SRC_ADDR);

  // DMA制御レジスタの設定
  // (ディスクリプタの最終IDがALL1 => 始めの1回のみ設定)
  if (last_id == 0xFF)
  {
    iowrite32(WR_CTRL_BUF_BASE_LOW, bk_ptr->bar[0] + DESC_CTRLLER_BASE + ALTERA_LITE_DMA_WR_CTLR_LOW_DEST_ADDR);
    iowrite32(WR_CTRL_BUF_BASE_HI, bk_ptr->bar[0] + DESC_CTRLLER_BASE + ALTERA_LITE_DMA_WR_CTRL_HIGH_DEST_ADDR);
  }

  wmb();

  if (last_id == 0xFF)
    last_id = 127;

  last_id = last_id + bk_ptr->dma_status.altera_dma_descriptor_num;

  if (last_id > 127)
  {
    last_id = last_id - 128;
    if ((bk_ptr->dma_status.altera_dma_descriptor_num > 1) && (last_id != 127))
      write_127 = 1;
  }

  if (write_127)
    iowrite32(127, bk_ptr->bar[0] + DESC_CTRLLER_BASE + ALTERA_LITE_DMA_WR_LAST_PTR);

  iowrite32(last_id, bk_ptr->bar[0] + DESC_CTRLLER_BASE + ALTERA_LITE_DMA_WR_LAST_PTR);

  // 転送完了待ち
  while (1)
  {

    if (bk_ptr->lite_table_wr_cpu_virt_addr->header.flags[last_id])
    {
      break;
    }

    if (timeout == 0)
    {
      printk(KERN_DEBUG "Read DMA times out\n");
      bk_ptr->dma_status.read_eplast_timeout = 1;
      printk(KERN_DEBUG "DWORD = %08x\n", bk_ptr->dma_status.altera_dma_num_dwords);
      printk(KERN_DEBUG "Desc = %08x\n", bk_ptr->dma_status.altera_dma_descriptor_num);
      rc = -EFAULT;
      return rc;
      break;
    }
    timeout--;
    cpu_relax();
  }

  // タイムアウト処理
  if (timeout == 0)
  {
    bk_ptr->dma_status.pass_write = 0;
    rc = -ETIME;
  }
  else
  {
    bk_ptr->dma_status.pass_write = 1;
  }

  atomic_set(&bk_ptr->status, 0);
  return rc;
}

// DMA転送(PC->FPGA)※FPGAからみてリード転送
static int dma_pc_to_fpga(struct altera_pcie_dma_bookkeep *bk_ptr, u64 fpga_adr, u32 len)
{

  // ローカル変数
  dma_addr_t rp_rd_buffer_bus_addr = bk_ptr->rp_rd_buffer_bus_addr;
  // 	dma_addr_t rp_wr_buffer_bus_addr = bk_ptr->rp_wr_buffer_bus_addr;
  int i, j;
  u32 last_id, write_127;
  u32 timeout;

  int rc = 0;

  // 初期化
  atomic_set(&bk_ptr->status, 1);
  bk_ptr->dma_status.pass_read = 0;
  //  bk_ptr->dma_status.pass_write = 0;
  //  bk_ptr->dma_status.pass_simul = 0;
  bk_ptr->dma_status.read_eplast_timeout = 0;
  //  bk_ptr->dma_status.write_eplast_timeout = 0;
  bk_ptr->dma_status.altera_dma_num_dwords = ALTERA_DMA_NUM_DWORDS; // 転送サイズは8KB単位に固定

  // ディスクリプタ数設定(端数は切り上げ)
  //  bk_ptr->dma_status.altera_dma_descriptor_num = ALTERA_DMA_DESCRIPTOR_NUM;
  bk_ptr->dma_status.altera_dma_descriptor_num = (ALTERA_DMA_NUM_DWORDS >= (len / 4)) ? 1 : (int)(((len / 4) + ALTERA_DMA_NUM_DWORDS - 1) / ALTERA_DMA_NUM_DWORDS);

  //	printk(KERN_DEBUG "altera_dma_num_dwords     = %d\n",bk_ptr->dma_status.altera_dma_num_dwords);
  //	printk(KERN_DEBUG "altera_dma_descriptor_num = %d\n",bk_ptr->dma_status.altera_dma_descriptor_num);

  timeout = TIMEOUT;

  write_127 = 0;

  // 最終ID値読み出し
  last_id = ioread32((u32 *)(bk_ptr->bar[0] + DESC_CTRLLER_BASE + ALTERA_LITE_DMA_RD_LAST_PTR));
  //	printk(KERN_DEBUG "Read ID = %08x\n", last_id);

  // 転送完了フラグ初期化
  set_lite_table_header((struct lite_dma_header *)bk_ptr->lite_table_rd_cpu_virt_addr);

  wmb();

  // ディスクリプタ設定
  for (i = 0; i < bk_ptr->dma_status.altera_dma_descriptor_num; i++)
  {
    // 		set_read_desc(&bk_ptr->lite_table_rd_cpu_virt_addr->descriptors[i], (dma_addr_t)rp_rd_buffer_bus_addr, (u64)fpga_adr+(i*MAX_NUM_DWORDS*4), bk_ptr->dma_status.altera_dma_num_dwords, i);
    set_read_desc(&bk_ptr->lite_table_rd_cpu_virt_addr->descriptors[i],
                  (dma_addr_t)rp_rd_buffer_bus_addr + (i * bk_ptr->dma_status.altera_dma_num_dwords * 4),
                  (u64)fpga_adr + (i * bk_ptr->dma_status.altera_dma_num_dwords * 4),
                  bk_ptr->dma_status.altera_dma_num_dwords,
                  i);
    // 		set_read_desc(&bk_ptr->lite_table_rd_cpu_virt_addr->descriptors[i], (dma_addr_t)rp_rd_buffer_bus_addr, (u64)fpga_adr, bk_ptr->dma_status.altera_dma_num_dwords, i);
  }

  // ディスクリプタテーブルのPC側の先頭アドレスを通知
  // Program source write descriptor table lower 32-bit in RC into register thru bar0
  iowrite32((dma_addr_t)bk_ptr->lite_table_rd_bus_addr, bk_ptr->bar[0] + DESC_CTRLLER_BASE + ALTERA_LITE_DMA_RD_RC_LOW_SRC_ADDR);
  iowrite32(((dma_addr_t)bk_ptr->lite_table_rd_bus_addr) >> 32, bk_ptr->bar[0] + DESC_CTRLLER_BASE + ALTERA_LITE_DMA_RD_RC_HIGH_SRC_ADDR);

  // DMA制御レジスタの設定
  // (ディスクリプタの最終IDがALL1 => 始めの1回のみ設定)
  if (last_id == 0xFF)
  {
    iowrite32(RD_CTRL_BUF_BASE_LOW, bk_ptr->bar[0] + DESC_CTRLLER_BASE + ALTERA_LITE_DMA_RD_CTLR_LOW_DEST_ADDR);
    iowrite32(RD_CTRL_BUF_BASE_HI, bk_ptr->bar[0] + DESC_CTRLLER_BASE + ALTERA_LITE_DMA_RD_CTRL_HIGH_DEST_ADDR);
  }

  wmb();

  if (last_id == 0xFF)
    last_id = 127;

  last_id = last_id + bk_ptr->dma_status.altera_dma_descriptor_num;

  if (last_id > 127)
  {
    last_id = last_id - 128;
    if ((bk_ptr->dma_status.altera_dma_descriptor_num > 1) && (last_id != 127))
      write_127 = 1;
  }

  if (write_127)
    iowrite32(127, bk_ptr->bar[0] + DESC_CTRLLER_BASE + ALTERA_LITE_DMA_RD_LAST_PTR);

  iowrite32(last_id, bk_ptr->bar[0] + DESC_CTRLLER_BASE + ALTERA_LITE_DMA_RD_LAST_PTR);

  // 転送完了待ち
  while (1)
  {

    if (bk_ptr->lite_table_rd_cpu_virt_addr->header.flags[last_id])
    {
      break;
    }

    if (timeout == 0)
    {
      printk(KERN_DEBUG "Read DMA times out\n");
      bk_ptr->dma_status.read_eplast_timeout = 1;
      printk(KERN_DEBUG "DWORD = %08x\n", bk_ptr->dma_status.altera_dma_num_dwords);
      printk(KERN_DEBUG "Desc = %08x\n", bk_ptr->dma_status.altera_dma_descriptor_num);
      break;
    }
    timeout--;
    cpu_relax();
  }

  // タイムアウト処理
  if (timeout == 0)
  {
    bk_ptr->dma_status.pass_read = 0;
    rc = -ETIME;
  }
  else
  {
    bk_ptr->dma_status.pass_read = 1;
  }

  atomic_set(&bk_ptr->status, 0);
  return rc;
}


int altera_dma_open(struct inode *inode, struct file *file)
{
  struct altera_pcie_dma_bookkeep *bk_ptr = 0;

  bk_ptr = container_of(inode->i_cdev, struct altera_pcie_dma_bookkeep, cdev);
  file->private_data = bk_ptr;
  bk_ptr->user_pid = current->pid;

  return 0;
}

int altera_dma_release(struct inode *inode, struct file *file)
{
  return 0;
}

// 割り込みハンドラ
static irqreturn_t dma_isr(int irq, void *dev_id)
{

  int irq_sts = 0;

  struct altera_pcie_dma_bookkeep *bk_ptr = dev_id;

  // 割り込みステータスリード
  irq_sts = ioread32((u32 *)(bk_ptr->bar[BAR] + BAR2_PCIEXTREG_INTCR));
  printk(KERN_DEBUG "IRQ_STS = %d\n", irq_sts);

  // スタータス検出時のみ割り込みクリア
  if (irq_sts == 1)
  {

    printk(KERN_DEBUG "Device IRQ Detect!!!\n");

    // 割り込みクリア
    iowrite32(cpu_to_le32(1), (u32 *)(bk_ptr->bar[BAR] + BAR2_PCIEXTREG_INTCR));

    fpga_ack = 1;
  }

  wake_up(&bk_ptr->wait_q);

  return IRQ_HANDLED;
}

struct file_operations altera_dma_fops = {
    .owner = THIS_MODULE,
    .read = altera_dma_read,
    .write = (void *)altera_dma_write,
    .open = altera_dma_open,
    .release = altera_dma_release,
    .unlocked_ioctl = altera_dma_ioctl,
};

static int __init init_chrdev(struct altera_pcie_dma_bookkeep *bk_ptr)
{
  int dev_minor = 0;
  int dev_major = 0;
  int devno = -1;

  int result = alloc_chrdev_region(&bk_ptr->cdevno, dev_minor, 1, ALTERA_DMA_DEVFILE);

  dev_major = MAJOR(bk_ptr->cdevno);
  if (result < 0)
  {
    printk(KERN_DEBUG "cannot get major ID %d", dev_major);
  }

  devno = MKDEV(dev_major, dev_minor);

  cdev_init(&bk_ptr->cdev, &altera_dma_fops);
  bk_ptr->cdev.owner = THIS_MODULE;
  bk_ptr->cdev.ops = &altera_dma_fops;
  result = cdev_add(&bk_ptr->cdev, devno, 1);

  if (result)
    return -1;
  return 0;
}

static int set_read_desc(struct dma_descriptor *rd_desc, dma_addr_t source, u64 dest, u32 ctl_dma_len, u32 id)
{
  rd_desc->src_addr_ldw = cpu_to_le32(source & 0xffffffffUL);
  rd_desc->src_addr_udw = cpu_to_le32((source >> 32));
  rd_desc->dest_addr_ldw = cpu_to_le32(dest & 0xffffffffUL);
  rd_desc->dest_addr_udw = cpu_to_le32((dest >> 32));
  rd_desc->ctl_dma_len = cpu_to_le32(ctl_dma_len | (id << 18));
  rd_desc->reserved[0] = cpu_to_le32(0x0);
  rd_desc->reserved[1] = cpu_to_le32(0x0);
  rd_desc->reserved[2] = cpu_to_le32(0x0);
  return 0;
}

/*
   static int print_desc(struct dma_descriptor *desc)
   {

   printk(KERN_DEBUG "Print Desc"                   );
   printk(KERN_DEBUG "0x%x\n",    *(u32*)desc       );
   printk(KERN_DEBUG "0x%x\n",    *((u32*)desc+0x1) );
   printk(KERN_DEBUG "0x%x\n",    *((u32*)desc+0x2) );
   printk(KERN_DEBUG "0x%x\n",    *((u32*)desc+0x3) );
   printk(KERN_DEBUG "0x%x\n",    *((u32*)desc+0x4) );
   printk(KERN_DEBUG "0x%x\n",    *((u32*)desc+0x5) );
   printk(KERN_DEBUG "0x%x\n",    *((u32*)desc+0x6) );
   printk(KERN_DEBUG "0x%x\n",    *((u32*)desc+0x7) );
   return 0;
   }*/

static int set_write_desc(struct dma_descriptor *wr_desc, u64 source, dma_addr_t dest, u32 ctl_dma_len, u32 id)
{
  wr_desc->src_addr_ldw = cpu_to_le32(source & 0xffffffffUL);
  wr_desc->src_addr_udw = cpu_to_le32((source >> 32));
  wr_desc->dest_addr_ldw = cpu_to_le32(dest & 0xffffffffUL);
  wr_desc->dest_addr_udw = cpu_to_le32((dest >> 32));
  wr_desc->ctl_dma_len = cpu_to_le32(ctl_dma_len | (id << 18));
  wr_desc->reserved[0] = cpu_to_le32(0x0);
  wr_desc->reserved[1] = cpu_to_le32(0x0);
  wr_desc->reserved[2] = cpu_to_le32(0x0);
  return 0;
}

static int scan_bars(struct altera_pcie_dma_bookkeep *bk_ptr, struct pci_dev *dev)
{
  int i;
  for (i = 0; i < ALTERA_DMA_BAR_NUM; i++)
  {
    unsigned long bar_start = pci_resource_start(dev, i);
    unsigned long bar_end = pci_resource_end(dev, i);
    unsigned long bar_flags = pci_resource_flags(dev, i);
    bk_ptr->bar_length[i] = pci_resource_len(dev, i);
    dev_info(&dev->dev, "BAR[%d] 0x%08lx-0x%08lx flags 0x%08lx, length %d", i, bar_start, bar_end, bar_flags, (int)bk_ptr->bar_length[i]);
  }
  return 0;
}

static int init_rp_mem(u8 *rp_buffer_virt_addr, u32 num_dwords, u32 init_value, u8 increment)
{
  u32 i = 0;
  u32 increment_value = 0;
  u32 tmp_rand;
  for (i = 0; i < num_dwords; i++)
  {
    get_random_bytes(&tmp_rand, sizeof(tmp_rand));
    *((u32 *)rp_buffer_virt_addr + i) = cpu_to_le32(tmp_rand);
  }
  return 0;
}

static int set_lite_table_header(struct lite_dma_header *header)
{
  int i;
  for (i = 0; i < 128; i++)
    header->flags[i] = cpu_to_le32(0x0);
  return 0;
}

static int __init map_bars(struct altera_pcie_dma_bookkeep *bk_ptr, struct pci_dev *dev)
{
  int i;
  for (i = 0; i < ALTERA_DMA_BAR_NUM; i++)
  {
    unsigned long bar_start = pci_resource_start(dev, i);
    //unsigned long bar_end = pci_resource_end(dev, i);
    //unsigned long bar_flags = pci_resource_flags(dev, i);
    bk_ptr->bar_length[i] = pci_resource_len(dev, i);
    if (!bk_ptr->bar_length[i])
    {
      bk_ptr->bar[i] = NULL;
      continue;
    }
    bk_ptr->bar[i] = ioremap(bar_start, bk_ptr->bar_length[i]);
    if (!bk_ptr->bar[i])
    {
      dev_err(&dev->dev, "could not map BAR[%d]", i);
      return -1;
    }
    else
      dev_info(&dev->dev, "BAR[%d] mapped to 0x%p, length %lu", i, bk_ptr->bar[i], (long unsigned int)bk_ptr->bar_length[i]);
  }
  return 0;
}

static void unmap_bars(struct altera_pcie_dma_bookkeep *bk_ptr, struct pci_dev *dev)
{
  int i;
  for (i = 0; i < ALTERA_DMA_BAR_NUM; i++)
  {
    if (bk_ptr->bar[i])
    {
      pci_iounmap(dev, bk_ptr->bar[i]);
      bk_ptr->bar[i] = NULL;
    }
  }
}
static int __init altera_pci_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
  int rc = 0;
  struct altera_pcie_dma_bookkeep *bk_ptr = NULL;

  bk_ptr = kzalloc(sizeof(struct altera_pcie_dma_bookkeep), GFP_KERNEL);
  if (!bk_ptr)
    goto err_bk_alloc;

  bk_ptr->pci_dev = dev;
  pci_set_drvdata(dev, bk_ptr);

  rc = init_chrdev(bk_ptr);
  if (rc)
  {
    dev_err(&dev->dev, "init_chrdev() failed\n");
    goto err_initchrdev;
  }
  rc = pci_enable_device(dev);
  if (rc)
  {
    dev_err(&dev->dev, "pci_enable_device() failed\n");
    goto err_enable;
  }
  else
  {
    dev_info(&dev->dev, "pci_enable_device() successful");
  }
  rc = pci_request_regions(dev, ALTERA_DMA_DRIVER_NAME);
  if (rc)
  {
    dev_err(&dev->dev, "pci_request_regions() failed\n");
    goto err_regions;
  }
  pci_set_master(dev);
  rc = pci_enable_msi(dev);
  if (rc)
  {
    dev_info(&dev->dev, "pci_enable_msi() failed\n");
    bk_ptr->msi_enabled = 0;
  }
  else
  {
    dev_info(&dev->dev, "pci_enable_msi() successful\n");
    bk_ptr->msi_enabled = 1;
  }

  // レガシー割り込みの為、MSI割り込みを解除
  pci_disable_msi(dev);
  dev_info(&dev->dev, "pci_disable_msi() successful\n");
  bk_ptr->msi_enabled = 0;

  pci_read_config_byte(dev, PCI_REVISION_ID, &bk_ptr->revision);
  pci_read_config_byte(dev, PCI_INTERRUPT_PIN, &bk_ptr->irq_pin);
  pci_read_config_byte(dev, PCI_INTERRUPT_LINE, &bk_ptr->irq_line);

  if (!pci_set_dma_mask(dev, DMA_BIT_MASK(DMAMASK)))
  {
    pci_set_consistent_dma_mask(dev, DMA_BIT_MASK(DMAMASK));
    dev_info(&dev->dev, "using a 64-bit irq mask\n");
  }
  else
  {
    dev_info(&dev->dev, "unable to use 64-bit irq mask\n");
    goto err_dma_mask;
  }

  dev_info(&dev->dev, "irq pin: %d\n", bk_ptr->irq_pin);
  dev_info(&dev->dev, "irq line: %d\n", bk_ptr->irq_line);
  dev_info(&dev->dev, "irq: %d\n", dev->irq);

  // レガシー割り込みの場合はIRQ番号で割り込みハンドリング
  //  rc = request_irq(bk_ptr->irq_line, dma_isr, IRQF_SHARED, ALTERA_DMA_DRIVER_NAME, (void *)bk_ptr);
  rc = request_irq(dev->irq, dma_isr, IRQF_SHARED, ALTERA_DMA_DRIVER_NAME, (void *)bk_ptr);

  if (rc)
  {
    dev_info(&dev->dev, "Could not request IRQ #%d", bk_ptr->irq_line);
    bk_ptr->irq_line = -1;
    goto err_irq;
  }
  else
  {
    dev_info(&dev->dev, "request irq: %d", bk_ptr->irq_line);
  }

  do_gettimeofday(&trg_tv1);

  scan_bars(bk_ptr, dev);
  map_bars(bk_ptr, dev);

  // waitqueue for user process
  init_waitqueue_head(&bk_ptr->wait_q);

  // set default settings to run
  bk_ptr->dma_status.altera_dma_num_dwords = ALTERA_DMA_NUM_DWORDS;
  bk_ptr->dma_status.altera_dma_descriptor_num = ALTERA_DMA_DESCRIPTOR_NUM;
  bk_ptr->dma_status.run_write = 1;
  bk_ptr->dma_status.run_read = 1;
  bk_ptr->dma_status.run_simul = 1;
  bk_ptr->dma_status.offset = 0;
  bk_ptr->dma_status.onchip = 1;
  bk_ptr->dma_status.rand = 0;
  bk_ptr->table_rd_cpu_virt_addr = ((struct dma_desc_table *)pci_alloc_consistent(dev, sizeof(struct dma_desc_table), &bk_ptr->table_rd_bus_addr));
  bk_ptr->lite_table_rd_cpu_virt_addr = ((struct lite_dma_desc_table *)pci_alloc_consistent(dev, sizeof(struct lite_dma_desc_table), &bk_ptr->lite_table_rd_bus_addr));

  if (!bk_ptr->table_rd_cpu_virt_addr || !bk_ptr->lite_table_rd_cpu_virt_addr)
  {
    rc = -ENOMEM;
    goto err_rd_table;
  }
  bk_ptr->table_wr_cpu_virt_addr = ((struct dma_desc_table *)pci_alloc_consistent(dev, sizeof(struct dma_desc_table), &bk_ptr->table_wr_bus_addr));
  bk_ptr->lite_table_wr_cpu_virt_addr = ((struct lite_dma_desc_table *)pci_alloc_consistent(dev, sizeof(struct lite_dma_desc_table), &bk_ptr->lite_table_wr_bus_addr));

  if (!bk_ptr->table_wr_cpu_virt_addr || !bk_ptr->lite_table_wr_cpu_virt_addr)
  {
    rc = -ENOMEM;
    goto err_wr_table;
  }
  bk_ptr->numpages = (PAGE_SIZE >= MAX_NUM_DWORDS * 4) ? 1 : (int)((MAX_NUM_DWORDS * 4) / PAGE_SIZE);
  bk_ptr->rp_rd_buffer_virt_addr = pci_alloc_consistent(dev, PAGE_SIZE * bk_ptr->numpages, &bk_ptr->rp_rd_buffer_bus_addr);
  if (!bk_ptr->rp_rd_buffer_virt_addr)
  {
    rc = -ENOMEM;
    goto err_rd_buffer;
  }
  bk_ptr->rp_wr_buffer_virt_addr = pci_alloc_consistent(dev, PAGE_SIZE * bk_ptr->numpages, &bk_ptr->rp_wr_buffer_bus_addr);
  if (!bk_ptr->rp_wr_buffer_virt_addr)
  {
    rc = -ENOMEM;
    goto err_wr_buffer;
  }
  return 0;

  // error clean up
err_wr_buffer:
  dev_err(&dev->dev, "goto err_wr_buffer");
  pci_free_consistent(dev, PAGE_SIZE * bk_ptr->numpages, bk_ptr->rp_rd_buffer_virt_addr, bk_ptr->rp_rd_buffer_bus_addr);
err_rd_buffer:
  dev_err(&dev->dev, "goto err_rd_buffer");
  pci_free_consistent(dev, sizeof(struct dma_desc_table), bk_ptr->table_wr_cpu_virt_addr, bk_ptr->table_wr_bus_addr);
err_wr_table:
  dev_err(&dev->dev, "goto err_wr_table");
  pci_free_consistent(dev, sizeof(struct dma_desc_table), bk_ptr->table_rd_cpu_virt_addr, bk_ptr->table_rd_bus_addr);
err_rd_table:
  dev_err(&dev->dev, "goto err_rd_table");
err_irq:
  dev_err(&dev->dev, "goto err_regions");
err_dma_mask:
  dev_err(&dev->dev, "goto err_dma_mask");
  pci_release_regions(dev);
err_regions:
  dev_err(&dev->dev, "goto err_irq");
  pci_disable_device(dev);
err_enable:
  dev_err(&dev->dev, "goto err_enable");
  unregister_chrdev_region(bk_ptr->cdevno, 1);
err_initchrdev:
  dev_err(&dev->dev, "goto err_initchrdev");
  kfree(bk_ptr);
err_bk_alloc:
  dev_err(&dev->dev, "goto err_bk_alloc");
  return rc;
}


static void __exit altera_pci_remove(struct pci_dev *dev)
{
  struct altera_pcie_dma_bookkeep *bk_ptr = NULL;
  bk_ptr = pci_get_drvdata(dev);
  cdev_del(&bk_ptr->cdev);
  unregister_chrdev_region(bk_ptr->cdevno, 1);
  pci_disable_device(dev);
  if (bk_ptr)
  {
    if (bk_ptr->msi_enabled)
    {
      pci_disable_msi(dev);
      bk_ptr->msi_enabled = 0;
    }
  }
  unmap_bars(bk_ptr, dev);
  pci_release_regions(dev);
  if (dev->irq >= 0)
  {
    printk(KERN_DEBUG "Freeing IRQ #%d", dev->irq);
    free_irq(dev->irq, (void *)bk_ptr);
  }
  pci_free_consistent(dev, sizeof(struct dma_desc_table), bk_ptr->table_rd_cpu_virt_addr, bk_ptr->table_rd_bus_addr);
  pci_free_consistent(dev, sizeof(struct lite_dma_desc_table), bk_ptr->lite_table_rd_cpu_virt_addr, bk_ptr->lite_table_rd_bus_addr);
  pci_free_consistent(dev, sizeof(struct dma_desc_table), bk_ptr->table_wr_cpu_virt_addr, bk_ptr->table_wr_bus_addr);
  pci_free_consistent(dev, sizeof(struct lite_dma_desc_table), bk_ptr->lite_table_wr_cpu_virt_addr, bk_ptr->lite_table_wr_bus_addr);
  pci_free_consistent(dev, PAGE_SIZE * bk_ptr->numpages, bk_ptr->rp_rd_buffer_virt_addr, bk_ptr->rp_rd_buffer_bus_addr);
  pci_free_consistent(dev, PAGE_SIZE * bk_ptr->numpages, bk_ptr->rp_wr_buffer_virt_addr, bk_ptr->rp_wr_buffer_bus_addr);

  kfree(bk_ptr);
  //    printk(KERN_DEBUG ALTERA_DMA_DRIVER_NAME ": " "altera_dma_remove()," " " __DATE__ " " __TIME__ " " "\n");
  printk(KERN_DEBUG ALTERA_DMA_DRIVER_NAME ": "
                                           "altera_dma_remove(),"
                                           " "
                                           " "
                                           " "
                                           "\n");
}

static struct pci_device_id pci_ids[] = {
    {PCI_DEVICE(ALTERA_DMA_VID, ALTERA_DMA_DID)},
    {0}};

static struct pci_driver dma_driver_ops = {
    .name = ALTERA_DMA_DRIVER_NAME,
    .id_table = pci_ids,
    .probe = altera_pci_probe,
    .remove = altera_pci_remove,
};

static int __init altera_dma_init(void)
{
  int rc = 0;

  //  printk(KERN_DEBUG ALTERA_DMA_DRIVER_NAME ": " "altera_dma_init()," " " __DATE__ " " __TIME__ " " "\n");
  printk(KERN_DEBUG ALTERA_DMA_DRIVER_NAME ": "
                                           "altera_dma_init(),"
                                           " "
                                           " "
                                           " "
                                           "\n");
  rc = pci_register_driver(&dma_driver_ops);
  if (rc)
  {
    printk(KERN_ERR ALTERA_DMA_DRIVER_NAME ": PCI driver registration failed\n");
    goto exit;
  }

exit:
  return rc;
}

static void __exit altera_dma_exit(void)
{
  pci_unregister_driver(&dma_driver_ops);
}

static int diff_timeval(struct timeval *result, struct timeval *t2, struct timeval *t1)
{
  long int diff = (t2->tv_usec + 1000000 * t2->tv_sec) - (t1->tv_usec + 1000000 * t1->tv_sec);
  result->tv_sec = diff / 1000000;
  result->tv_usec = diff % 1000000;
  return (diff < 0);
}

module_init(altera_dma_init);
module_exit(altera_dma_exit);

MODULE_DESCRIPTION("EdgeAI PCIe Driver");
MODULE_VERSION(ALTERA_DMA_DRIVER_VERSION);
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DEVICE_TABLE(pci, pci_ids);


void do_gettimeofday(struct timeval *tv)
{
  struct timespec64 ts;
  ktime_get_real_ts64(&ts);
  tv->tv_sec = ts.tv_sec;
  tv->tv_usec = ts.tv_nsec / 1000;
}
