#ifndef __SUNXI_NAND_H__
#define __SUNXI_NAND_H__

#define U_BOOT_NAND		(1)

#if (U_BOOT_NAND)
#include <common.h>
#include <asm/errno.h>
#include <asm/io.h>
#include <malloc.h>
#include <linux/mtd/nand.h>
#include <nand.h>
#include <asm-generic/gpio.h>
#include <dm/device.h>
#else
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_mtd.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/version.h>
#include <mach/sys_config.h>
#include <mach/nand_platform.h>
#endif

#ifndef GENMASK
#define GENMASK(a, b)	(((1 << (a-b + 1)) - 1) << b)
#endif

#define NFC_REG_CTL		0x0000
#define NFC_REG_ST		0x0004
#define NFC_REG_INT		0x0008
#define NFC_REG_TIMING_CTL	0x000C
#define NFC_REG_TIMING_CFG	0x0010
#define NFC_REG_ADDR_LOW	0x0014
#define NFC_REG_ADDR_HIGH	0x0018
#define NFC_REG_SECTOR_NUM	0x001C
#define NFC_REG_CNT		0x0020
#define NFC_REG_CMD		0x0024
#define NFC_REG_RCMD_SET	0x0028
#define NFC_REG_WCMD_SET	0x002C
#define NFC_REG_IO_DATA		0x0030
#define NFC_REG_ECC_CTL		0x0034
#define NFC_REG_ECC_ST		0x0038
#define NFC_REG_DEBUG		0x003C
#define NFC_REG_ECC_ERR_CNT(x)	((0x0040 + (x)) & ~0x3)
#define NFC_REG_USER_DATA(x)	(0x0050 + ((x) * 4))
#define NFC_REG_SPARE_AREA	0x00A0
#define NFC_RAM0_BASE		0x0400
#define NFC_RAM1_BASE		0x0800

/* define bit use in NFC_CTL */
#define NFC_EN			BIT(0)
#define NFC_RESET		BIT(1)
#define NFC_BUS_WIDTH_MSK	BIT(2)
#define NFC_BUS_WIDTH_8		(0 << 2)
#define NFC_BUS_WIDTH_16	(1 << 2)
#define NFC_RB_SEL_MSK		BIT(3)
#define NFC_RB_SEL(x)		((x) << 3)
#define NFC_CE_SEL_MSK		GENMASK(26, 24)
#define NFC_CE_SEL(x)		((x) << 24)
#define NFC_CE_CTL		BIT(6)
#define NFC_PAGE_SHIFT_MSK	GENMASK(11, 8)
#define NFC_PAGE_SHIFT(x)	(((x) < 10 ? 0 : (x) - 10) << 8)
#define NFC_SAM			BIT(12)
#define NFC_RAM_METHOD		BIT(14)
#define NFC_DEBUG_CTL		BIT(31)

/* define bit use in NFC_ST */
#define NFC_RB_B2R		BIT(0)
#define NFC_CMD_INT_FLAG	BIT(1)
#define NFC_DMA_INT_FLAG	BIT(2)
#define NFC_CMD_FIFO_STATUS	BIT(3)
#define NFC_STA			BIT(4)
#define NFC_NATCH_INT_FLAG	BIT(5)
#define NFC_RB_STATE(x)		BIT(x + 8)

/* define bit use in NFC_INT */
#define NFC_B2R_INT_ENABLE	BIT(0)
#define NFC_CMD_INT_ENABLE	BIT(1)
#define NFC_DMA_INT_ENABLE	BIT(2)
#define NFC_INT_MASK		(NFC_B2R_INT_ENABLE | \
				 NFC_CMD_INT_ENABLE | \
				 NFC_DMA_INT_ENABLE)

/* define bit use in NFC_TIMING_CTL */
#define NFC_TIMING_CTL_EDO	BIT(8)

/* define NFC_TIMING_CFG register layout */
#define NFC_TIMING_CFG(tWB, tADL, tWHR, tRHW, tCAD)		\
	(((tWB) & 0x3) | (((tADL) & 0x3) << 2) |		\
	(((tWHR) & 0x3) << 4) | (((tRHW) & 0x3) << 6) |		\
	(((tCAD) & 0x7) << 8))

/* define bit use in NFC_CMD */
#define NFC_CMD_LOW_BYTE_MSK	GENMASK(7, 0)
#define NFC_CMD_HIGH_BYTE_MSK	GENMASK(15, 8)
#define NFC_CMD(x)		(x)
#define NFC_ADR_NUM_MSK		GENMASK(18, 16)
#define NFC_ADR_NUM(x)		(((x) - 1) << 16)
#define NFC_SEND_ADR		BIT(19)
#define NFC_ACCESS_DIR		BIT(20)
#define NFC_DATA_TRANS		BIT(21)
#define NFC_SEND_CMD1		BIT(22)
#define NFC_WAIT_FLAG		BIT(23)
#define NFC_SEND_CMD2		BIT(24)
#define NFC_SEQ			BIT(25)
#define NFC_DATA_SWAP_METHOD	BIT(26)
#define NFC_ROW_AUTO_INC	BIT(27)
#define NFC_SEND_CMD3		BIT(28)
#define NFC_SEND_CMD4		BIT(29)
#define NFC_CMD_TYPE_MSK	GENMASK(31, 30)
#define NFC_NORMAL_OP		(0 << 30)
#define NFC_ECC_OP		(1 << 30)
#define NFC_PAGE_OP		(2 << 30)

/* define bit use in NFC_RCMD_SET */
#define NFC_READ_CMD_MSK	GENMASK(7, 0)
#define NFC_RND_READ_CMD0_MSK	GENMASK(15, 8)
#define NFC_RND_READ_CMD1_MSK	GENMASK(23, 16)

/* define bit use in NFC_WCMD_SET */
#define NFC_PROGRAM_CMD_MSK	GENMASK(7, 0)
#define NFC_RND_WRITE_CMD_MSK	GENMASK(15, 8)
#define NFC_READ_CMD0_MSK	GENMASK(23, 16)
#define NFC_READ_CMD1_MSK	GENMASK(31, 24)

/* define bit use in NFC_ECC_CTL */
#define NFC_ECC_EN		BIT(0)
#define NFC_ECC_PIPELINE	BIT(3)
#define NFC_ECC_EXCEPTION	BIT(4)
#define NFC_ECC_BLOCK_SIZE_MSK	BIT(5)
#define NFC_RANDOM_EN		BIT(9)
#define NFC_RANDOM_DIRECTION	BIT(10)
#define NFC_ECC_MODE_MSK	GENMASK(15, 12)
#define NFC_ECC_MODE(x)		((x) << 12)
#define NFC_RANDOM_SEED_MSK	GENMASK(30, 16)
#define NFC_RANDOM_SEED(x)	((x) << 16)

/* define bit use in NFC_ECC_ST */
#define NFC_ECC_ERR(x)		BIT(x)
#define NFC_ECC_PAT_FOUND(x)	BIT(x + 16)
#define NFC_ECC_ERR_CNT(b, x)	(((x) >> ((b) * 8)) & 0xff)

#define NFC_DEFAULT_TIMEOUT_MS	1000//0000

#define NFC_SRAM_SIZE		1024

/*
*	DEBUGGING FEATURES
*
*/

#define DEBUG	0

#if DEBUG
#define NAND_DBG(msg...)		{ printk("sunxi-nand: " msg); }
#else
#define NAND_DBG(msg...)		do {} while (0)
#endif

#if (U_BOOT_NAND)
#define STRUCT_NANDCHIP_HAVE_ONFI_TIMING	0
#define NAND_SDR_TIMINGS_DEFINED			1
#else
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 0, 0))
#define STRUCT_NANDCHIP_HAVE_ONFI_TIMING	1
#define NAND_SDR_TIMINGS_DEFINED			1
#else
#define STRUCT_NANDCHIP_HAVE_ONFI_TIMING	0
#define NAND_SDR_TIMINGS_DEFINED			0
#endif
#endif

#define NFC_MAX_CS		7
#define MAX_RETRIES 10

/*
 * Ready/Busy detection type: describes the Ready/Busy detection modes
 *
 * @RB_NONE:	no external detection available, rely on STATUS command
 *		and software timeouts
 * @RB_NATIVE:	use sunxi NAND controller Ready/Busy support. The Ready/Busy
 *		pin of the NAND flash chip must be connected to one of the
 *		native NAND R/B pins (those which can be muxed to the NAND
 *		Controller)
 * @RB_GPIO:	use a simple GPIO to handle Ready/Busy status. The Ready/Busy
 *		pin of the NAND flash chip must be connected to a GPIO capable
 *		pin.
 */
enum sunxi_nand_csrb_type {
	CSRB_NONE,
	CSRB_NATIVE,
	CSRB_GPIO,
};

/*
 * Ready/Busy structure: stores information related to Ready/Busy detection
 *
 * @type:	the Ready/Busy detection mode
 * @info:	information related to the R/B detection mode. Either a gpio
 *		id or a native R/B id (those supported by the NAND controller).
 */
struct sunxi_nand_pin {
	enum sunxi_nand_csrb_type type;
	union {
		int gpio;
		int nativeid;
	} info;
	int gpio_port;
	int gpio_pin;
	int gpio_state;
};

/*
 * Chip Select structure: stores information related to NAND Chip Select
 *
 * @cs:		the NAND CS id used to communicate with a NAND Chip
 * @rb:		the Ready/Busy description
 */
struct sunxi_nand_chip_sel {
	struct sunxi_nand_pin cs;
	struct sunxi_nand_pin rb;
};

/*
 * sunxi HW ECC infos: stores information related to HW ECC support
 *
 * @mode:	the sunxi ECC mode field deduced from ECC requirements
 * @layout:	the OOB layout depending on the ECC requirements and the
 *		selected ECC mode
 */
struct sunxi_nand_hw_ecc {
	int mode;
	struct nand_ecclayout layout;
};

struct sunxi_pin_struct {
	int nsels;
	struct sunxi_nand_chip_sel sels[8];
};

/*
 * NAND Controller structure: stores sunxi NAND controller information
 *
 * @controller:		base controller structure
 * @dev:		parent device (used to print error messages)
 * @regs:		NAND controller registers
 * @ahb_clk:		NAND Controller AHB clock
 * @mod_clk:		NAND Controller mod clock
 * @assigned_cs:	bitmask describing already assigned CS lines
 * @clk_rate:		NAND controller current clock rate
 * @chips:		a list containing all the NAND chips attached to
 *			this NAND controller
 * @complete:		a completion object used to wait for NAND
 *			controller events
 */
struct sunxi_nand_chip;

struct sunxi_nfc {
	int irq;
	void __iomem *regs;
	struct clk *ahb_clk;
	struct clk *mod_clk;
	unsigned long assigned_cs;
	unsigned long clk_rate;
	/* ksw : Why would I bother with list head? */
	int current_chip;
	int num_of_chips;
	struct sunxi_nand_chip *chips[NFC_MAX_CS];
	//struct list_head chips;
#if !(U_BOOT_NAND)
	struct completion complete;
#endif
	int nres;
	struct sunxi_pin_struct *pins;	/* This is temporary structure to initialize the chips */
};

/*
 * NAND chip structure: stores NAND chip device related information
 *
 * @node:		used to store NAND chips into a list
 * @nand:		base NAND chip structure
 * @mtd:		base MTD structure
 * @clk_rate:		clk_rate required for this NAND chip
 * @timing_cfg		TIMING_CFG register value for this NAND chip
 * @selected:		current active CS
 * @nsels:		number of CS lines required by the NAND chip
 * @sels:		array of CS lines descriptions
 */
struct sunxi_nand_chip {
	//struct list_head node;
	struct nand_chip *nand;
	struct mtd_info  *mtd;
#if !STRUCT_NANDCHIP_HAVE_ONFI_TIMING
	int onfi_timing_mode_default;
	//int onfi_set_features;
	int ecc_step_ds;
	int ecc_strength_ds;
#endif
	unsigned long clk_rate;
	u32 timing_cfg;
	u32 timing_ctl;
	struct sunxi_nand_chip_sel chip_sel;
};





#define DRV_NAME	"sunxi-nand"

extern int sunxi_nfc_rst(struct sunxi_nfc *nfc);
extern irqreturn_t sunxi_nfc_interrupt(int irq, void *dev_id);
extern void sunxi_nfc_cmd_ctrl(struct mtd_info *mtd, int dat, unsigned int ctrl);
extern int sunxi_nfc_dev_ready(struct mtd_info *mtd);
extern void sunxi_nfc_select_chip(struct mtd_info *mtd, int chip);
extern void sunxi_nfc_read_buf(struct mtd_info *mtd, uint8_t *buf, int len);
extern void sunxi_nfc_write_buf(struct mtd_info *mtd, const uint8_t *buf, int len);
extern uint8_t sunxi_nfc_read_byte(struct mtd_info *mtd);
extern int sunxi_nand_ecc_init(struct mtd_info *mtd, struct nand_ecc_ctrl *ecc);
extern int sunxi_nand_hw_ecc_ctrl_init(struct mtd_info *mtd, struct nand_ecc_ctrl *ecc);
extern void nand_clock_setup(void);
extern uint8_t sunxi_nfc_read_byte(struct mtd_info *mtd);
extern int sunxi_nand_chip_init_timings(struct sunxi_nand_chip *chip);
#endif