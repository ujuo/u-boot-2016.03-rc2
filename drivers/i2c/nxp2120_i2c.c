/*
 * i2c driver for nxp2120 chip.
 *
 * Copyright(c) 2011 MOSTiTECH co.,ltd.
 * All right reserved. By Seungwoo Kim <ksw@mostitech.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>

#if defined(CONFIG_HARD_I2C) && defined (CONFIG_NXP2120_I2C)

#include <asm/arch/regs.h>

#ifdef CFG_I2C_NXP2120_PORT1
#define I2C_BASE	PHY_BASEADDR_I2C0_MODULE
#define GPIOB_MASK  ((0x3 << 8) | (0x3 << 10))
#define GPIOB_SET   ((0x1 << 8) | (0x1 << 10))
#elif defined (CFG_I2C_NXP2120_PORT2)
#define I2C_BASE	PHY_BASEADDR_I2C1_MODULE
#define GPIOB_MASK  ((0x3 << 12) | (0x3 << 14))
#define GPIOB_SET   ((0x1 << 12) | (0x1 << 14))
#else
#error "define CFG_I2C_NXP2120_PORTx to use the NXP2120 I2C driver"
#endif

//#define DEBUG

#ifdef DEBUG
#define DPRINTF(args...)  printf(args)
#else
#define DPRINTF(args...)
#endif

#define TX_SETUP    100
#define SEND_COMPLETE   1
#define I2C_M_RD        1
#define I2C_M_NOSTART   2
#define	ACK_WAIT_TIMEOUT	(50)	/* wait 50 msec */

int i2c_msg_ptr;
int i2c_msg_len, i2c_flag;
uchar *i2c_msg_buf;
int i2c_cond, i2c_prev_data, i2c_nostop;

enum nxp2120_i2c_state {
	STATE_IDLE,
	STATE_START,
	STATE_READ,
	STATE_WRITE,
	STATE_STOP
};
enum nxp2120_i2c_state i2c_state;

#if 0
extern void WriteIODW(unsigned int *p, unsigned int data);
#else
#define WriteIODW(p, data) (*(volatile unsigned int*)p)=((unsigned int)data)
#endif

static int wait_busy(void)
{
	int timeout = 10000;
	struct NX_I2C_RegisterSet *const i2c = (struct NX_I2C_RegisterSet *)I2C_BASE;

	while (!(i2c->ICSR & NXP2120_ICSR_BUSY) && --timeout)
		udelay(1);

	//WriteIODW(&i2c->PEND , NXP2120_I2C_PEND_OP_HOLD); /* clear interrupt PENDING/OPERATION HOLDING */
	i2c->PEND = NXP2120_I2C_PEND_OP_HOLD;

	return timeout;
}

static inline void nxp2120_i2c_disable_ack(void)
{
    unsigned long tmp;
    struct NX_I2C_RegisterSet *const i2c = (struct NX_I2C_RegisterSet *)I2C_BASE;

    tmp = i2c->ICCR;
    //WriteIODW(&i2c->ICCR , tmp & ~NXP2120_ICCR_ACKGEN);
    i2c->ICCR = tmp & ~NXP2120_ICCR_ACKGEN;
}

static inline void nxp2120_i2c_enable_ack(void)
{
    unsigned long tmp;
    struct NX_I2C_RegisterSet *const i2c = (struct NX_I2C_RegisterSet *)I2C_BASE;

    tmp = i2c->ICCR;
    //WriteIODW(&i2c->ICCR , tmp | NXP2120_ICCR_ACKGEN);
    i2c->ICCR = tmp | NXP2120_ICCR_ACKGEN;
}

#if 0
#define TRUE    1
#define FALSE   0
#define NX_I2C_TXRXMODE_SLAVE_RX	0	///< Slave Receive Mode
#define NX_I2C_TXRXMODE_SLAVE_TX	1	///< Slave Transmit Mode
#define NX_I2C_TXRXMODE_MASTER_RX	2	///< Master Receive Mode
#define NX_I2C_TXRXMODE_MASTER_TX	3	///< Master Transmit Mode

static int wait_i2c_ack(int wait_ack)
{
	int   i;
	int   ret = FALSE;
	struct NX_I2C_RegisterSet *const i2c = (struct NX_I2C_RegisterSet *)I2C_BASE;

	DPRINTF("%s(i2c)\n", __func__);

	/* wait polling */
	{
		for (i = 0; (ACK_WAIT_TIMEOUT * 1000) > i; i++) {
            ret = i2c->PEND & NXP2120_I2C_PEND_PEND ? TRUE : FALSE;
			if (TRUE == ret || i2c_cond)
				break;
			udelay(1);
		}
	}

	if (i2c_cond)
		ret = TRUE;

	/* check arbitration */
	if (i2c->ICSR & NXP2120_ICSR_ARBIT_FAIL) {
		DPRINTF("fail, i2c: arbitration\n");
		ret = FALSE;
	}

	if (FALSE == ret) {
		DPRINTF("fail, i2c: cond %d, pend %d, arbit %d\n", i2c_cond,
			i2c->PEND & NXP2120_I2C_PEND_PEND, i2c->ICSR & NXP2120_ICSR_ARBIT_FAIL);
		goto err_ack;
	}

	if ((TRUE == wait_ack) && (i2c->ICSR & NXP2120_ICSR_ACK_STATUS)) {
		DPRINTF("fail, i2c: receive nack \n");
		ret = FALSE;	/* fail */
	}

err_ack:
	/* clear irq condition */
	i2c_cond = 0;
    i2c->PEND = (i2c->PEND & ~0x03) | NXP2120_I2C_PEND_PEND;

	return ret;
}

static int i2c_message_start(u8 chip)
{
	u_char addr;
	u_int  mode;
	unsigned int ax;
	struct NX_I2C_RegisterSet *const i2c = (struct NX_I2C_RegisterSet *)I2C_BASE;

	addr = (chip << 1);
	printf("chip = %x, addr = %x\n", chip, addr);

	if (i2c_flag & I2C_M_RD) {
		addr += 1;
		mode  = NX_I2C_TXRXMODE_MASTER_RX;
	} else {
		mode  = NX_I2C_TXRXMODE_MASTER_TX;
	}
	ax = addr;
	i2c_cond = 0;
	i2c_prev_data  = addr;

	DPRINTF("%s(i2c: addr:0x%02x, %s)\n", __func__, addr, i2c_flag &I2C_M_RD?"R":"W");

 	/* clear irq condition */

	i2c->PEND = (i2c->PEND & ~0x03) | NXP2120_I2C_PEND_PEND | NXP2120_I2C_PEND_OP_HOLD;
	nxp2120_i2c_disable_ack();

	/* start and trans address byte */
	if (! i2c_nostop)
	    i2c->ICSR = (i2c->ICSR & 0x1F0F) | (NX_I2C_TXRXMODE_SLAVE_RX <<6)  | NXP2120_ICSR_TXRX_ENABLE | NXP2120_ICSR_ST_ENB;
	
    i2c->IDSR =  ax; //addr;

    printf("IDSR = %x\n", i2c->IDSR);
    
    i2c->ICSR =  (i2c->ICSR & 0x1F0F) | (mode<<6) | NXP2120_ICSR_I2C_START | NXP2120_ICSR_TXRX_ENABLE | NXP2120_ICSR_ST_ENB ;

    if(i2c_flag & I2C_M_RD)
        i2c->PEND = (i2c->PEND & ~0x03) | NXP2120_I2C_PEND_OP_HOLD;

	/* wait ack for address */
	if (! wait_i2c_ack(TRUE)) {
		DPRINTF("fail, i2c: start wait ack, addr:0x%02x \n", addr);
		return FALSE;
	}
	return TRUE;
}

static int i2c_message_data(void)
{
	int	  len  = i2c_msg_len;
	int   ack = TRUE;
	int	  i;
	struct NX_I2C_RegisterSet *const i2c = (struct NX_I2C_RegisterSet *)I2C_BASE;

	DPRINTF("%s(i2c: %s, len:%d)\n", __func__, i2c_flag &I2C_M_RD?"R":"W", len);

	/* i2c read */
	if (i2c_flag & I2C_M_RD) {

		for (i = 0; len > i; i++) {

			if ((i + 1) == len)
				ack = FALSE;

            if (FALSE == ack) {
                i2c->ICCR = i2c->ICCR & ~NXP2120_ICCR_ACKGEN;
            } else {
                i2c->ICCR = i2c->ICCR | NXP2120_ICCR_ACKGEN;
            }
            i2c->PEND = i2c->PEND | NXP2120_I2C_PEND_PEND | NXP2120_I2C_PEND_OP_HOLD;

 			if (! wait_i2c_ack(FALSE)) {
 				DPRINTF("fail, i2c: Read  wait ack, len:%d   \n", i);
 				return FALSE;
			}

			/* read byte */
			i2c_msg_buf[i] = i2c->IDSR;
			DPRINTF("(R i2c: %d=0x%02x)\n", i, i2c_msg_buf[i]);
		}

	/* i2c write */
	} else {
		for (i = 0; len > i; i++) {

			/* make SDA high */
			if (!(i2c_prev_data & 0x80) && (i2c_msg_buf[i] & 0x80)) {
			    i2c->IDSR = i2c_msg_buf[i];
				udelay(1);
			}

			/* set previos data */
			i2c_prev_data = i2c_msg_buf[i];

			/* write byte and trans */
			i2c->ICCR = i2c->ICCR & ~NXP2120_ICCR_ACKGEN;
			i2c->IDSR = i2c_msg_buf[i];
			i2c->PEND = i2c->PEND | NXP2120_I2C_PEND_PEND | NXP2120_I2C_PEND_OP_HOLD;

			DPRINTF("(W i2c: %2d=0x%02x)\n", i, i2c_msg_buf[i]);

			if (! wait_i2c_ack(TRUE)) {
				DPRINTF("fail, i2c: Write wait ack, data[%2d]=0x%02x\n", i, i2c_msg_buf[i]);
				for (i = 0; len > i; i++)
					DPRINTF("w data[%2d]=0x%02x\n", i, i2c_msg_buf[i]);
				return FALSE;
			}
		}
		/* make high level */
        i2c->IDSR = 0xFF;
	}
	return TRUE;
}

static void i2c_stop(void)
{
    struct NX_I2C_RegisterSet *const i2c = (struct NX_I2C_RegisterSet *)I2C_BASE;
	u_int mode = i2c_flag & I2C_M_RD ? NX_I2C_TXRXMODE_MASTER_RX : NX_I2C_TXRXMODE_MASTER_TX;

	DPRINTF("%s(i2c: %s, %s)\n",
		__func__, i2c_flag &I2C_M_RD?"R":"W", i2c_flag & I2C_M_NOSTART ?"nostop":"stop");

	if (i2c_flag & I2C_M_RD)
		i2c_flag &= ~I2C_M_NOSTART;

	i2c_nostop = i2c_flag & I2C_M_NOSTART ? TRUE : FALSE;

	/* stop signal */
	if (! i2c_nostop ) {
        i2c->ICSR = (i2c->ICSR & 0x1F0F) | (mode << 6) | NXP2120_ICSR_TXRX_ENABLE | NXP2120_ICSR_ST_ENB;
        i2c->PEND = (i2c->PEND & ~0x03) | NXP2120_I2C_PEND_PEND | NXP2120_I2C_PEND_OP_HOLD;
 	}
}

int i2c_init(int speed, int unused)
{
    struct NX_I2C_RegisterSet *const i2c = (struct NX_I2C_RegisterSet *)I2C_BASE;
    struct NX_GPIO_RegisterSet *const gpiob = (struct NX_GPIO_RegisterSet *)PHY_BASEADDR_GPIO_B_MODULE;
   	//unsigned long iicon = NXP2120_ICCR_IRQENB; /* ksw: There's no IRQ but this bit is needed */

	/* inititalise the gpio */
	gpiob->GPIOxALTFN[0] &= ~GPIOB_MASK;
	gpiob->GPIOxALTFN[0] |= GPIOB_SET;

	/* Enable PCLK for correct operation */
	i2c->CLKENB = i2c->CLKENB | 0x8;

	/* write slave address and enable ack */

	i2c->IAR  = 0; /* ksw : what should I do? */

	//WriteIODW(&i2c->ICCR , iicon | NXP2120_ICCR_CLKSRC_PCLK_DIV_256);
	i2c->ICCR = i2c->ICCR | NXP2120_ICCR_CLKSRC_PCLK_DIV_16 | 15; // iicon | IRQ disable ? 
	/* set clock to 377KHz... */
   
    /* 1/16 prescale and 1/11 would make 377KHz */
    //WriteIODW(&i2c->ICSR , NXP2120_ICSR_TXRX_ENABLE | NXP2120_ICSR_ST_ENB);
    i2c->ICSR = i2c->ICSR | NXP2120_ICSR_TXRX_ENABLE | NXP2120_ICSR_ST_ENB;
    
    /* QcntMax should be set as 2~16 */
    i2c->QCNT_MAX = 2;
    
    /* todo - check that the i2c lines aren't being dragged anywhere */
    //i2c_stop();

    return 0;
}

static int nxp_set_master(void)
{
	unsigned long iicstat;
	struct NX_I2C_RegisterSet *const i2c = (struct NX_I2C_RegisterSet *)I2C_BASE;
	int timeout = 400;

	//i2c_stop();
	while (timeout-- > 0) {
		iicstat = i2c->ICSR;

		if (!(iicstat & NXP2120_ICSR_BUSY))
			return 0;

		udelay(10);
	}

	return -1;
}

int i2c_do_xfer(u8 chip, int read, uchar *buf, int len)
{
    unsigned long x;
    int ret = nxp_set_master();

    if (ret != 0) {
        printf("Error : cannot get bus (error %d)\n", ret);

        return -1;
    }

    i2c_msg_ptr = 0;
    i2c_msg_buf = buf;
    i2c_msg_len = len;
    i2c_flag = 0;
    if (read)
        i2c_flag |= I2C_M_RD;

    ret = -1;
    /* transfer */
	if (! i2c_message_start(chip))
		goto err_i2c;

	if (! i2c_message_data())
		goto err_i2c;

	ret = i2c_msg_len;

err_i2c:

	if (ret != i2c_msg_len)
		i2c_flag &= ~I2C_M_NOSTART;

	i2c_stop();

	if (ret == i2c_msg_len && ! i2c_nostop) {
		wait_busy();
		ret = 0;
	}

    return ret;
}
#else

static void i2c_message_start(u8 chip)
{
    unsigned int addr = (chip & 0x7f) << 1;
	unsigned long stat;
	unsigned long iiccon;
	int i;
	struct NX_I2C_RegisterSet *const i2c = (struct NX_I2C_RegisterSet *)I2C_BASE;

	stat =  NXP2120_ICSR_TXRX_ENABLE | NXP2120_ICSR_ST_ENB;
	

	if (i2c_flag & I2C_M_RD) {
		stat |= NXP2120_ICSR_MASTER;
		addr += 1;
	} else
		stat |= NXP2120_ICSR_MASTER | NXP2120_ICSR_TX;

	/* Clear IRQ Pending... */
	//WriteIODW(&i2c->PEND, NXP2120_I2C_PEND_PEND);
	i2c->PEND= NXP2120_I2C_PEND_PEND | NXP2120_I2C_PEND_OP_HOLD;
	/* todo - check for whether ack wanted or not */
	nxp2120_i2c_disable_ack();

	iiccon = i2c->ICCR;
	stat = i2c->ICSR & 0x1E0F;
	//WriteIODW(&i2c->ICSR , NXP2120_ICSR_TXRX_ENABLE | NXP2120_ICSR_ST_ENB); /* Send Stop */
	i2c->ICSR = stat | NXP2120_ICSR_TXRX_ENABLE | NXP2120_ICSR_ST_ENB;

	//printf("%x %x :", i2c->ICCR, i2c->ICSR);
	DPRINTF("START: %08lx to IICSTAT, %02x to DS\n", stat, addr);
	WriteIODW(&i2c->IDSR, addr);
	//i2c->IDSR = addr;
	//printf("%x ", i2c->IDSR);

	/* delay here to ensure the data byte has gotten onto the bus
	 * before the transaction is started */

	//udelay(TX_SETUP);

	DPRINTF("iiccon, %08lx\n", iiccon);
	//WriteIODW(&i2c->ICCR ,iiccon);
	//i2c->ICCR =iiccon;
	//i2c->PEND |= NXP2120_I2C_PEND_OP_HOLD | NXP2120_I2C_PEND_PEND;
	//printf("PEND = %x\n", i2c->PEND);
	stat = i2c->ICSR;
	stat &= 0x1E0F;
	stat |= NXP2120_ICSR_I2C_START | NXP2120_ICSR_TXRX_ENABLE | NXP2120_ICSR_ST_ENB | NXP2120_ICSR_MASTER | NXP2120_ICSR_TX;
	WriteIODW(&i2c->ICSR,stat);
	//i2c->ICSR = stat;
#if 0
	for (i=0; i<20; i++) {
	    printf("PEND=%x %x\n", i2c->PEND, i2c->ICSR);
	    if (i2c->PEND & NXP2120_I2C_PEND_PEND) {
	        printf("status = %x\n", i2c->ICSR);
	        i2c->PEND |= NXP2120_I2C_PEND_PEND;
	        //WriteIODW(&i2c->IDSR, addr);
	        i2c->IDSR = addr;
	        break;
	    }
	}
#endif
	/* Now SET OP_HOLD = 1 to start transmition */
	i2c->PEND |= NXP2120_I2C_PEND_OP_HOLD;

}

static void i2c_stop(void)
{
    struct NX_I2C_RegisterSet *const i2c = (struct NX_I2C_RegisterSet *)I2C_BASE;
    unsigned long iicstat = i2c->ICSR;
    unsigned long pend;

	DPRINTF("STOP\n");

	/* stop the transfer */
	iicstat &= 0x1F0F;
	//iicstat |= NXP2120_ICSR_TXRX_ENABLE | NXP2120_ICSR_ST_ENB;
	//WriteIODW(&i2c->ICSR , iicstat | NXP2120_ICSR_MASTER | NXP2120_ICSR_TX | NXP2120_ICSR_TXRX_ENABLE | NXP2120_ICSR_ST_ENB);
	i2c->ICSR = iicstat | NXP2120_ICSR_MASTER | NXP2120_ICSR_TX | NXP2120_ICSR_TXRX_ENABLE | NXP2120_ICSR_ST_ENB;
	/* Now SET OP_HOLD = 1 to stop condition. */
	pend = i2c->PEND;
	//WriteIODW(&i2c->PEND , pend | NXP2120_I2C_PEND_OP_HOLD);
	i2c->PEND = pend | NXP2120_I2C_PEND_OP_HOLD;
	i2c->ICSR = 0;

	i2c_state = STATE_STOP;
}

static int nxp_do_nextbyte(void)
{
    unsigned long tmp;
    unsigned long iicstat;
    unsigned char byte;
    struct NX_I2C_RegisterSet *const i2c = (struct NX_I2C_RegisterSet *)I2C_BASE;
    int ret = 0;

    iicstat = i2c->ICSR;

    switch (i2c_state) {

    case STATE_IDLE:
        goto out;
        break;

    case STATE_STOP:
        goto out_ack;

    case STATE_START:
        /* last thing we did was send a start condition on the
        * bus, or started a new i2c message
        */

        if (iicstat & NXP2120_ICSR_ACK_STATUS) {
            /* ack was not received... */
            ret = -1;
            i2c_stop();
            goto out_ack;
        }

        if (i2c_flag & I2C_M_RD)
            i2c_state = STATE_READ;
        else
            i2c_state = STATE_WRITE;

        /* terminate the transfer if there is nothing to do
        * as this is used by the i2c probe to find devices. */

        if ((i2c_msg_ptr > i2c_msg_len) || (i2c_msg_len == 0)) {
            ret = SEND_COMPLETE;
            i2c_stop();
            goto out_ack;
        }

        if (i2c_state == STATE_READ)
            goto prepare_read;

        /* fall through to the write state, as we will need to
        * send a byte as well */

    case STATE_WRITE:
        /* we are writing data to the device... check for the
        * end of the message, and if so, work out what to do
        */
        if (iicstat & NXP2120_ICSR_ACK_STATUS) {
            ret = -1;
            i2c_stop();
            printf("ack error2\n");
            goto out_ack;
        }

 retry_write:

        if (i2c_msg_ptr < i2c_msg_len) {
            byte = i2c_msg_buf[i2c_msg_ptr++];
            //WriteIODW(&i2c->IDSR , byte);
            i2c->IDSR = byte;

            /* delay after writing the byte to allow the
            * data setup time on the bus, as writing the
            * data to the register causes the first bit
            * to appear on SDA, and SCL will change as
            * soon as the interrupt is acknowledged */

            udelay(TX_SETUP);

        } else {
            /* send stop */
            i2c_stop();
            ret = SEND_COMPLETE;
        }
        break;

    case STATE_READ:
	    /* we have a byte of data in the data register, do
	     * something with it, and then work out wether we are
	     * going to do any more read/write
	     */

	     byte = i2c->IDSR;
	     i2c_msg_buf[i2c_msg_ptr++] = byte;

 prepare_read:
        if (i2c_msg_ptr == i2c_msg_len-1) {
            /* last byte of buffer */

            nxp2120_i2c_disable_ack();
        } else  {
            /* ok, we've read the entire buffer, see if there
            * is anything else we need to do */

            
            /* last message, send stop and complete */

            i2c_stop();
        }

        break;
    }

    /* acknowlegde the IRQ and get back on with the work */

 out_ack:
    WriteIODW(&i2c->PEND ,NXP2120_I2C_PEND_OP_HOLD | NXP2120_I2C_PEND_PEND);
 out:
    return ret;
}

int i2c_init(int speed, int unused)
{
    struct NX_I2C_RegisterSet *const i2c = (struct NX_I2C_RegisterSet *)I2C_BASE;
    struct NX_GPIO_RegisterSet *const gpiob = (struct NX_GPIO_RegisterSet *)PHY_BASEADDR_GPIO_B_MODULE;
   	unsigned long iicon = NXP2120_ICCR_IRQENB; /* ksw: There's no IRQ but this bit is needed */

	/* inititalise the gpio */
	gpiob->GPIOxALTFN[0] &= ~GPIOB_MASK;
	gpiob->GPIOxALTFN[0] |= GPIOB_SET;

	/* Enable PCLK for correct operation */
	i2c->CLKENB |=  0x8;

	/* write slave address and enable ack */

	i2c->IAR  = 0; /* ksw : what should I do? */

	//WriteIODW(&i2c->ICCR , iicon | NXP2120_ICCR_CLKSRC_PCLK_DIV_256);
	i2c->ICCR = NXP2120_ICCR_CLKSRC_PCLK_DIV_256; // iicon | IRQ disable ?
	/* set clock to 377KHz... */
   
    /* 1/16 prescale and 1/11 would make 377KHz */
    //WriteIODW(&i2c->ICSR , NXP2120_ICSR_TXRX_ENABLE | NXP2120_ICSR_ST_ENB);
    i2c->ICSR = NXP2120_ICSR_TXRX_ENABLE | NXP2120_ICSR_ST_ENB;
    
    /* QCNT_MAX should set as 2~15 */
    i2c->QCNT_MAX = 2;
    /* todo - check that the i2c lines aren't being dragged anywhere */
    //i2c_stop();

    return 0;
}

static int nxp_set_master()
{
	unsigned long iicstat;
	struct NX_I2C_RegisterSet *const i2c = (struct NX_I2C_RegisterSet *)I2C_BASE;
	int timeout = 400;

	//i2c_stop();
	while (timeout-- > 0) {
		iicstat = i2c->ICSR;

		if (!(iicstat & NXP2120_ICSR_BUSY))
			return 0;

		udelay(10);
	}

	return -1;
}

int i2c_do_xfer(u8 chip, int read, uchar *buf, int len)
{
    unsigned long x;
    struct NX_I2C_RegisterSet *const i2c = (struct NX_I2C_RegisterSet *)I2C_BASE;
    int i;
    int ret = nxp_set_master();

    if (ret != 0) {
        printf("Error : cannot get bus (error %d)\n", ret);

        return -1;
    }

    i2c_msg_ptr = 0;
    i2c_state   = STATE_START;
    i2c_msg_buf = buf;
    i2c_msg_len = len;
    i2c_flag = 0;
    if (read)
        i2c_flag |= I2C_M_RD;
   
    //printf("message start chip=%x\n", chip);
    i2c_message_start(chip);
    i=0;
    do {
        while (!((x = i2c->PEND) & NXP2120_I2C_PEND_PEND)) {
            udelay(100);
            i++;
            if (i > 10000) break;
        }
        //printf("pending is not...\n");
        ret = nxp_do_nextbyte();
        if (ret < 0)
            return ret;
    } while (ret != SEND_COMPLETE);

    return 0;
}

#endif
int i2c_probe(uchar chip)
{
	int ret;
	unsigned int freq;

    ret = i2c_do_xfer(chip, 0, NULL, 0);

	return ret;
}


int i2c_read(uchar chip, uint addr, int alen, uchar *buf, int len)
{
	int ret, i;
	char addrs[4];

	DPRINTF("%s chip: 0x%02x addr: 0x%04x alen: %d len: %d\n",__FUNCTION__, chip, addr, alen, len);

	for (i=0; i<alen; i++) {
	    addrs[alen-i-1] = addr & 0xFF;
	    addr >>= 8;
	}
	/* send addr */
	ret = i2c_do_xfer(chip, 0, addrs, alen);
	if (ret == 0) {
	    /* Now receive data */
	    ret = i2c_do_xfer(chip, 1, buf, len);
	}

	return ret;
}

int i2c_write(uchar chip, uint addr, int alen, uchar *buf, int len)
{
    int ret,i;
    char data[64]; /* Max 64 byte include addresses will transfer */
    DPRINTF("%s chip: 0x%02x addr: 0x%04x alen: %d len: %d\n",__FUNCTION__, chip, addr, alen, len);

    for (i=0; i<alen; i++) {
	    data[alen-i-1] = addr & 0xFF;
	    addr >>= 8;
	}
	for (i=alen; i<(len + alen); i++) {
	    data[i] = buf[i-alen];
	}
	/* send addr/data */
	ret = i2c_do_xfer(chip, 0, data, alen+len);

	return ret;
}

// ksw : For compatability, I added some basic function
uchar i2c_reg_read(uchar chip, uchar reg) {
	uchar val;
	i2c_read(chip, (uint)reg, 1, &val, 1);
	return val;
}
int i2c_reg_write(uchar chip, uchar reg, uchar val) {
	i2c_write(chip, (uint)reg, 1, &val, 1);
	return 1;
}

#endif /* CONFIG_HARD_I2C */
