/*
 * (C) Copyright 2009
 * jung hyun kim, Nexell Co, <jhkim@nexell.co.kr>
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
/*---------------------------------------------------------------------------*/
#define	I2C_DELAY_HZ			100000
/*----------------------------------------------------------------------------
 * I2C control macro
 */
#define SCL_HIGH(_io)		NX_GPIO_SetOutputValue 	(PAD_GET_GROUP(_io), PAD_GET_BITNO(_io), CTRUE )
#define SCL_LOW(_io)		NX_GPIO_SetOutputValue 	(PAD_GET_GROUP(_io), PAD_GET_BITNO(_io), CFALSE)
#define SCL_OUTPUT(_io)		NX_GPIO_SetOutputEnable	(PAD_GET_GROUP(_io), PAD_GET_BITNO(_io), CTRUE )
#define SCL_INPUT(_io)		NX_GPIO_SetOutputEnable	(PAD_GET_GROUP(_io), PAD_GET_BITNO(_io), CFALSE)
#define SCL_DATA(_io)		NX_GPIO_GetInputValue  	(PAD_GET_GROUP(_io), PAD_GET_BITNO(_io))
#define SDA_HIGH(_io)		NX_GPIO_SetOutputValue 	(PAD_GET_GROUP(_io), PAD_GET_BITNO(_io), CTRUE )
#define SDA_LOW(_io)		NX_GPIO_SetOutputValue 	(PAD_GET_GROUP(_io), PAD_GET_BITNO(_io), CFALSE)
#define SDA_OUTPUT(_io)		NX_GPIO_SetOutputEnable	(PAD_GET_GROUP(_io), PAD_GET_BITNO(_io), CTRUE )
#define SDA_INPUT(_io)		NX_GPIO_SetOutputEnable	(PAD_GET_GROUP(_io), PAD_GET_BITNO(_io), CFALSE)
#define SDA_DATA(_io)		NX_GPIO_GetInputValue  	(PAD_GET_GROUP(_io), PAD_GET_BITNO(_io))

//#define I2C_DELAY(_n)		{ volatile u_int x=0; while ((cyc)*_n > x++); }	/* default _d = 0x200 */
#define I2C_DELAY(_n)	udelay(_n * cyc);	

#define	SHT		(2)		/* start  hold  time */
#define	EST		(2)		/* Stop   setup time */

#define	DHT		(1)		/* data   hold  time */
#define	DST		(1)		/* data   setup time */
#define	CHT		(1)		/* clock  high  time */

/* 			________		 ___________		______
 *	<SDA>	 		|_______|			|_______|
 *			 <1>|<1>|SHT|DHT|DST|CHT|DHT|DST|EST|<1>
 *			____________		 ___		 __________
 *	<SCL>		 		|_______|	|_______|
 */

/*----------------------------------------------------------------------------*/
static inline void pio_start(struct i2c_params *par)
{
	u_int scl = par->sclpad;
	u_int sda = par->sdapad;
	u_int cyc = par->delay;
	/* SCL/SDA High */
	SDA_HIGH	(sda);
	SDA_OUTPUT	(sda);
	I2C_DELAY	(1);

	SCL_HIGH	(scl);
	SCL_OUTPUT	(scl);
	I2C_DELAY	(1);

	/* START signal */
	SDA_LOW		(sda);	/* Start condition */
	I2C_DELAY	(SHT);	/* Start hold */

	SCL_LOW		(scl);
	I2C_DELAY	(DHT);		/* Data  hold */
}

static inline void pio_stop(struct i2c_params *par)
{
	u_int scl = par->sclpad;
	u_int sda = par->sdapad;
	u_int cyc = par->delay;

	/* STOP signal */
	SDA_LOW		(sda);
	SDA_OUTPUT	(sda);
	I2C_DELAY	(DST);

	SCL_HIGH	(scl);
	I2C_DELAY	(EST);

	SDA_HIGH	(sda);
	I2C_DELAY	(1);

	SCL_INPUT	(scl);
	SDA_INPUT	(sda);
}

static inline int pio_putbyte(struct i2c_params *par, unsigned char data)
{
	u_int scl = par->sclpad;
	u_int sda = par->sdapad;
	u_int cyc = par->delay;
	int i, nack = 0;

	SDA_OUTPUT	(sda);

	for (i=7 ; i >= 0 ; i--) {
		if (data & (1<<i))
			SDA_HIGH(sda);
		else
			SDA_LOW	(sda);

		I2C_DELAY	(DST);

		SCL_HIGH	(scl);
		I2C_DELAY	(CHT);

		SCL_LOW		(scl);
		I2C_DELAY	(DHT);
	}

	SDA_INPUT	(sda);
	I2C_DELAY	(DST);

	SCL_HIGH	(scl);
	I2C_DELAY	(CHT);

	/* Falling Edge */
#if (0)
	for (i = 0; (ACK_WAIT_TIMEOUT * 1000) > i; i++) {
		nack = SDA_DATA	(sda);
		if (! nack) break;
		udelay(1);
	}
#else
	nack = SDA_DATA(sda);
#endif

	SCL_LOW		(scl);
	I2C_DELAY	(DHT);

	SDA_INPUT	(sda);	/* END */

	return (nack ? -1 : 0);
}

static inline u_char pio_getbyte(struct i2c_params *par, bool ack)
{
	u_int  scl = par->sclpad;
	u_int  sda = par->sdapad;
	u_int  cyc = par->delay;
	u_char dat = 0;
	int   i;

	SDA_INPUT	(sda);

	for ( i=7; i >= 0; i-- ) {

		I2C_DELAY	(DST);
		SCL_HIGH	(scl);
		I2C_DELAY	(CHT);

		/* Falling Edge */
		if (SDA_DATA(sda))
			dat = (unsigned char)(dat | (1<<i));
		else
			dat = (unsigned char)(dat | (0<<i));

		SCL_LOW		(scl);
		I2C_DELAY	(DHT);
	}

	SDA_OUTPUT(sda);

	if (ack)
		SDA_LOW	(sda);
	else
		SDA_HIGH(sda);

	I2C_DELAY	(DST);

	SCL_HIGH	(scl);
	I2C_DELAY	(CHT);

	SCL_LOW		(scl);
	I2C_DELAY	(DHT);

	SDA_INPUT	(sda);	/* END */

	return dat;
}

/*----------------------------------------------------------------------------
 * I2C u-boot
 */
#if defined(CONFIG_I2C_MULTI_BUS)
int i2c_set_bus_num (unsigned int bus)
{
	if (bus > NUMBER_OF_I2C_MODULE-1) {
		printf("i2c bus %d is not exist (max bus %d)\n", bus, NUMBER_OF_I2C_MODULE-1);
		return -1;
	}

	if (NULL == _pi2c)
		i2c_init(CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);

	_pi2c   = &i2c_info[bus];
	_busnum = bus;
	DBGOUT("[%d]: set to bus=%d, speed=%d\n", _busnum, bus, _pi2c->speed);
	return 0;
}

unsigned int i2c_get_bus_num(void)
{
	DBGOUT("[%d]: get bus=%d\n", _busnum, _busnum);
	return _busnum;
}
#endif

int i2c_set_bus_speed(unsigned int speed)
{
	if (NULL == _pi2c)
		i2c_init(CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);
	
	_pi2c = &i2c_info[_busnum];
	_pi2c->speed = speed;

#ifdef CONFIG_MMU_ENABLE
	_pi2c->delay = (speed > I2C_DELAY_HZ) ?  1000000/(3*speed) + 1:
							1000000/(3*I2C_DELAY_HZ) + 1 ;
#else
	int tmp;
	tmp = (speed > I2C_DELAY_HZ) ?  1000000/(speed)  : 
							1000000/( I2C_DELAY_HZ );
	_pi2c->delay = ((tmp- 7) / 3) ;
#endif
	if(0 >= _pi2c->delay) 
  	 	_pi2c->delay = 1;

	DBGOUT("i2c speed=%d, delay=%d\n", _pi2c->speed, _pi2c->delay);
	return 0;
}

unsigned int i2c_get_bus_speed(void)
{
	if (NULL == _pi2c)
		return 0;
	return _pi2c->speed;
}

int i2c_write(u8 chip, u32 addr, int alen, u8 *buffer, int len)
{
	struct i2c_params *i2c = _pi2c;
	int no_stop = i2c->no_stop;

	u8  addr_bytes[3]; /* lowest...highest byte of data address */
	u8  data;
	int ret  = -1;

	chip = (chip<<1);

#if (DEBUG_I2C)
{
	int i = 0;
	printf("i2c[%d]: W chip=0x%2x, addr=0x%x, alen=%d, wlen=%d ",
		_busnum, chip, addr, alen, len);
	for (; len > i; i++)
		printf("[%d]:0x%2x ", i, buffer[i]);
	printf("\n");
}
#endif

	if (NULL == i2c) {
		i2c_init(CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);
		i2c = _pi2c = &i2c_info[_busnum];
	}

	/*
	 * send memory address bytes;
	 * alen defines how much bytes we have to send.
	 */
	addr_bytes[0] = (u8)((addr >>  0) & 0x000000FF);
	addr_bytes[1] = (u8)((addr >>  8) & 0x000000FF);
	addr_bytes[2] = (u8)((addr >> 16) & 0x000000FF);

	/*
	 * transfer : slave addresss
	 */
	data = chip;
	while(0){
	pio_start(i2c);
	ret = pio_putbyte(i2c, data);
	pio_stop(i2c);
	mdelay(100);
	}
	pio_start(i2c);
	ret = pio_putbyte(i2c, data);
	if (ret) {
		printf("fail, i2c:%d start wait ack, addr:0x%02x \n", i2c->device, data);
		goto __end_i2c_w;
	}

	/*
	 * transfer : regsiter addr
	 */
	while (--alen >= 0) {
		data = addr_bytes[alen];
		ret  = pio_putbyte(i2c, data);
		if (ret) {
			printf("Fail, i2c[%d] no ack data [0x%2x] \n", i2c->device, data);
			goto __end_i2c_w;
		}
	}

	/*
	 * transfer : data
	 */
	while (len--) {
		data = *(buffer++);
		ret  = pio_putbyte(i2c, data);
		if (ret) {
			printf("Fail, i2c[%d] no ack data [0x%2x] \n", i2c->device, data);
			goto __end_i2c_w;
		}
	}

#if (DEBUG_I2C)
	printf("i2c[%d]: W done chip=0x%2x \n", i2c->device, chip);
#endif

__end_i2c_w:
	/*
	 * transfer : end
	 */
	if (ret || !no_stop)
		pio_stop(i2c);

	return ret;
}

int i2c_read (u8 chip, uint addr, int alen, u8 *buffer, int len)
{
	struct i2c_params *i2c = _pi2c;
	u8  data;
	int ret  = -1;

#if (DEBUG_I2C)
	printf("i2c[%d]: R chip=0x%2x, addr=0x%x, alen=%d, rlen=%d\n",
		i2c->device, (chip<<1)|0x1, addr, alen, len);
#endif

	/*
	 * transfer : register addr
	 */
	if (0 > i2c_write(chip, addr, alen, NULL, 0))
		return ret;

	/*
	 * transfer : slave addresss
	 */
	data = (chip<<1) | 0x01;
	while(0){
	pio_start(i2c);
	ret = pio_putbyte(i2c, data);
	pio_stop(i2c);
	mdelay(100);
	}

	pio_start(i2c);
	ret = pio_putbyte(i2c, data);
	if (ret) {
		printf("fail, i2c:%d start wait ack, addr:0x%02x \n", i2c->device, data);
		goto __end_i2c_r;
	}

	/*
	 * transfer : read data
	 */
	while (len--) {
		int ack_gen = (len == 0) ? 0: 1;
		*buffer = pio_getbyte(i2c, ack_gen);
		 buffer++;
	}

__end_i2c_r:

	pio_stop(i2c);
	return ret;
}

void i2c_init(int speed, int slaveaddr)
{
	int i, scl, sda;

    NX_GPIO_Initialize();
    for (i = 0; NX_GPIO_GetNumberOfModule() > i; i++) {
        NX_GPIO_SetBaseAddress(i, (U32)IO_ADDRESS(NX_GPIO_GetPhysicalAddress(i)));
        NX_GPIO_OpenModule(i);
    }

	for (i = 0; NUMBER_OF_I2C_MODULE > i; i++) {
		/* set pad function */
		scl = i2c_info[i].sclpad;
		sda = i2c_info[i].sdapad;
		NX_GPIO_SetPadFunction(PAD_GET_GROUP(scl), PAD_GET_BITNO(scl), PAD_GET_FUNC(scl));
		NX_GPIO_SetPadFunction(PAD_GET_GROUP(sda), PAD_GET_BITNO(sda), PAD_GET_FUNC(sda));
		DBGOUT("[%d]: scl=%d.%d, sda=%d.%d\n", i,
			PAD_GET_GROUP(scl), PAD_GET_BITNO(scl),
			PAD_GET_GROUP(sda), PAD_GET_BITNO(sda));

		/* set bus speed */
	    _pi2c   = &i2c_info[i];
		_busnum = i;
		i2c_set_bus_speed(speed);
	}

    _busnum = 0;
    _pi2c = &i2c_info[_busnum];
}

/*
 * params in:
 * 		chip = slave addres
 * return:
 *		0 = detect else not detect
 */
int i2c_probe(u8 chip)
{
	struct i2c_params *i2c = _pi2c;
	u8  data;
	int ret  = -1;

	if (NULL == _pi2c){
		i2c_init(CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);
		i2c = _pi2c = &i2c_info[_busnum];
	}

	/*
	 * test with tx
	 * probe ragne 0 ~ 128
	 */
	chip <<= 1;
	DBGOUT("[%d]: chip=0x%02x, speed=%d\n",
		i2c->device, chip, i2c->speed);

	/*
	 * transfer : slave addresss
	 */
	data = chip;
	pio_start(i2c);
	ret = pio_putbyte(i2c, data);
	pio_stop(i2c);

//	if (0 == ret)
//		printf(" 0x%02x -> 0x", chip);
	return ret;
}
