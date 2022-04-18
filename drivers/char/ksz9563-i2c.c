/*
 * ATMEL I2C TPM AT97SC3204T
 *
 * Copyright (C) 2012 V Lab Technologies
 *  Teddy Reed <teddy@prosauce.org>
 * Copyright (C) 2013, Obsidian Research Corp.
 *  Jason Gunthorpe <jgunthorpe@obsidianresearch.com>
 * Device driver for ATMEL I2C TPMs.
 *
 * Teddy Reed determined the basic I2C command flow, unlike other I2C TPM
 * devices the raw TCG formatted TPM command data is written via I2C and then
 * raw TCG formatted TPM command data is returned via I2C.
 *
 * TGC status/locality/etc functions seen in the LPC implementation do not
 * seem to be present.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see http://www.gnu.org/licenses/>.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/i2c.h>

#define I2C_DRIVER_NAME "ksz9563-i2c"

 //KSZ9563 ports
 #define KSZ9563_PORT1 1
 #define KSZ9563_PORT2 2
  
 //SPI command byte
 #define KSZ9563_SPI_CMD_WRITE 0x40000000
 #define KSZ9563_SPI_CMD_READ  0x60000000
 #define KSZ9563_SPI_CMD_ADDR  0x001FFFE0
  
 //KSZ9563 PHY registers
 #define KSZ9563_BMCR                              0x00
 #define KSZ9563_BMSR                              0x01
 #define KSZ9563_PHYID1                            0x02
 #define KSZ9563_PHYID2                            0x03
 #define KSZ9563_ANAR                              0x04
 #define KSZ9563_ANLPAR                            0x05
 #define KSZ9563_ANER                              0x06
 #define KSZ9563_ANNPR                             0x07
 #define KSZ9563_ANLPNPR                           0x08
 #define KSZ9563_GBCR                              0x09
 #define KSZ9563_GBSR                              0x0A
 #define KSZ9563_MMDACR                            0x0D
 #define KSZ9563_MMDAADR                           0x0E
 #define KSZ9563_GBESR                             0x0F
 #define KSZ9563_RLB                               0x11
 #define KSZ9563_LINKMD                            0x12
 #define KSZ9563_DPMAPCSS                          0x13
 #define KSZ9563_RXERCTR                           0x15
 #define KSZ9563_ICSR                              0x1B
 #define KSZ9563_AUTOMDI                           0x1C
 #define KSZ9563_PHYCON                            0x1F
  
 //KSZ9563 Switch registers
 #define KSZ9563_CHIP_ID0                          0x0000
 #define KSZ9563_CHIP_ID1                          0x0001
 #define KSZ9563_CHIP_ID2                          0x0002
 #define KSZ9563_CHIP_ID3                          0x0003
 #define KSZ9563_PME_PIN_CTRL                      0x0006
 #define KSZ9563_CHIP_ID4                          0x000F
 #define KSZ9563_GLOBAL_INT_STAT                   0x0010
 #define KSZ9563_GLOBAL_INT_MASK                   0x0014
 #define KSZ9563_GLOBAL_PORT_INT_STAT              0x0018
 #define KSZ9563_GLOBAL_PORT_INT_MASK              0x001C
 #define KSZ9563_SWITCH_OP                         0x0300
 #define KSZ9563_PORT1_INT_STATUS                  0x101B
 #define KSZ9563_PORT1_INT_MASK                    0x101F
 #define KSZ9563_PORT1_OP_CTRL0                    0x1020
 #define KSZ9563_PORT1_STATUS                      0x1030
 #define KSZ9563_PORT1_MSTP_STATE                  0x1B04
 #define KSZ9563_PORT2_INT_STATUS                  0x201B
 #define KSZ9563_PORT2_INT_MASK                    0x201F
 #define KSZ9563_PORT2_OP_CTRL0                    0x2020
 #define KSZ9563_PORT2_STATUS                      0x2030
 #define KSZ9563_PORT2_MSTP_STATE                  0x2B04
 #define KSZ9563_PORT3_INT_STATUS                  0x301B
 #define KSZ9563_PORT3_INT_MASK                    0x301F
 #define KSZ9563_PORT3_OP_CTRL0                    0x3020
 #define KSZ9563_PORT3_STATUS                      0x3030
 #define KSZ9563_PORT3_XMII_CTRL0                  0x3300
 #define KSZ9563_PORT3_XMII_CTRL1                  0x3301
 #define KSZ9563_PORT3_MSTP_STATE                  0x3B04
  
 //KSZ9563 Switch register access macros
 #define KSZ9563_PORTn_INT_STATUS(port)            (0x001B + ((port) * 0x1000))
 #define KSZ9563_PORTn_INT_MASK(port)              (0x001F + ((port) * 0x1000))
 #define KSZ9563_PORTn_OP_CTRL0(port)              (0x0020 + ((port) * 0x1000))
 #define KSZ9563_PORTn_STATUS(port)                (0x0030 + ((port) * 0x1000))
 #define KSZ9563_PORTn_XMII_CTRL0(port)            (0x0300 + ((port) * 0x1000))
 #define KSZ9563_PORTn_XMII_CTRL1(port)            (0x0301 + ((port) * 0x1000))
 #define KSZ9563_PORTn_MSTP_STATE(port)            (0x0B04 + ((port) * 0x1000))
 #define KSZ9563_PORTn_ETH_PHY_REG(port, addr)     (0x0100 + ((port) * 0x1000) + ((addr) * 2))
  
 //PHY Basic Control register
 #define KSZ9563_BMCR_RESET                        0x8000
 #define KSZ9563_BMCR_LOOPBACK                     0x4000
 #define KSZ9563_BMCR_SPEED_SEL_LSB                0x2000
 #define KSZ9563_BMCR_AN_EN                        0x1000
 #define KSZ9563_BMCR_POWER_DOWN                   0x0800
 #define KSZ9563_BMCR_ISOLATE                      0x0400
 #define KSZ9563_BMCR_RESTART_AN                   0x0200
 #define KSZ9563_BMCR_DUPLEX_MODE                  0x0100
 #define KSZ9563_BMCR_COL_TEST                     0x0080
 #define KSZ9563_BMCR_SPEED_SEL_MSB                0x0040
  
 //PHY Basic Status register
 #define KSZ9563_BMSR_100BT4                       0x8000
 #define KSZ9563_BMSR_100BTX_FD                    0x4000
 #define KSZ9563_BMSR_100BTX_HD                    0x2000
 #define KSZ9563_BMSR_10BT_FD                      0x1000
 #define KSZ9563_BMSR_10BT_HD                      0x0800
 #define KSZ9563_BMSR_EXTENDED_STATUS              0x0100
 #define KSZ9563_BMSR_MF_PREAMBLE_SUPPR            0x0040
 #define KSZ9563_BMSR_AN_COMPLETE                  0x0020
 #define KSZ9563_BMSR_REMOTE_FAULT                 0x0010
 #define KSZ9563_BMSR_AN_CAPABLE                   0x0008
 #define KSZ9563_BMSR_LINK_STATUS                  0x0004
 #define KSZ9563_BMSR_JABBER_DETECT                0x0002
 #define KSZ9563_BMSR_EXTENDED_CAPABLE             0x0001
  
 //PHY ID High register
 #define KSZ9563_PHYID1_DEFAULT                    0x0022
  
 //PHY ID Low register
 #define KSZ9563_PHYID2_DEFAULT                    0x1631
  
 //PHY Auto-Negotiation Advertisement register
 #define KSZ9563_ANAR_NEXT_PAGE                    0x8000
 #define KSZ9563_ANAR_REMOTE_FAULT                 0x2000
 #define KSZ9563_ANAR_PAUSE                        0x0C00
 #define KSZ9563_ANAR_100BT4                       0x0200
 #define KSZ9563_ANAR_100BTX_FD                    0x0100
 #define KSZ9563_ANAR_100BTX_HD                    0x0080
 #define KSZ9563_ANAR_10BT_FD                      0x0040
 #define KSZ9563_ANAR_10BT_HD                      0x0020
 #define KSZ9563_ANAR_SELECTOR                     0x001F
 #define KSZ9563_ANAR_SELECTOR_DEFAULT             0x0001
  
 //PHY Auto-Negotiation Link Partner Ability register
 #define KSZ9563_ANLPAR_NEXT_PAGE                  0x8000
 #define KSZ9563_ANLPAR_ACK                        0x4000
 #define KSZ9563_ANLPAR_REMOTE_FAULT               0x2000
 #define KSZ9563_ANLPAR_PAUSE                      0x0C00
 #define KSZ9563_ANLPAR_100BT4                     0x0200
 #define KSZ9563_ANLPAR_100BTX_FD                  0x0100
 #define KSZ9563_ANLPAR_100BTX_HD                  0x0080
 #define KSZ9563_ANLPAR_10BT_FD                    0x0040
 #define KSZ9563_ANLPAR_10BT_HD                    0x0020
 #define KSZ9563_ANLPAR_SELECTOR                   0x001F
 #define KSZ9563_ANLPAR_SELECTOR_DEFAULT           0x0001
  
 //PHY Auto-Negotiation Expansion Status register
 #define KSZ9563_ANER_PAR_DETECT_FAULT             0x0010
 #define KSZ9563_ANER_LP_NEXT_PAGE_ABLE            0x0008
 #define KSZ9563_ANER_NEXT_PAGE_ABLE               0x0004
 #define KSZ9563_ANER_PAGE_RECEIVED                0x0002
 #define KSZ9563_ANER_LP_AN_ABLE                   0x0001
  
 //PHY Auto-Negotiation Next Page register
 #define KSZ9563_ANNPR_NEXT_PAGE                   0x8000
 #define KSZ9563_ANNPR_MSG_PAGE                    0x2000
 #define KSZ9563_ANNPR_ACK2                        0x1000
 #define KSZ9563_ANNPR_TOGGLE                      0x0800
 #define KSZ9563_ANNPR_MESSAGE                     0x07FF
  
 //PHY Auto-Negotiation Link Partner Next Page Ability register
 #define KSZ9563_ANLPNPR_NEXT_PAGE                 0x8000
 #define KSZ9563_ANLPNPR_ACK                       0x4000
 #define KSZ9563_ANLPNPR_MSG_PAGE                  0x2000
 #define KSZ9563_ANLPNPR_ACK2                      0x1000
 #define KSZ9563_ANLPNPR_TOGGLE                    0x0800
 #define KSZ9563_ANLPNPR_MESSAGE                   0x07FF
  
 //PHY 1000BASE-T Control register
 #define KSZ9563_GBCR_TEST_MODE                    0xE000
 #define KSZ9563_GBCR_MS_MAN_CONF_EN               0x1000
 #define KSZ9563_GBCR_MS_MAN_CONF_VAL              0x0800
 #define KSZ9563_GBCR_PORT_TYPE                    0x0400
 #define KSZ9563_GBCR_1000BT_FD                    0x0200
 #define KSZ9563_GBCR_1000BT_HD                    0x0100
  
 //PHY 1000BASE-T Status register
 #define KSZ9563_GBSR_MS_CONF_FAULT                0x8000
 #define KSZ9563_GBSR_MS_CONF_RES                  0x4000
 #define KSZ9563_GBSR_LOCAL_RECEIVER_STATUS        0x2000
 #define KSZ9563_GBSR_REMOTE_RECEIVER_STATUS       0x1000
 #define KSZ9563_GBSR_LP_1000BT_FD                 0x0800
 #define KSZ9563_GBSR_LP_1000BT_HD                 0x0400
 #define KSZ9563_GBSR_IDLE_ERR_COUNT               0x00FF
  
 //PHY MMD Setup register
 #define KSZ9563_MMDACR_FUNC                       0xC000
 #define KSZ9563_MMDACR_FUNC_ADDR                  0x0000
 #define KSZ9563_MMDACR_FUNC_DATA_NO_POST_INC      0x4000
 #define KSZ9563_MMDACR_FUNC_DATA_POST_INC_RW      0x8000
 #define KSZ9563_MMDACR_FUNC_DATA_POST_INC_W       0xC000
 #define KSZ9563_MMDACR_DEVAD                      0x001F
  
 //PHY Extended Status register
 #define KSZ9563_GBESR_1000BX_FD                   0x8000
 #define KSZ9563_GBESR_1000BX_HD                   0x4000
 #define KSZ9563_GBESR_1000BT_FD                   0x2000
 #define KSZ9563_GBESR_1000BT_HD                   0x1000
  
 //PHY Remote Loopback register
 #define KSZ9563_RLB_REMOTE_LOOPBACK               0x0100
  
 //PHY LinkMD register
 #define KSZ9563_LINKMD_TEST_EN                    0x8000
 #define KSZ9563_LINKMD_PAIR                       0x3000
 #define KSZ9563_LINKMD_PAIR_A                     0x0000
 #define KSZ9563_LINKMD_PAIR_B                     0x1000
 #define KSZ9563_LINKMD_PAIR_C                     0x2000
 #define KSZ9563_LINKMD_PAIR_D                     0x3000
 #define KSZ9563_LINKMD_STATUS                     0x0300
 #define KSZ9563_LINKMD_STATUS_NORMAL              0x0000
 #define KSZ9563_LINKMD_STATUS_OPEN                0x0100
 #define KSZ9563_LINKMD_STATUS_SHORT               0x0200
  
 //PHY Digital PMA/PCS Status register
 #define KSZ9563_DPMAPCSS_1000BT_LINK_STATUS       0x0002
 #define KSZ9563_DPMAPCSS_100BTX_LINK_STATUS       0x0001
  
 //Port Interrupt Control/Status register
 #define KSZ9563_ICSR_JABBER_IE                    0x8000
 #define KSZ9563_ICSR_RECEIVE_ERROR_IE             0x4000
 #define KSZ9563_ICSR_PAGE_RECEIVED_IE             0x2000
 #define KSZ9563_ICSR_PAR_DETECT_FAULT_IE          0x1000
 #define KSZ9563_ICSR_LP_ACK_IE                    0x0800
 #define KSZ9563_ICSR_LINK_DOWN_IE                 0x0400
 #define KSZ9563_ICSR_REMOTE_FAULT_IE              0x0200
 #define KSZ9563_ICSR_LINK_UP_IE                   0x0100
 #define KSZ9563_ICSR_JABBER_IF                    0x0080
 #define KSZ9563_ICSR_RECEIVE_ERROR_IF             0x0040
 #define KSZ9563_ICSR_PAGE_RECEIVED_IF             0x0020
 #define KSZ9563_ICSR_PAR_DETECT_FAULT_IF          0x0010
 #define KSZ9563_ICSR_LP_ACK_IF                    0x0008
 #define KSZ9563_ICSR_LINK_DOWN_IF                 0x0004
 #define KSZ9563_ICSR_REMOTE_FAULT_IF              0x0002
 #define KSZ9563_ICSR_LINK_UP_IF                   0x0001
  
 //PHY Auto MDI/MDI-X register
 #define KSZ9563_AUTOMDI_MDI_SET                   0x0080
 #define KSZ9563_AUTOMDI_SWAP_OFF                  0x0040
  
 //PHY Control register
 #define KSZ9563_PHYCON_JABBER_EN                  0x0200
 #define KSZ9563_PHYCON_SPEED_1000BT               0x0040
 #define KSZ9563_PHYCON_SPEED_100BTX               0x0020
 #define KSZ9563_PHYCON_SPEED_10BT                 0x0010
 #define KSZ9563_PHYCON_DUPLEX_STATUS              0x0008
 #define KSZ9563_PHYCON_1000BT_MS_STATUS           0x0004
  
 //Global Chip ID 0 register
 #define KSZ9563_CHIP_ID0_DEFAULT                  0x00
  
 //Global Chip ID 1 register
 #define KSZ9563_CHIP_ID1_DEFAULT                  0x98
  
 //Global Chip ID 2 register
 #define KSZ9563_CHIP_ID2_DEFAULT                  0x93
  
 //Global Chip ID 3 register
 #define KSZ9563_CHIP_ID3_REVISION_ID              0xF0
 #define KSZ9563_CHIP_ID3_GLOBAL_SOFT_RESET        0x01
  
 //PME Pin Control register
 #define KSZ9563_PME_PIN_CTRL_PME_PIN_OUT_EN       0x02
 #define KSZ9563_PME_PIN_CTRL_PME_PIN_OUT_POL      0x01
  
 //Global Chip ID 4 register
 #define KSZ9563_CHIP_ID4_SKU_ID                   0xFF
  
 //Global Interrupt Status register
 #define KSZ9563_GLOBAL_INT_STAT_LUE               0x80000000
 #define KSZ9563_GLOBAL_INT_STAT_GPIO_TRIG_TS_UNIT 0x40000000
  
 //Global Interrupt Mask register
 #define KSZ9563_GLOBAL_INT_MASK_LUE               0x80000000
 #define KSZ9563_GLOBAL_INT_MASK_GPIO_TRIG_TS_UNIT 0x40000000
  
 //Global Port Interrupt Status register
 #define KSZ9563_GLOBAL_PORT_INT_STAT_PORT3        0x00000004
 #define KSZ9563_GLOBAL_PORT_INT_STAT_PORT2        0x00000002
 #define KSZ9563_GLOBAL_PORT_INT_STAT_PORT1        0x00000001
  
 //Global Port Interrupt Mask register
 #define KSZ9563_GLOBAL_PORT_INT_MASK_PORT3        0x00000004
 #define KSZ9563_GLOBAL_PORT_INT_MASK_PORT2        0x00000002
 #define KSZ9563_GLOBAL_PORT_INT_MASK_PORT1        0x00000001
  
 //Switch Operation register
 #define KSZ9563_SWITCH_OP_DOUBLE_TAG_EN           0x80
 #define KSZ9563_SWITCH_OP_SOFT_HARD_RESET         0x02
 #define KSZ9563_SWITCH_OP_START_SWITCH            0x01
  
 //Port N Interrupt Status register
 #define KSZ9563_PORTn_INT_STATUS_PTP              0x04
 #define KSZ9563_PORTn_INT_STATUS_PHY              0x02
 #define KSZ9563_PORTn_INT_STATUS_ACL              0x01
  
 //Port N Interrupt Mask register
 #define KSZ9563_PORTn_INT_MASK_PTP                0x04
 #define KSZ9563_PORTn_INT_MASK_PHY                0x02
 #define KSZ9563_PORTn_INT_MASK_ACL                0x01
  
 //Port N Operation Control 0 register
 #define KSZ9563_PORTn_OP_CTRL0_LOCAL_LOOPBACK     0x80
 #define KSZ9563_PORTn_OP_CTRL0_REMOTE_LOOPBACK    0x40
 #define KSZ9563_PORTn_OP_CTRL0_TAIL_TAG_EN        0x04
 #define KSZ9563_PORTn_OP_CTRL0_TX_QUEUE_SPLIT_EN  0x03
  
 //Port N Status register
 #define KSZ9563_PORTn_STATUS_SPEED                0x18
 #define KSZ9563_PORTn_STATUS_SPEED_10MBPS         0x00
 #define KSZ9563_PORTn_STATUS_SPEED_100MBPS        0x08
 #define KSZ9563_PORTn_STATUS_SPEED_1000MBPS       0x10
 #define KSZ9563_PORTn_STATUS_DUPLEX               0x04
 #define KSZ9563_PORTn_STATUS_TX_FLOW_CTRL_EN      0x02
 #define KSZ9563_PORTn_STATUS_RX_FLOW_CTRL_EN      0x01
  
 //XMII Port N Control 0 register
 #define KSZ9563_PORTn_XMII_CTRL0_DUPLEX           0x40
 #define KSZ9563_PORTn_XMII_CTRL0_TX_FLOW_CTRL_EN  0x20
 #define KSZ9563_PORTn_XMII_CTRL0_SPEED_10_100     0x10
 #define KSZ9563_PORTn_XMII_CTRL0_RX_FLOW_CTRL_EN  0x08
  
 //XMII Port N Control 1 register
 #define KSZ9563_PORTn_XMII_CTRL1_SPEED_1000       0x40
 #define KSZ9563_PORTn_XMII_CTRL1_RGMII_ID_IG      0x10
 #define KSZ9563_PORTn_XMII_CTRL1_RGMII_ID_EG      0x08
 #define KSZ9563_PORTn_XMII_CTRL1_MII_RMII_MODE    0x04
 #define KSZ9563_PORTn_XMII_CTRL1_IF_TYPE          0x03
 #define KSZ9563_PORTn_XMII_CTRL1_IF_TYPE_MII      0x00
 #define KSZ9563_PORTn_XMII_CTRL1_IF_TYPE_RMII     0x01
 #define KSZ9563_PORTn_XMII_CTRL1_IF_TYPE_RGMII    0x03
  
 //Port N MSTP State register
 #define KSZ9563_PORTn_MSTP_STATE_TRANSMIT_EN      0x04
 #define KSZ9563_PORTn_MSTP_STATE_RECEIVE_EN       0x02
 #define KSZ9563_PORTn_MSTP_STATE_LEARNING_DIS     0x01
  
 //Tail tag encoding
 #define KSZ9563_TAIL_TAG_ENCODE(port) (0x20 | ((port) & 0x03))
 //Tail tag decoding
 #define KSZ9563_TAIL_TAG_DECODE(tag) (((tag) & 0x01) + 1)
 
int debug = 3;
struct i2c_client *g_client;

static int ksz9563_write_reg(struct i2c_client *client, u16 reg, u8 data)
{
	int ret = 0;
	u8 dev_addr;
	u8 buf1[] = { reg >> 8, reg & 0xFF };
	u8 buf2[] = { data };
	struct i2c_msg msg1 = { .flags = 0, .buf = buf1, .len = 2 };
	struct i2c_msg msg2 = { .flags = 0, .buf = buf2, .len = 1 };

	dev_addr = client->addr;
	msg1.addr = dev_addr;
	msg2.addr = dev_addr;

	if (debug >= 2)
		printk("%s: reg=0x%04X, data=0x%02X\n", __func__, reg, data);

	ret = i2c_transfer(client->adapter, &msg1, 1);
	if (ret != 1)
		return -EIO;

	ret = i2c_transfer(client->adapter, &msg2, 1);
	return (ret != 1) ? -EIO : 0;
}

static int ksz9563_read_reg(struct i2c_client *client, u16 reg, u8 *p_data)
{
	int ret;
	u8 dev_addr;

	u8 buf1[] = { reg >> 8, reg & 0xFF };
	u8 buf2[] = { 0 };
	struct i2c_msg msg1 = { .flags = 0, .buf = buf1, .len = 2 };
	struct i2c_msg msg2 = { .flags = I2C_M_RD, .buf = buf2, .len = 1 };

	dev_addr = client->addr;
	msg1.addr = dev_addr;
	msg2.addr = dev_addr;

	ret = i2c_transfer(client->adapter, &msg1, 1);
	if (ret != 1) {
		printk("%s: error reg=0x%04x, ret=%i\n", __func__, reg, ret);
		return -EIO;
	}

	ret = i2c_transfer(client->adapter, &msg2, 1);
	if (ret != 1)
		return -EIO;

	*p_data = buf2[0];
	if (debug >= 2)
		printk("%s: reg=0x%04X, data=0x%02X\n",
			__func__, reg, buf2[0]);

	return 0;
}

 void ksz9563DumpPhyReg(struct i2c_client *client, uint8_t port)
 {
    uint8_t i;
	uint8_t temp;
	uint16_t regaddr;
	
    //Loop through PHY registers
    for(i = 0; i < 32; i++)
    {
       //Display current PHY register
      // 
      //    
	  regaddr = KSZ9563_PORTn_ETH_PHY_REG(port, i);
	  ksz9563_read_reg(client, regaddr, &temp);
	  pr_notice("i = %d  reg:%x : 0x%4x \r\n", i, regaddr, temp);
    }
  
    //Terminate with a line feed
    pr_notice("\r\n");
 }
 
void i2c_ksz9563_init(void)
{
	uint8_t temp;
	unsigned int port;
	
	
	//Wait for the serial interface to be ready
	do
	{
	  //Read CHIP_ID1 register
	  ksz9563_read_reg(g_client, KSZ9563_CHIP_ID1,&temp);
	  pr_notice("temp value:%x\t",temp);

	  //The returned data is invalid until the serial interface is ready
	} while(temp != KSZ9563_CHIP_ID1_DEFAULT);
	   
	//Reset switch
	ksz9563_write_reg(g_client, KSZ9563_SWITCH_OP,
	  KSZ9563_SWITCH_OP_SOFT_HARD_RESET);

	//Wait for the reset to complete
	do
	{
	  //Read switch operation register
	  ksz9563_read_reg(g_client, KSZ9563_SWITCH_OP, &temp);

	  //The reset bit is self-clearing
	} while((temp & KSZ9563_SWITCH_OP_SOFT_HARD_RESET) != 0);

	//Add internal delay to ingress and egress RGMII clocks
	ksz9563_read_reg(g_client, KSZ9563_PORT3_XMII_CTRL1,&temp);
	temp |= KSZ9563_PORTn_XMII_CTRL1_RGMII_ID_IG;
	temp |= KSZ9563_PORTn_XMII_CTRL1_RGMII_ID_EG;
	ksz9563_write_reg(g_client, KSZ9563_PORT3_XMII_CTRL1, temp);

	//Start switch operation
	ksz9563_write_reg(g_client, KSZ9563_SWITCH_OP,
	  KSZ9563_SWITCH_OP_START_SWITCH);
	  
	//Loop through ports
	#if 0
    for(port = KSZ9563_PORT1; port <= KSZ9563_PORT2; port++)
    {
       //Debug message
       pr_notice("Port %u:\r\n", port);
       //Dump PHY registers for debugging purpose
       ksz9563DumpPhyReg(g_client, port);
    }
	#endif
}
static int i2c_ksz9563_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	//struct device *dev = &client->dev;
	unsigned short regaddr = 0x0330;
	unsigned char value = 0x06;
	int retry;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;


	//priv = devm_kzalloc(dev, sizeof(struct priv_data), GFP_KERNEL);
	//if (!priv)
	//	return -ENOMEM;../

	//dev_set_drvdata(&chip->dev, priv);

	regaddr = 0x0001;
	ksz9563_read_reg(client,regaddr, &value);
	printk("ldx read out chip id: high 0x%x\n", value);

	regaddr = 0x0002;
	retry = 10;
	do{

		ksz9563_read_reg(client,regaddr, &value);
		printk("ldx read out chip id: low 0x%x\n", value);
		if(value == 0x93)
		{
			printk("found ksz9563 chip\n");
			break;
		}
		retry--;
	
	}while(retry);


	//regaddr = 0x0330;
	//value = 0x06;

	//ksz9563_write_reg(client,regaddr, value);

	ksz9563_read_reg(client,regaddr, &value);
	printk("ldx read out 0x0330: 0x%x\n", value);
	g_client = client;

	return 0;
}


static int i2c_ksz9563_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id i2c_ksz9563_id[] = {
	{I2C_DRIVER_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, i2c_ksz9563_id);

#ifdef CONFIG_OF
static const struct of_device_id i2c_ksz9563_of_match[] = {
	{.compatible = "microchip,ksz9563"},
	{},
};
MODULE_DEVICE_TABLE(of, i2c_ksz9563_of_match);
#endif


static struct i2c_driver i2c_ksz9563_driver = {
	.id_table = i2c_ksz9563_id,
	.probe = i2c_ksz9563_probe,
	.remove = i2c_ksz9563_remove,
	.driver = {
		.name = I2C_DRIVER_NAME,
		.of_match_table = of_match_ptr(i2c_ksz9563_of_match),
	},
};

module_i2c_driver(i2c_ksz9563_driver);

MODULE_AUTHOR("Jason Gunthorpe <jgunthorpe@obsidianresearch.com>");
MODULE_DESCRIPTION("Atmel TPM I2C Driver");
MODULE_LICENSE("GPL");
