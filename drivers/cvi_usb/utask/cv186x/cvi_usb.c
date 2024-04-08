/**********************************************************************
 * main.c
 *
 * USB Core Driver
 * main component function
 ***********************************************************************/
#include "include/debug.h"
#include "include/cvi_usb.h"
#include "include/platform_def.h"
#include <common.h>
#include <linux/delay.h>
#include <mmio.h>
#include <cpu_func.h>

extern int acm_app(void);

static void cvi_usb_hw_init(void)
{
	uint32_t value;
	uint32_t otp_usb_xtal_phy;
	uint32_t los_mask;

	value = mmio_read_32(REG_USB_SYS_REG_0114) | (reg_phy0_phy_reset);
	mmio_write_32(REG_USB_SYS_REG_0114, value);
	value = mmio_read_32(SOFT_RSTN_1) & (~reg_soft_reset_x_usb0);
	mmio_write_32(SOFT_RSTN_1, value);

	udelay(50);

	value = mmio_read_32(REG_USB_SYS_REG_010C);
	mmio_write_32(REG_USB_SYS_REG_010C, value | reg_phy0_ref_ssp_en);

	value = mmio_read_32(SOFT_RSTN_1) | reg_soft_reset_x_usb0;
	mmio_write_32(SOFT_RSTN_1, value);
	value = mmio_read_32(REG_USB_SYS_REG_0114) & ~reg_phy0_phy_reset;
	mmio_write_32(REG_USB_SYS_REG_0114, value);

	value = mmio_read_32(REG_USB_SYS_REG_0100) | reg_usb0_en;
	mmio_write_32(REG_USB_SYS_REG_0100, value);

	otp_usb_xtal_phy = mmio_read_32(OTP_USB_XTAL_PHY) & OTP_USB_XTAL_PHY_MASK;
	value &= mmio_read_32(REG_USB_SYS_REG_010C)
				& ~(reg_phy0_ref_clkdiv2) & ~(reg_phy0_fsel_Msk)
				& ~(reg_phy0_mpll_multiplier_Msk) & ~(reg_phy0_ssc_ref_clk_sel_Msk);
	switch(otp_usb_xtal_phy)
	{
		case 0x0:	// xtal = 24 MHz
			value |= (0x2A << reg_phy0_fsel_Pos);
			los_mask = 240;
			break;
		case 0x1:	// xtal = 19.2 MHz
			value |= (0x38 << reg_phy0_fsel_Pos);
			los_mask = 192;
			break;
		case 0x2:	// xtal = 20 MHz
			value |= (0x31 << reg_phy0_fsel_Pos);
			los_mask = 200;
			break;
		case 0x3:	// xtal = 40 MHz
			value |= reg_phy0_ref_clkdiv2 | (0x31 << reg_phy0_fsel_Pos);
			los_mask = 200;
			break;
	}
	mmio_write_32(REG_USB_SYS_REG_010C, value);

	value = mmio_read_32(USB_PHY0_TUNE_CTRL_REG1) & ~(reg_usb_phy0_pcs_rx_los_mask_val_Msk);
	value |= (los_mask << reg_usb_phy0_pcs_rx_los_mask_val_Pos);
	mmio_write_32(USB_PHY0_TUNE_CTRL_REG1, value);

	NOTICE("%s\n", __func__);
}

/* program starts here */
int cvi_usb_polling(void)
{
	cvi_usb_hw_init();

	acm_app();

	return 0;
}
