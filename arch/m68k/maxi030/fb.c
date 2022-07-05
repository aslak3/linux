/*
 *	MAXI030 framebuffer driver for 640x480 8bpp paletted mode
 *
 *	Copyright (C) 2022 Lawrence Manning, based on:
 *	
 *      linux/drivers/video/maxinefb.c
 *
 *	DECstation 5000/xx onboard framebuffer support ... derived from:
 *	"HP300 Topcat framebuffer support (derived from macfb of all things)
 *	Phil Blundell <philb@gnu.org> 1998", the original code can be
 *      found in the file hpfb.c in the same directory.
 *
 *      DECstation related code Copyright (C) 1999,2000,2001 by
 *      Michael Engel <engel@unix-ag.org> and
 *      Karsten Merker <merker@linuxtag.org>.
 *      This file is subject to the terms and conditions of the GNU General
 *      Public License.  See the file COPYING in the main directory of this
 *      archive for more details.
 *
 */

/*
 * Changes:
 * 2001/01/27 removed debugging and testing code, fixed fb_ops
 *            initialization which had caused a crash before,
 *            general cleanup, first official release (KM)
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/fb.h>

static struct fb_info fb_info;

static const struct fb_var_screeninfo maxi030fb_defined = {
	.xres =		640,
	.yres =		480,
	.xres_virtual =	640,
	.yres_virtual =	480,
	.bits_per_pixel =8,
	.activate =	FB_ACTIVATE_NOW,
	.height =	-1,
	.width =	-1,
	.vmode =	FB_VMODE_NONINTERLACED,
	.red            = { .length = 8, },
	.green          = { .length = 8, },
	.blue           = { .length = 8, },
};

static struct fb_fix_screeninfo maxi030fb_fix __initdata = {
	.id =		"maxi030",
	.smem_len =	(640*480),
	.type =		FB_TYPE_PACKED_PIXELS,
	.visual =	FB_VISUAL_PSEUDOCOLOR,
	.line_length =	640,
};

#define PALETTE_BASE 0x40200400

/* Set the palette */
static int maxi030fb_setcolreg(unsigned regno, unsigned red, unsigned green,
			      unsigned blue, unsigned transp, struct fb_info *info)
{
	/* value to be written into the palette reg. */
	unsigned long hw_colorvalue = 0;
	unsigned long regaddr;

	if (regno > 255)
		return 1;

	red >>= 8;
	green >>= 8;
	blue >>= 8;

	hw_colorvalue = (red << 16) + (green << 8) + (blue);

	regaddr = PALETTE_BASE + (regno * 4);

//	TODO: Figure out why this is being byte swapped
//	writel(hw_colorvalue, regaddr);
	writeb(red, regaddr + 1);
	writeb(green, regaddr + 2);
	writeb(blue, regaddr + 3);
	
	return 0;
}

static const struct fb_ops maxi030fb_ops = {
	.owner		= THIS_MODULE,
	.fb_setcolreg	= maxi030fb_setcolreg,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
};

int __init maxi030fb_init(void)
{
	unsigned long fboff;
	unsigned long fb_start;

	if (fb_get_options("maxi030fb", NULL))
		return -ENODEV;

	printk(KERN_INFO "MAXI030: initializing framebuffer\n");

	/* Framebuffer display memory base address */
	fb_start = 0x40000000;

	/* Clear screen */
	for (fboff = fb_start; fboff < fb_start + (640 * 480); fboff++)
		*(volatile unsigned char *)fboff = 0x0;

	maxi030fb_fix.smem_start = fb_start;

	fb_info.fbops = &maxi030fb_ops;
	fb_info.screen_base = (char *)maxi030fb_fix.smem_start;
	fb_info.var = maxi030fb_defined;
	fb_info.fix = maxi030fb_fix;
	fb_info.flags = FBINFO_DEFAULT;

	fb_alloc_cmap(&fb_info.cmap, 256, 0);

	if (register_framebuffer(&fb_info) < 0)
		return 1;
		
	return 0;
}

static void __exit maxi030fb_exit(void)
{
	unregister_framebuffer(&fb_info);
}

#ifdef MODULE
MODULE_LICENSE("GPL");
#endif
module_init(maxi030fb_init);
module_exit(maxi030fb_exit);

