/*	$NetBSD: if_urtwn.c,v 1.104 2022/06/23 13:46:27 brook Exp $	*/
/*	$OpenBSD: if_urtwn.c,v 1.42 2015/02/10 23:25:46 mpi Exp $	*/

/*-
 * Copyright (c) 2010 Damien Bergamini <damien.bergamini@free.fr>
 * Copyright (c) 2014 Kevin Lo <kevlo@FreeBSD.org>
 * Copyright (c) 2016 Nathanial Sloss <nathanialsloss@yahoo.com.au>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/* Some code taken from FreeBSD dev/usb/wlan/if_urtw.c with copyright */
/*-
 * Copyright (c) 2008 Weongyo Jeong <weongyo@FreeBSD.org>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/*-
 * Driver for Realtek RTL8188CE-VAU/RTL8188CUS/RTL8188EU/RTL8188RU/RTL8192CU
 * RTL8192EU.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD: if_urtwn.c,v 1.104 2022/06/23 13:46:27 brook Exp $");

#ifdef _KERNEL_OPT
#include "opt_inet.h"
#include "opt_usb.h"
#endif

#include <sys/param.h>
#include <sys/sockio.h>
#include <sys/sysctl.h>
#include <sys/mbuf.h>
#include <sys/kernel.h>
#include <sys/kmem.h>
#include <sys/socket.h>
#include <sys/systm.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/conf.h>
#include <sys/device.h>

#include <sys/bus.h>
#include <machine/endian.h>
#include <sys/intr.h>
#include <sys/sbuf.h>

#include <net/if.h>
#include <net/if_arp.h>
#include <net/if_dl.h>
#include <net/if_ether.h>
#include <net/if_media.h>
#include <net/if_types.h>

#include <netinet/in.h>
#include <netinet/in_systm.h>
#include <netinet/in_var.h>
#include <netinet/ip.h>
#include <netinet/if_inarp.h>

#include <net80211/ieee80211_radiotap.h>
#include <net80211/ieee80211_regdomain.h>
#include <dev/firmload.h>

#include <dev/usb/usbwifi.h>
#include <dev/usb/usbhist.h>

#include <dev/ic/rtwnreg.h>
#include <dev/ic/rtwn_data.h>
#include <dev/usb/if_urtwnreg.h>
#include <dev/usb/if_urtwnvar.h>
#include <net80211/ieee80211_ratectl.h>

#ifdef URTWN_DEBUG
#define DBG_INIT	__BIT(0)
#define DBG_FN		__BIT(1)
#define DBG_TX		__BIT(2)
#define DBG_RX		__BIT(3)
#define DBG_STM 	__BIT(4)
#define DBG_RF		__BIT(5)
#define DBG_REG 	__BIT(6)
#define DBG_ALL 	0xffffffffU
u_int urtwn_debug = DBG_ALL;
#define DPRINTFN(n, fmt, a, b, c, d) do {			\
	if (urtwn_debug & (n)) {				\
		KERNHIST_LOG(usbhist, fmt, a, b, c, d);		\
	}							\
} while (/*CONSTCOND*/0)
#define URTWNHIST_FUNC() USBHIST_FUNC()
#define URTWNHIST_CALLED() do {					\
	if (urtwn_debug & DBG_FN) {				\
		KERNHIST_CALLED(usbhist);			\
	}							\
} while(/*CONSTCOND*/0)
#define URTWNHIST_CALLARGS(fmt, a, b, c, d) do {		\
	if (urtwn_debug & DBG_FN) {				\
		KERNHIST_CALLARGS(usbhist, fmt, a, b, c, d);	\
	}							\
} while(/*CONSTCOND*/0)
#else
#define DPRINTFN(n, fmt, a, b, c, d)
#define URTWNHIST_FUNC()
#define URTWNHIST_CALLED()
#define URTWNHIST_CALLARGS(fmt, a, b, c, d)
#endif

#define URTWN_DEV(v,p)	{ { USB_VENDOR_##v, USB_PRODUCT_##v##_##p }, 0 }
#define URTWN_RTL8188E_DEV(v,p) \
	{ { USB_VENDOR_##v, USB_PRODUCT_##v##_##p }, FLAG_RTL8188E }
#define URTWN_RTL8192EU_DEV(v,p) \
	{ { USB_VENDOR_##v, USB_PRODUCT_##v##_##p }, FLAG_RTL8192E }
#define URTWN_RTL8821AU_DEV(v,p) \
	{ { USB_VENDOR_##v, USB_PRODUCT_##v##_##p }, FLAG_RTL8821A }
#define URTWN_R21AU_RADAR_CHECK_PERIOD	(2 * hz)

static const struct urtwn_dev {
	struct usb_devno	dev;
	uint32_t		flags;
#define FLAG_RTL8188E	__BIT(0)
#define FLAG_RTL8192E	__BIT(1)
#define	FLAG_RTL8821A	__BIT(2)
} urtwn_devs[] = {
	URTWN_DEV(ABOCOM,	RTL8188CU_1),
	URTWN_DEV(ABOCOM,	RTL8188CU_2),
	URTWN_DEV(ABOCOM,	RTL8192CU),
	URTWN_DEV(ASUSTEK,	RTL8192CU),
	URTWN_DEV(ASUSTEK,	RTL8192CU_3),
	URTWN_DEV(ASUSTEK,	USBN10NANO),
	URTWN_DEV(ASUSTEK,	RTL8192CU_3),
	URTWN_DEV(AZUREWAVE,	RTL8188CE_1),
	URTWN_DEV(AZUREWAVE,	RTL8188CE_2),
	URTWN_DEV(AZUREWAVE,	RTL8188CU),
	URTWN_DEV(BELKIN,	F7D2102),
	URTWN_DEV(BELKIN,	RTL8188CU),
	URTWN_DEV(BELKIN,	RTL8188CUS),
	URTWN_DEV(BELKIN,	RTL8192CU),
	URTWN_DEV(BELKIN,	RTL8192CU_1),
	URTWN_DEV(BELKIN,	RTL8192CU_2),
	URTWN_DEV(CHICONY,	RTL8188CUS_1),
	URTWN_DEV(CHICONY,	RTL8188CUS_2),
	URTWN_DEV(CHICONY,	RTL8188CUS_3),
	URTWN_DEV(CHICONY,	RTL8188CUS_4),
	URTWN_DEV(CHICONY,	RTL8188CUS_5),
	URTWN_DEV(CHICONY,	RTL8188CUS_6),
	URTWN_DEV(COMPARE,	RTL8192CU),
	URTWN_DEV(COREGA,	RTL8192CU),
	URTWN_DEV(DLINK,	DWA131B),
	URTWN_DEV(DLINK,	RTL8188CU),
	URTWN_DEV(DLINK,	RTL8192CU_1),
	URTWN_DEV(DLINK,	RTL8192CU_2),
	URTWN_DEV(DLINK,	RTL8192CU_3),
	URTWN_DEV(DLINK,	RTL8192CU_4),
	URTWN_DEV(EDIMAX,	RTL8188CU),
	URTWN_DEV(EDIMAX,	RTL8192CU),
	URTWN_DEV(FEIXUN,	RTL8188CU),
	URTWN_DEV(FEIXUN,	RTL8192CU),
	URTWN_DEV(GUILLEMOT,	HWNUP150),
	URTWN_DEV(GUILLEMOT,	RTL8192CU),
	URTWN_DEV(HAWKING,	RTL8192CU),
	URTWN_DEV(HAWKING,	RTL8192CU_2),
	URTWN_DEV(HP3,		RTL8188CU),
	URTWN_DEV(IODATA,	WNG150UM),
	URTWN_DEV(IODATA,	RTL8192CU),
	URTWN_DEV(NETGEAR,	WNA1000M),
	URTWN_DEV(NETGEAR,	RTL8192CU),
	URTWN_DEV(NETGEAR4,	RTL8188CU),
	URTWN_DEV(NOVATECH,	RTL8188CU),
	URTWN_DEV(PLANEX2,	RTL8188CU_1),
	URTWN_DEV(PLANEX2,	RTL8188CU_2),
	URTWN_DEV(PLANEX2,	RTL8192CU),
	URTWN_DEV(PLANEX2,	RTL8188CU_3),
	URTWN_DEV(PLANEX2,	RTL8188CU_4),
	URTWN_DEV(PLANEX2,	RTL8188CUS),
	URTWN_DEV(REALTEK,	RTL8188CE_0),
	URTWN_DEV(REALTEK,	RTL8188CE_1),
	URTWN_DEV(REALTEK,	RTL8188CTV),
	URTWN_DEV(REALTEK,	RTL8188CU_0),
	URTWN_DEV(REALTEK,	RTL8188CU_1),
	URTWN_DEV(REALTEK,	RTL8188CU_2),
	URTWN_DEV(REALTEK,	RTL8188CU_3),
	URTWN_DEV(REALTEK,	RTL8188CU_COMBO),
	URTWN_DEV(REALTEK,	RTL8188CUS),
	URTWN_DEV(REALTEK,	RTL8188RU),
	URTWN_DEV(REALTEK,	RTL8188RU_2),
	URTWN_DEV(REALTEK,	RTL8188RU_3),
	URTWN_DEV(REALTEK,	RTL8191CU),
	URTWN_DEV(REALTEK,	RTL8192CE),
	URTWN_DEV(REALTEK,	RTL8192CU),
	URTWN_DEV(SITECOMEU,	RTL8188CU),
	URTWN_DEV(SITECOMEU,	RTL8188CU_2),
	URTWN_DEV(SITECOMEU,	RTL8192CU),
	URTWN_DEV(SITECOMEU,	RTL8192CUR2),
	URTWN_DEV(TPLINK,	RTL8192CU),
	URTWN_DEV(TRENDNET,	RTL8188CU),
	URTWN_DEV(TRENDNET,	RTL8192CU),
	URTWN_DEV(TRENDNET,	TEW648UBM),
	URTWN_DEV(ZYXEL,	RTL8192CU),

	/* URTWN_RTL8188E */
	URTWN_RTL8188E_DEV(DLINK, DWA125D1),
	URTWN_RTL8188E_DEV(ELECOM, WDC150SU2M),
	URTWN_RTL8188E_DEV(REALTEK, RTL8188ETV),
	URTWN_RTL8188E_DEV(REALTEK, RTL8188EU),
	URTWN_RTL8188E_DEV(ABOCOM, RTL8188EU),
	URTWN_RTL8188E_DEV(TPLINK, RTL8188EU),
	URTWN_RTL8188E_DEV(DLINK, DWA121B1),
	URTWN_RTL8188E_DEV(EDIMAX, EW7811UNV2),

	/* URTWN_RTL8192EU */
	URTWN_RTL8192EU_DEV(DLINK,	DWA131E),
	URTWN_RTL8192EU_DEV(REALTEK,	RTL8192EU),
	URTWN_RTL8192EU_DEV(TPLINK,	WN821NV5),
	URTWN_RTL8192EU_DEV(TPLINK,	WN822NV4),
	URTWN_RTL8192EU_DEV(TPLINK,	WN823NV2),

	/* RTL8821AU */
	URTWN_RTL8821AU_DEV(DLINK,		DWA171A1),
	URTWN_RTL8821AU_DEV(DLINK,		DWA172A1),
	URTWN_RTL8821AU_DEV(EDIMAX,		EW7811UTC_1),
	URTWN_RTL8821AU_DEV(EDIMAX,		EW7811UTC_2),
	URTWN_RTL8821AU_DEV(ELECOM,		WDB433SU2M2),
	URTWN_RTL8821AU_DEV(HAWKING,		HD65U),
	URTWN_RTL8821AU_DEV(MELCO,		WIU2433DM),
	URTWN_RTL8821AU_DEV(MELCO,		WIU2433DHP),
	URTWN_RTL8821AU_DEV(NETGEAR,		A6100),
	URTWN_RTL8821AU_DEV(REALTEK,		RTL8821AU_1),
	URTWN_RTL8821AU_DEV(REALTEK,		RTL8821AU_2),
	URTWN_RTL8821AU_DEV(TPLINK,		T2UNANO),
	URTWN_RTL8821AU_DEV(TPLINK,		T2UPLUS),
	URTWN_RTL8821AU_DEV(TPLINK,		T2UV3),
};
#undef URTWN_DEV
#undef URTWN_RTL8188E_DEV
#undef URTWN_RTL8192EU_DEV
#undef URTWN_RTL8821AU_DEV

static int	urtwn_match(device_t, cfdata_t, void *);
static void	urtwn_attach(device_t, device_t, void *);
static int	urtwn_detach(device_t, int);
static void	urtwn_rx_loop(struct usbwifi *, struct usbwifi_chain *,
		    uint32_t);
static unsigned	urtwn_tx_prepare(struct usbwifi *,
				struct usbwifi_chain *,
				uint8_t qid);
static unsigned	urtwn_r88e_tx_prepare(struct usbwifi *,
				struct usbwifi_chain *,
				uint8_t qid);
static unsigned	urtwn_r21a_tx_prepare(struct usbwifi *,
				struct usbwifi_chain *,
				uint8_t qid);
static int	urtwn_init(struct usbwifi *);
static void	urtwn_stop(struct usbwifi *);
static void urtwn_postattach(struct urtwn_softc *);
static void urtwn_r21a_postattach(struct urtwn_softc *);

CFATTACH_DECL_NEW(urtwn, sizeof(struct urtwn_softc), urtwn_match,
    urtwn_attach, urtwn_detach, usbwifi_activate);

static const struct usbwifi_ops urtwn_ops = {
	.uwo_rx_loop = urtwn_rx_loop,
	.uwo_tx_prepare = urtwn_tx_prepare,
	.uwo_init = urtwn_init,
	.uwo_stop = urtwn_stop,
};

static int	urtwn_configure_pipes(struct urtwn_softc*, int*, int*);
static void	urtwn_task(void *);
static void	urtwn_do_async(struct urtwn_softc *,
			void (*)(struct urtwn_softc *, void *), void *, int);
//static void	urtwn_wait_async(struct urtwn_softc *);
static int	urtwn_write_region_1(struct urtwn_softc *, uint16_t, uint8_t *,
		    int);
static void	urtwn_write_1(struct urtwn_softc *, uint16_t, uint8_t);
static void	urtwn_write_2(struct urtwn_softc *, uint16_t, uint16_t);
static void	urtwn_write_4(struct urtwn_softc *, uint16_t, uint32_t);
static int	urtwn_write_region(struct urtwn_softc *, uint16_t, uint8_t *,
		    int);
static void	urtwn_setbits_1(struct urtwn_softc *, uint16_t, uint8_t,
    uint8_t);
static void	urtwn_setbits_1_shift(struct urtwn_softc *, uint16_t, uint32_t ,
    uint32_t, int);
static void urtwn_setbits_4(struct urtwn_softc *, uint16_t, uint32_t,
    uint32_t);
static int	urtwn_read_region_1(struct urtwn_softc *, uint16_t, uint8_t *,
		    int);
static uint8_t	urtwn_read_1(struct urtwn_softc *, uint16_t);
static uint16_t urtwn_read_2(struct urtwn_softc *, uint16_t);
static uint32_t urtwn_read_4(struct urtwn_softc *, uint16_t);
static int	urtwn_fw_cmd(struct urtwn_softc *, uint8_t, const void *, int);
static void	urtwn_r92c_rf_write(struct urtwn_softc *, int, uint8_t,
		    uint32_t);
static void	urtwn_r88e_rf_write(struct urtwn_softc *, int, uint8_t,
		    uint32_t);
static void	urtwn_r92e_rf_write(struct urtwn_softc *, int, uint8_t,
		    uint32_t);
static void	urtwn_r21a_rf_write(struct urtwn_softc *, int, uint8_t,
		    uint32_t);
static uint32_t urtwn_rf_read(struct urtwn_softc *, int, uint8_t);
static uint32_t urtwn_r92e_rf_read(struct urtwn_softc *, int, uint8_t);
static uint32_t urtwn_r21a_rf_read(struct urtwn_softc *, int, uint8_t);
static int	urtwn_llt_write(struct urtwn_softc *, uint32_t, uint32_t);
static uint8_t	urtwn_efuse_read_1(struct urtwn_softc *, uint16_t);
static void	urtwn_efuse_read(struct urtwn_softc *);
static void	urtwn_efuse_switch_power(struct urtwn_softc *);
static int	urtwn_read_chipid(struct urtwn_softc *);
#ifdef URTWN_DEBUG
static void	urtwn_dump_rom(struct urtwn_softc *, struct r92c_rom *);
#endif
static void	urtwn_read_rom(struct urtwn_softc *);
static void	urtwn_r88e_read_rom(struct urtwn_softc *);
static void urtwn_r21a_read_rom(struct urtwn_softc *);
static int	urtwn_ra_init(struct ieee80211vap *);
static int	urtwn_get_nettype(struct urtwn_softc *);
static void	urtwn_set_nettype0_msr(struct urtwn_softc *, uint8_t);
static void	urtwn_tsf_sync_enable(struct urtwn_softc *, struct ieee80211_node *);
static void	urtwn_set_led(struct urtwn_softc *, int, int);
static void	urtwn_calib_to(void *);
static void	urtwn_calib_to_cb(struct urtwn_softc *, void *);
static int	urtwn_newstate(struct ieee80211vap *, enum ieee80211_state,
		    int);
static int urtwn_r21au_newstate(struct ieee80211vap *, enum ieee80211_state,
		    int);
static int	urtwn_wme_update(struct ieee80211com *);
static void	urtwn_wme_update_cb(struct urtwn_softc *, void *);
static void	urtwn_update_avgrssi(struct urtwn_softc *, int, int8_t);
static int8_t	urtwn_get_rssi(struct urtwn_softc *, int, void *);
static int8_t	urtwn_r88e_get_rssi(struct urtwn_softc *, int, void *);
static int8_t	urtwn_r21a_get_rssi(struct urtwn_softc *, int, void *);
static void	urtwn_rx_frame(struct urtwn_softc *, uint8_t *, int);
static void	urtwn_watchdog(void*);
static int	urtwn_r92c_power_on(struct urtwn_softc *);
static int	urtwn_r92e_power_on(struct urtwn_softc *);
static int	urtwn_r88e_power_on(struct urtwn_softc *);
static int	urtwn_r21a_power_on(struct urtwn_softc *);
static void	urtwn_r21a_power_off(struct urtwn_softc *);
static int	urtwn_llt_init(struct urtwn_softc *);
static void	urtwn_fw_reset(struct urtwn_softc *);
static void	urtwn_r88e_fw_reset(struct urtwn_softc *);
static void urtwn_r21a_fw_reset(struct urtwn_softc *);
static int	urtwn_fw_loadpage(struct urtwn_softc *, int, uint8_t *, int);
static int	urtwn_load_firmware(struct urtwn_softc *);
static int	urtwn_r92c_dma_init(struct urtwn_softc *);
static int	urtwn_r88e_dma_init(struct urtwn_softc *);
static int 	urtwn_r21a_dma_init(struct urtwn_softc *);
static void	urtwn_mac_init(struct urtwn_softc *);
static void urtwn_mrr_init(struct urtwn_softc *);
static void	urtwn_bb_init(struct urtwn_softc *);
static void	urtwn_rf_init(struct urtwn_softc *);
static void	urtwn_cam_init(struct urtwn_softc *);
static void	urtwn_pa_bias_init(struct urtwn_softc *);
static void	urtwn_rxfilter_init(struct urtwn_softc *);
static void	urtwn_edca_init(struct urtwn_softc *);
static void	urtwn_write_txpower(struct urtwn_softc *, int, uint16_t[]);
static void urtwn_r21a_write_txpower(struct urtwn_softc *, struct ieee80211_channel *, int, uint16_t[]);
static void	urtwn_get_txpower(struct urtwn_softc *, size_t, u_int, u_int,
		    uint16_t[]);
static void	urtwn_r88e_get_txpower(struct urtwn_softc *, size_t, u_int,
		    u_int, uint16_t[]);
static void	urtwn_set_txpower(struct urtwn_softc *, u_int, u_int);
static void urtwn_r21a_set_txpower(struct urtwn_softc *, struct ieee80211_channel *);
static void urtwn_r21a_get_txpower(struct urtwn_softc *, struct ieee80211_channel *, int, uint16_t[]);
static void	urtwn_set_chan(struct urtwn_softc *, struct ieee80211_channel *);
static void urtwn_r21a_set_chan(struct urtwn_softc *, struct ieee80211_channel *);
static void urtwn_r21a_set_band_2ghz(struct urtwn_softc *, uint32_t);
static void urtwn_r21a_set_band_5ghz(struct urtwn_softc *, uint32_t);
static void urtwn_r21a_set_band(struct urtwn_softc *, struct ieee80211_channel *,
			struct ieee80211com *);
static void urtwn_get_rates(struct urtwn_softc *, const struct ieee80211_rateset *,
    		const struct ieee80211_htrateset *, uint32_t *, int *, int);
static void urtwn_set_basicrates(struct urtwn_softc *, uint32_t);
static void urtwn_r21a_bypass_ext_lna_2ghz(struct urtwn_softc *);
static void	urtwn_iq_calib(struct urtwn_softc *, bool);
static void urtwn_r21a_iq_calib(struct urtwn_softc *, bool);
static void urtwn_r21a_iq_calib_sw(struct urtwn_softc *);
static void urtwn_r21a_iq_calib_fw(struct urtwn_softc *);
static int urtwn_r21a_iq_calib_fw_supported(struct urtwn_softc *);
static void	urtwn_lc_calib(struct urtwn_softc *);
static void	urtwn_temp_calib(struct urtwn_softc *);
static void	urtwn_newassoc(struct ieee80211_node *, int);
static void	urtwn_delay_ms(struct urtwn_softc *, int ms);
/* Functions for wifi refresh */
static struct ieee80211vap *
		urtwn_vap_create(struct ieee80211com *,
		    const char [IFNAMSIZ], int, enum ieee80211_opmode, int,
		    const uint8_t [IEEE80211_ADDR_LEN],
		    const uint8_t [IEEE80211_ADDR_LEN]);
static void	urtwn_vap_delete(struct ieee80211vap *);
static void urtwn_vap_preattach(struct urtwn_softc *, struct ieee80211vap *);
static void urtwn_r21au_vap_preattach(struct urtwn_softc *, struct ieee80211vap *);
static void	urtwn_get_radiocaps(struct ieee80211com *, int, int *,
		    struct ieee80211_channel chans[]);
static void	urtwn_scan_start(struct ieee80211com *);
static void	urtwn_scan_end(struct ieee80211com *);
static void urtwn_r21a_scan_start(struct ieee80211com *);
static void urtwn_r21a_scan_end(struct ieee80211com *);
static void	urtwn_set_channel(struct ieee80211com *);
static void	urtwn_update_mcast(struct ieee80211com *);
static void urtwn_r21au_init_ampdu(struct urtwn_softc *);
static void urtwn_r21au_arfb_init(struct urtwn_softc *);
static int 	urtwn_r21a_check_condition(struct urtwn_softc *,const uint8_t[]);
static int  urtwn_r92c_check_condition(struct urtwn_softc *, const uint8_t[]);
static void urtwn_r21au_chan_check(void *, int);
static void urtwn_r21au_dfs_radar_disable(struct urtwn_softc *);
static int urtwn_r21au_dfs_radar_enable(struct urtwn_softc *);
static int urtwn_r21au_dfs_radar_is_enabled(struct urtwn_softc *);
static int urtwn_r21au_dfs_radar_reset(struct urtwn_softc *);
static int urtwn_r21au_dfs_radar_is_detected(struct urtwn_softc *);
static void urtwn_set_media_status(struct urtwn_softc *sc, int macid);
static void urtwn_r21a_set_media_status(struct urtwn_softc *sc, int macid);

static void	power_control(struct urtwn_softc *, bool);
/* Aliases. */
#define	urtwn_bb_write	urtwn_write_4
#define	urtwn_bb_read	urtwn_read_4

#define urtwn_bb_setbits urtwn_setbits_4

static __inline void
urtwn_nop_softc(struct urtwn_softc *sc)
{
}

static __inline int
urtwn_nop_int_softc(struct urtwn_softc *sc)
{
	return (0);
}

static __inline void
urtwn_nop_vappreattach(struct urtwn_softc *sc, struct ieee80211vap *vap)
{
}

static __inline void
urtwn_nop_softc_int(struct urtwn_softc *sc, int id)
{
}

#define urtwn_lookup(d,v,p)	((const struct urtwn_dev *)usb_lookup(d,v,p))

static const uint16_t addaReg[] = {
	R92C_FPGA0_XCD_SWITCHCTL, R92C_BLUETOOTH, R92C_RX_WAIT_CCA,
	R92C_TX_CCK_RFON, R92C_TX_CCK_BBON, R92C_TX_OFDM_RFON,
	R92C_TX_OFDM_BBON, R92C_TX_TO_RX, R92C_TX_TO_TX, R92C_RX_CCK,
	R92C_RX_OFDM, R92C_RX_WAIT_RIFS, R92C_RX_TO_RX,
	R92C_STANDBY, R92C_SLEEP, R92C_PMPD_ANAEN
};

/*
 * We ovveride the VAP's newstate method, so need to save the old
 * function pointer for each VAP.
 */
struct urtwn_vap {
	struct ieee80211vap vap;
	int (*newstate)(struct ieee80211vap *, enum ieee80211_state, int);
};

static int
urtwn_match(device_t parent, cfdata_t match, void *aux)
{
	struct usb_attach_arg *uaa = aux;

	return urtwn_lookup(urtwn_devs, uaa->uaa_vendor, uaa->uaa_product) !=
	    NULL ?  UMATCH_VENDOR_PRODUCT : UMATCH_NONE;
}

static void
urtwn_r21a_postattach(struct urtwn_softc *sc){
	struct ieee80211com *ic = usbwifi_ic(&sc->sc_uw);
	struct urtwn_r21a_data *r21a_data = sc->sc_chip_priv.data;

	char name[MAXCOMLEN];
	snprintf(name, sizeof name, "%swq", ic->ic_name);
	name[sizeof name - 1] = 0;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();
	DPRINTFN(DBG_FN, "WQ_NAME = %s", name, 0,0,0);
	TIMEOUT_TASK_INIT(sc->sc_tq, &r21a_data->chan_check, 0,
	    urtwn_r21au_chan_check, sc);
}

static void
urtwn_attach(device_t parent, device_t self, void *aux)
{
	struct urtwn_softc *sc = device_private(self);
	struct ieee80211com *ic = usbwifi_ic(&sc->sc_uw);
	struct usb_attach_arg *uaa = aux;
	char *devinfop;
	const struct urtwn_dev *dev;
	usb_device_request_t req;
	// NNN loop below size_t i;
	int error, num_rx, num_tx;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	/*
	 * Generic usbwifi(9) setup
	 */
	sc->sc_uw.uw_sc = sc;
	sc->sc_uw.uw_dev = self;
	sc->sc_uw.uw_udev = uaa->uaa_device;
	sc->sc_uw.uw_ops = &urtwn_ops;
	sc->sc_uw.uw_rx_bufsz = URTWN_RXBUFSZ;	/* size of a single buffer */
	sc->sc_uw.uw_tx_bufsz = URTWN_TXBUFSZ;	/* for RX/TX */
	sc->sc_uw.uw_tx_list_cnt = URTWN_TX_LIST_COUNT;	/* max RX/TX buffers */
	sc->sc_uw.uw_rx_list_cnt = URTWN_RX_LIST_COUNT;	/* in the rings */

	dev = urtwn_lookup(urtwn_devs, uaa->uaa_vendor, uaa->uaa_product);
	if (dev != NULL && ISSET(dev->flags, FLAG_RTL8188E))
		SET(sc->chip, URTWN_CHIP_88E);
	if (dev != NULL && ISSET(dev->flags, FLAG_RTL8192E))
		SET(sc->chip, URTWN_CHIP_92EU);
	if (dev != NULL && ISSET(dev->flags, FLAG_RTL8821A)) {
		SET(sc->chip, URTWN_CHIP_21A);
		sc->sc_chip_priv.data = malloc(sizeof(struct urtwn_r21a_data), M_DEVBUF, M_WAITOK | M_ZERO);
		if(sc->sc_chip_priv.data == NULL)
			goto fail;
	}

	aprint_naive("\n");
	aprint_normal("\n");

	devinfop = usbd_devinfo_alloc(sc->sc_uw.uw_udev, 0);
	aprint_normal_dev(self, "%s\n", devinfop);
	usbd_devinfo_free(devinfop);

	req.bmRequestType = UT_WRITE_DEVICE;
	req.bRequest = UR_SET_FEATURE;
	USETW(req.wValue, UF_DEVICE_REMOTE_WAKEUP);
	USETW(req.wIndex, UHF_PORT_SUSPEND);
	USETW(req.wLength, 0);

	(void) usbd_do_request(sc->sc_uw.uw_udev, &req, 0);

	cv_init(&sc->sc_task_cv, "urtwntsk");
	mutex_init(&sc->sc_task_mtx, MUTEX_DEFAULT, IPL_NET);

	usbwifi_attach(&sc->sc_uw);

	/* Override default settings */
	sc->sc_uw.uw_tx_xfer_timeout = URTWN_TX_TIMEOUT;

	/* NNN make these callouts use a vap ... in vap create??? */
	callout_init(&sc->sc_calib_to, CALLOUT_MPSAFE);
	callout_setfunc(&sc->sc_calib_to, urtwn_calib_to, sc);
	callout_init(&sc->sc_watchdog_to, CALLOUT_MPSAFE);
	callout_setfunc(&sc->sc_watchdog_to, urtwn_watchdog, sc);

	if (workqueue_create(&sc->sc_tq, "urtwnwq",
	    ieee80211_runwork, ic, PRI_SOFTNET, IPL_NET, WQ_MPSAFE)){
		aprint_error_dev(self, "workqueue not created");
		goto fail;
	}

	error = usbd_set_config_no(sc->sc_uw.uw_udev, 1, 0);
	if (error != 0) {
		aprint_error_dev(self, "failed to set configuration"
		    ", err=%s\n", usbd_errstr(error));
		goto fail;
	}

	/* Get the first interface handle. */
	error = usbd_device2interface_handle(sc->sc_uw.uw_udev, 0,
	    &sc->sc_uw.uw_iface);
	if (error != 0) {
		aprint_error_dev(self, "could not get interface handle\n");
		goto fail;
	}

	error = urtwn_read_chipid(sc);
	if (error != 0) {
		aprint_error_dev(self, "unsupported test chip\n");
		goto fail;
	}

	/* Determine number of Tx/Rx chains. */
	if (sc->chip & URTWN_CHIP_92C) {
		sc->ntxchains = (sc->chip & URTWN_CHIP_92C_1T2R) ? 1 : 2;
		sc->nrxchains = 2;
	} else if (sc->chip & URTWN_CHIP_92EU) {
		sc->ntxchains = 2;
		sc->nrxchains = 2;
	} else {
		sc->ntxchains = 1;
		sc->nrxchains = 1;
	}

	if (ISSET(sc->chip, URTWN_CHIP_88E) ||
	    ISSET(sc->chip, URTWN_CHIP_92EU))
		urtwn_r88e_read_rom(sc);
	else if (ISSET(sc->chip, URTWN_CHIP_21A))
		urtwn_r21a_read_rom(sc);
	else
		urtwn_read_rom(sc);

	aprint_normal_dev(self, "MAC/BB RTL%s, RF 6052 %zdT%zdR, address %s\n",
	    (sc->chip & URTWN_CHIP_92EU) ? "8192EU" :
	    (sc->chip & URTWN_CHIP_92C) ? "8192CU" :
	    (sc->chip & URTWN_CHIP_88E) ? "8188EU" :
	    (sc->chip & URTWN_CHIP_21A) ? "8821AU" :
	    (sc->board_type == R92C_BOARD_TYPE_HIGHPA) ? "8188RU" :
	    (sc->board_type == R92C_BOARD_TYPE_MINICARD) ? "8188CE-VAU" :
	    "8188CUS", sc->ntxchains, sc->nrxchains,
	    ether_sprintf(ic->ic_macaddr));

	error = urtwn_configure_pipes(sc, &num_tx, &num_rx);
	if (error != 0) {
		aprint_error_dev(sc->sc_uw.uw_dev,
		    "could not confiugre pipes\n");
		goto fail;
	}
	aprint_normal_dev(self, "%d rx pipe%s, %d tx pipe%s\n",
	    num_rx, num_rx > 1 ? "s" : "",
	    num_tx, num_tx > 1 ? "s" : "");

	/* use usbwifi tick instead? */
	usb_init_task(&sc->sc_task, urtwn_task, sc, 0);

	/* Set device capabilities. */
	ic->ic_caps =
	    IEEE80211_C_STA |		/* station mode */
	    IEEE80211_C_MONITOR |	/* Monitor mode supported. */
	    IEEE80211_C_IBSS |		/* IBSS mode supported */
	    IEEE80211_C_HOSTAP |	/* HostAp mode supported */
	    IEEE80211_C_SHPREAMBLE |	/* Short preamble supported. */
	    IEEE80211_C_SHSLOT |	/* Short slot time supported. */
	    IEEE80211_C_BGSCAN |	/* capable of bg scanning */
//	    IEEE80211_C_SWSLEEP|	/* handle sleeps */
	    IEEE80211_C_WME |		/* 802.11e */
	    IEEE80211_C_WPA;		/* 802.11i */

	ic->ic_htcaps =
	    IEEE80211_HTC_HT |
	    IEEE80211_HTCAP_SHORTGI20 | 	/* short GI in 20MHz */
#if 1
	    IEEE80211_HTCAP_MAXAMSDU_3839 |	/* max A-MSDU length */
#endif
	    IEEE80211_HTCAP_SMPS_OFF |		/* SM PS mode disabled */
	    IEEE80211_HTCAP_CHWIDTH40 | 	/* 40 MHz channel width */
	    IEEE80211_HTCAP_SHORTGI40 |	/* short GI in 40MHz */
	    IEEE80211_HTC_AMPDU	| 	/* A-MPDU tx */
	    IEEE80211_HTC_AMSDU;	/* A-MSDU tx */

	if (ISSET(sc->chip, URTWN_CHIP_21A))
		ic->ic_htcaps |= IEEE80211_HTC_TXLDPC;
#ifdef notyet
	ic->ic_cryptocaps =
		IEEE80211_CRYPTO_WEP |
		IEEE80211_CRYPTO_TKIP |
		IEEE80211_CRYPTO_AES_CCM;
#else
	ic->ic_cryptocaps = IEEE80211_CRYPTO_WEP |
		IEEE80211_CRYPTO_TKIP |
		IEEE80211_CRYPTO_AES_CCM;
#endif
	ic->ic_txstream = sc->ntxchains;
	ic->ic_rxstream = sc->nrxchains;

	/* XXX TODO: setup regdomain if URTW_EPROM_CHANPLAN_BY_HW bit is set.*/
	urtwn_get_radiocaps(ic, IEEE80211_CHAN_MAX, &ic->ic_nchans,
	    ic->ic_channels);

	/*
	 * Initialize the global (non-VAP specific) structures and create
	 * the VAP list.
	 */
	usbwifi_ic_attach(&sc->sc_uw, sc->ntxchains, sc->nrxchains, num_tx,
	   num_rx, IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST);

	/* override default methods */
	ic->ic_newassoc = urtwn_newassoc;
	ic->ic_wme.wme_update = urtwn_wme_update;
	ic->ic_vap_create = urtwn_vap_create;
	ic->ic_vap_delete = urtwn_vap_delete;
	
	if (ISSET(sc->chip, URTWN_CHIP_21A))
		ic->ic_scan_start = urtwn_r21a_scan_start;
	else
		ic->ic_scan_start = urtwn_scan_start;
	
	if (ISSET(sc->chip, URTWN_CHIP_21A))
		ic->ic_scan_end = urtwn_r21a_scan_end;
	else
		ic->ic_scan_end = urtwn_scan_end;
	
	ic->ic_getradiocaps = urtwn_get_radiocaps;
	ic->ic_set_channel = urtwn_set_channel;
	ic->ic_update_mcast = urtwn_update_mcast;

	urtwn_postattach(sc);

	sc->sc_rxtap.wr_ihdr.it_len = htole16(sizeof(sc->sc_rxtapu));
	sc->sc_rxtap.wr_ihdr.it_present = htole32(URTWN_RX_RADIOTAP_PRESENT);

	sc->sc_txtap_len = sizeof(sc->sc_txtapu);
	sc->sc_txtap.wt_ihdr.it_len = htole16(sc->sc_txtap_len);
	sc->sc_txtap.wt_ihdr.it_present = htole32(URTWN_TX_RADIOTAP_PRESENT);

	/* let the stack know we support radiotap */
	ic->ic_rh = &sc->sc_rxtapu.th.wr_ihdr;
	ic->ic_th = &sc->sc_txtapu.th.wt_ihdr;

	usbwifi_attach_finalize(&sc->sc_uw);
	return;

fail:
	aprint_error_dev(self, "attach failed\n");
}

static int
urtwn_detach(device_t self, int flags)
{
	struct urtwn_softc *sc = device_private(self);
	int err;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	err = usbwifi_detach(self, flags);
	if (err)
		return err;

	callout_halt(&sc->sc_calib_to, NULL);
	if (sc->sc_uw.uw_pri != NULL)
		usb_rem_task_wait(sc->sc_uw.uw_udev, &sc->sc_task,
		    USB_TASKQ_DRIVER, NULL);
	callout_destroy(&sc->sc_calib_to);
	cv_destroy(&sc->sc_task_cv);
	mutex_destroy(&sc->sc_task_mtx);
	return 0;
}

static int
urtwn_configure_pipes(struct urtwn_softc *sc, int *num_tx, int *num_rx)
{
	/* Bulk-out endpoints addresses (from highest to lowest prio). */
	static uint8_t epaddr[URTWN_MAX_EPOUT];
	static uint8_t rxepaddr[URTWN_MAX_EPIN];
	usb_interface_descriptor_t *id;
	usb_endpoint_descriptor_t *ed;
	size_t i, ntx = 0, nrx = 0;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	/* Determine the number of bulk-out pipes. */
	id = usbd_get_interface_descriptor(sc->sc_uw.uw_iface);
	for (i = 0; i < id->bNumEndpoints; i++) {
		ed = usbd_interface2endpoint_descriptor(sc->sc_uw.uw_iface, i);
		if (ed == NULL ||
		    UE_GET_XFERTYPE(ed->bmAttributes) != UE_BULK) {
			continue;
		}
		if (UE_GET_DIR(ed->bEndpointAddress) == UE_DIR_OUT) {
			if (ntx < sizeof(epaddr))
				epaddr[ntx] = ed->bEndpointAddress;
			ntx++;
		}
		if (UE_GET_DIR(ed->bEndpointAddress) == UE_DIR_IN) {
			if (nrx < sizeof(rxepaddr))
				rxepaddr[nrx] = ed->bEndpointAddress;
			nrx++;
		}
	}
	if (nrx == 0 || nrx > URTWN_MAX_EPIN) {
		aprint_error_dev(sc->sc_uw.uw_dev,
		    "%zd: invalid number of Rx bulk pipes\n", nrx);
		return EIO;
	}
	if (ntx == 0 || ntx > URTWN_MAX_EPOUT) {
		aprint_error_dev(sc->sc_uw.uw_dev,
		    "%zd: invalid number of Tx bulk pipes\n", ntx);
		return EIO;
	}
	if ((nrx + ntx) >= USBWIFI_ENDPT_MAX) {
		aprint_error_dev(sc->sc_uw.uw_dev,
		    "%zd: too many bulk pipes\n", ntx+nrx);
		return EIO;
	}

	DPRINTFN(DBG_INIT, "found %jd/%jd bulk-in/out pipes",
	    nrx, ntx, 0, 0);
	*num_rx = nrx;
	*num_tx = ntx;
	sc->ntx = ntx;
	memcpy(&sc->sc_uw.uw_ed[0], epaddr, sizeof(epaddr[0])*ntx);
	memcpy(&sc->sc_uw.uw_ed[ntx], rxepaddr, sizeof(rxepaddr[0])*nrx);

	/* Map 802.11 access categories to USB pipes. */
	sc->sc_uw.uw_ac2idx[WME_AC_BK] =
	sc->sc_uw.uw_ac2idx[WME_AC_BE] = (ntx >= 3) ? 2 : ((ntx == 2) ? 1 : 0);
	sc->sc_uw.uw_ac2idx[WME_AC_VI] = (ntx >= 3) ? 1 : 0;
	sc->sc_uw.uw_ac2idx[WME_AC_VO] = 0;	/* Always use highest prio. */

	return 0;
}

static void
urtwn_update_mcast(struct ieee80211com *ic)
{
	printf("urtwn_update_mcast: empty implementation\n");
}

static void
urtwn_task(void *arg)
{
	struct urtwn_softc *sc = arg;
	struct ieee80211com *ic = usbwifi_ic(&sc->sc_uw);
	struct ieee80211vap *vap;
	struct urtwn_host_cmd_ring *ring = &sc->cmdq;
	struct urtwn_host_cmd *cmd;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	usbwifi_lock_ic(&sc->sc_uw);
	/* handling beacon frames here is way too expensive! */
	TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) {
		if (vap->iv_state != IEEE80211_S_RUN ||
		    (vap->iv_opmode != IEEE80211_M_HOSTAP &&
		    vap->iv_opmode != IEEE80211_M_IBSS &&
		    vap->iv_opmode != IEEE80211_M_MBSS))
			continue;

		/* Send a beacon frame. */
		struct mbuf *m = ieee80211_beacon_alloc(vap->iv_bss);
		if (m == NULL) {
			aprint_error_dev(sc->sc_uw.uw_dev,
			    "could not allocate beacon");
		}
		if (ic->ic_raw_xmit(vap->iv_bss, m, NULL) != 0)
			aprint_error_dev(sc->sc_uw.uw_dev,
			    "could not send beacon\n");
	}
	usbwifi_unlock_ic(&sc->sc_uw);

	/* Process host commands. */
	mutex_spin_enter(&sc->sc_task_mtx);
	while (ring->next != ring->cur) {
		cmd = &ring->cmd[ring->next];
		ring->queued--;
		ring->next = (ring->next + 1) % URTWN_HOST_CMD_RING_COUNT;
		mutex_spin_exit(&sc->sc_task_mtx);
		/* Invoke callback with kernel lock held. */
		cmd->cb(sc, cmd->data);
		mutex_spin_enter(&sc->sc_task_mtx);
	}
	cv_broadcast(&sc->sc_task_cv);
	mutex_spin_exit(&sc->sc_task_mtx);
}

static void
urtwn_do_async(struct urtwn_softc *sc,
    void (*cb)(struct urtwn_softc*, void *),
    void *arg, int len)
{
	struct urtwn_host_cmd_ring *ring = &sc->cmdq;
	struct urtwn_host_cmd *cmd;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();
	DPRINTFN(DBG_FN, "cb=%jd, arg=%jd, len=%jd", (intptr_t)cb,
	    (intptr_t)arg, len, 0);

	mutex_spin_enter(&sc->sc_task_mtx);
	cmd = &ring->cmd[ring->cur];
	cmd->cb = cb;
	KASSERT(len <= sizeof(cmd->data));
	memcpy(cmd->data, arg, len);
	ring->cur = (ring->cur + 1) % URTWN_HOST_CMD_RING_COUNT;

	/* If there is no pending command already, schedule a task. */
	if (!usbwifi_isdying(&sc->sc_uw) && ++ring->queued == 1) {
		mutex_spin_exit(&sc->sc_task_mtx);
		usb_add_task(sc->sc_uw.uw_udev, &sc->sc_task, USB_TASKQ_DRIVER);
	} else
		mutex_spin_exit(&sc->sc_task_mtx);
}

__unused static void
urtwn_wait_async(struct urtwn_softc *sc)
{

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	/* Wait for all queued asynchronous commands to complete. */
	mutex_spin_enter(&sc->sc_task_mtx);
	while (sc->cmdq.queued > 0)
		cv_wait(&sc->sc_task_cv, &sc->sc_task_mtx);
	mutex_spin_exit(&sc->sc_task_mtx);
}

static int
urtwn_write_region_1(struct urtwn_softc *sc, uint16_t addr, uint8_t *buf,
    int len)
{
	usb_device_request_t req;
	usbd_status error;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();
	usbwifi_isowned_ic(&sc->sc_uw);

	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = R92C_REQ_REGS;
	USETW(req.wValue, addr);
	USETW(req.wIndex, 0);
	USETW(req.wLength, len);
	error = usbd_do_request(sc->sc_uw.uw_udev, &req, buf);
	if (error != USBD_NORMAL_COMPLETION) {
		DPRINTFN(DBG_REG, "error=%jd: addr=%#jx, len=%jd",
		    error, (intptr_t)addr, len, 0);
	}
	return error;
}

static void
urtwn_write_1(struct urtwn_softc *sc, uint16_t addr, uint8_t val)
{

	URTWNHIST_FUNC(); URTWNHIST_CALLED();
	DPRINTFN(DBG_REG, "addr=%#jx, val=%#jx", (intptr_t)addr, val, 0, 0);

	urtwn_write_region_1(sc, addr, &val, 1);
}

static void
urtwn_write_2(struct urtwn_softc *sc, uint16_t addr, uint16_t val)
{
	uint8_t buf[2];

	URTWNHIST_FUNC(); URTWNHIST_CALLED();
	DPRINTFN(DBG_REG, "addr=%#jx, val=%#jx", (intptr_t)addr, val, 0, 0);

	buf[0] = (uint8_t)val;
	buf[1] = (uint8_t)(val >> 8);
	urtwn_write_region_1(sc, addr, buf, 2);
}

static void
urtwn_write_4(struct urtwn_softc *sc, uint16_t addr, uint32_t val)
{
	uint8_t buf[4];

	URTWNHIST_FUNC(); URTWNHIST_CALLED();
	DPRINTFN(DBG_REG, "addr=%#jx, val=%#jx", (intptr_t)addr, val, 0, 0);

	buf[0] = (uint8_t)val;
	buf[1] = (uint8_t)(val >> 8);
	buf[2] = (uint8_t)(val >> 16);
	buf[3] = (uint8_t)(val >> 24);
	urtwn_write_region_1(sc, addr, buf, 4);
}

static __inline void
urtwn_setbits_4(struct urtwn_softc *sc, uint16_t addr, uint32_t clr,
    uint32_t set)
{
	return (urtwn_write_4(sc, addr,
	    (urtwn_read_4(sc, addr) & ~clr) | set));
}

static __inline void
urtwn_setbits_2(struct urtwn_softc *sc, uint16_t addr, uint32_t clr,
    uint32_t set)
{
	return (urtwn_write_2(sc, addr,
	    (urtwn_read_2(sc, addr) & ~clr) | set));
}

static int
urtwn_write_region(struct urtwn_softc *sc, uint16_t addr, uint8_t *buf,
    int len)
{

	URTWNHIST_FUNC(); URTWNHIST_CALLED();
	DPRINTFN(DBG_REG, "addr=%#jx, len=%#jx", (intptr_t)addr, len, 0, 0);

	return urtwn_write_region_1(sc, addr, buf, len);
}

/*
 * Methods to access subfields in registers.
 */
static __inline void
urtwn_setbits_1(struct urtwn_softc *sc, uint16_t addr, uint8_t clr,
    uint8_t set)
{
	return (urtwn_write_1(sc, addr,
	    (urtwn_read_1(sc, addr) & ~clr) | set));
}

static __inline void
urtwn_setbits_1_shift(struct urtwn_softc *sc, uint16_t addr, uint32_t clr,
    uint32_t set, int shift)
{
	return (urtwn_setbits_1(sc, addr + shift, clr >> shift * NBBY,
	    set >> shift * NBBY));
}

static int
urtwn_read_region_1(struct urtwn_softc *sc, uint16_t addr, uint8_t *buf,
    int len)
{
	usb_device_request_t req;
	usbd_status error;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();
	usbwifi_isowned_ic(&sc->sc_uw);

	req.bmRequestType = UT_READ_VENDOR_DEVICE;
	req.bRequest = R92C_REQ_REGS;
	USETW(req.wValue, addr);
	USETW(req.wIndex, 0);
	USETW(req.wLength, len);
	error = usbd_do_request(sc->sc_uw.uw_udev, &req, buf);
	if (error != USBD_NORMAL_COMPLETION) {
		printf("error=%u: addr=%#jx, len=%d",
		     error, (intptr_t)addr, len);
	}
	return error;
}

static uint8_t
urtwn_read_1(struct urtwn_softc *sc, uint16_t addr)
{
	uint8_t val;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	if (urtwn_read_region_1(sc, addr, &val, 1) != USBD_NORMAL_COMPLETION)
		return 0xff;

	DPRINTFN(DBG_REG, "addr=%#jx, val=%#jx",
	    (intptr_t)addr, val, 0, 0);
	return val;
}

static uint16_t
urtwn_read_2(struct urtwn_softc *sc, uint16_t addr)
{
	uint8_t buf[2];
	uint16_t val;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	if (urtwn_read_region_1(sc, addr, buf, 2) != USBD_NORMAL_COMPLETION)
		return 0xffff;

	val = LE_READ_2(&buf[0]);
	DPRINTFN(DBG_REG, "addr=%#jx, val=%#jx", (intptr_t)addr, val, 0, 0);
	return val;
}

static uint32_t
urtwn_read_4(struct urtwn_softc *sc, uint16_t addr)
{
	uint8_t buf[4];
	uint32_t val;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	if (urtwn_read_region_1(sc, addr, buf, 4) != USBD_NORMAL_COMPLETION)
		return 0xffffffff;

	val = LE_READ_4(&buf[0]);
	DPRINTFN(DBG_REG, "addr=%#jx, val=%#jx", (intptr_t)addr, val, 0, 0);
	return val;
}

static void
urtwn_set_basicrates(struct urtwn_softc *sc, uint32_t rates)
{

	URTWNHIST_FUNC(); URTWNHIST_CALLED();
	DPRINTFN(DBG_REG, "%s: rates 0x%08X\n", __func__, rates, 0, 0);

	urtwn_setbits_4(sc, R92C_RRSR, R92C_RRSR_RATE_BITMAP_M, rates);
}

static int
urtwn_fw_cmd(struct urtwn_softc *sc, uint8_t id, const void *buf, int len)
{
	struct r92c_fw_cmd cmd;
	uint8_t *cp;
	int fwcur;
	int ntries;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();
	DPRINTFN(DBG_REG, "id=%jd, buf=%#jx, len=%jd", id, (intptr_t)buf,
	    len, 0);

	usbwifi_isowned_ic(&sc->sc_uw);
	if (!ISSET(sc->sc_uw.uw_flags, URTWN_FLAG_FWREADY)) {
		DPRINTFN(DBG_INIT, "fw not running, uw_flags=%jx",
		    sc->sc_uw.uw_flags, 0, 0, 0);
		return EAGAIN;
	}

	fwcur = sc->fwcur;
	sc->fwcur = (sc->fwcur + 1) % R92C_H2C_NBOX;

	/* Wait for current FW box to be empty. */
	for (ntries = 0; ntries < 100; ntries++) {
		if (!(urtwn_read_1(sc, R92C_HMETFR) & (1 << fwcur)))
			break;
		urtwn_delay_ms(sc, 2);
	}
	if (ntries == 100) {
		aprint_error_dev(sc->sc_uw.uw_dev,
		    "could not send firmware command %d\n", id);
		return ETIMEDOUT;
	}

	memset(&cmd, 0, sizeof(cmd));
	KASSERT(len <= sizeof(cmd.msg));
	memcpy(cmd.msg, buf, len);

	/* Write the first word last since that will trigger the FW. */
	cp = (uint8_t *)&cmd;
	cmd.id = id;
	if (len >= 4) {
		if (!ISSET(sc->chip, URTWN_CHIP_92EU)) {
			cmd.id |= R92C_CMD_FLAG_EXT;
			urtwn_write_region(sc, R92C_HMEBOX_EXT(fwcur),
			    &cp[1], 2);
			urtwn_write_4(sc, R92C_HMEBOX(fwcur),
			    cp[0] + (cp[3] << 8) + (cp[4] << 16) +
			    ((uint32_t)cp[5] << 24));
		} else {
			urtwn_write_region(sc, R92E_HMEBOX_EXT(fwcur),
			    &cp[4], 2);
			urtwn_write_4(sc, R92C_HMEBOX(fwcur),
			    cp[0] + (cp[1] << 8) + (cp[2] << 16) +
			    ((uint32_t)cp[3] << 24));
		}
	} else {
		urtwn_write_region(sc, R92C_HMEBOX(fwcur), cp, len);
	}

	return 0;
}

static int
urtwn_r88e_fw_cmd(struct urtwn_softc *sc, uint8_t id, const void *buf, int len)
{
	struct r88e_fw_cmd cmd;
	int ntries;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();
	DPRINTFN(DBG_REG, "id=%jd, buf=%#jx, len=%jd", id, (intptr_t)buf,
	    len, 0);

	usbwifi_isowned_ic(&sc->sc_uw);
	if (!ISSET(sc->sc_uw.uw_flags, URTWN_FLAG_FWREADY)) {
		DPRINTFN(DBG_INIT, "fw not running, uw_flags=%jx",
		    sc->sc_uw.uw_flags, 0, 0, 0);
		return EAGAIN;
	}

	/* Wait for current FW box to be empty. */
	for (ntries = 0; ntries < 100; ntries++) {
		if (!(urtwn_read_1(sc, R92C_HMETFR) & (1 << sc->fwcur)))
			break;
		urtwn_delay_ms(sc, 2);
	}
	if (ntries == 100) {
		aprint_error_dev(sc->sc_uw.uw_dev,
		    "could not send firmware command\n");
		return (ETIMEDOUT);
	}
	memset(&cmd, 0, sizeof(cmd));
	cmd.id = id;
	KASSERT(len <= sizeof(cmd.msg));
	memcpy(cmd.msg, buf, len);

	/* Write the first word last since that will trigger the FW. */
	if (len > 3) {
		urtwn_write_4(sc, R88E_HMEBOX_EXT(sc->fwcur),
		    *(uint32_t *)((uint8_t *)&cmd + 4));
	}
	urtwn_write_4(sc, R92C_HMEBOX(sc->fwcur), *(uint32_t *)&cmd);

	sc->fwcur = (sc->fwcur + 1) % R92C_H2C_NBOX;

	return (0);
}

static __inline void
urtwn_rf_write(struct urtwn_softc *sc, int chain, uint8_t addr, uint32_t val)
{

	sc->sc_rf_write(sc, chain, addr, val);
}

static void
urtwn_r92c_rf_write(struct urtwn_softc *sc, int chain, uint8_t addr,
    uint32_t val)
{

	urtwn_bb_write(sc, R92C_LSSI_PARAM(chain),
	    SM(R92C_LSSI_PARAM_ADDR, addr) | SM(R92C_LSSI_PARAM_DATA, val));
}

static void
urtwn_r88e_rf_write(struct urtwn_softc *sc, int chain, uint8_t addr,
    uint32_t val)
{

	urtwn_bb_write(sc, R92C_LSSI_PARAM(chain),
	    SM(R88E_LSSI_PARAM_ADDR, addr) | SM(R92C_LSSI_PARAM_DATA, val));
}

static void
urtwn_r92e_rf_write(struct urtwn_softc *sc, int chain, uint8_t addr,
    uint32_t val)
{

	urtwn_bb_write(sc, R92C_LSSI_PARAM(chain),
	    SM(R88E_LSSI_PARAM_ADDR, addr) | SM(R92C_LSSI_PARAM_DATA, val));
}

static void
urtwn_r21a_rf_write(struct urtwn_softc *sc, int chain, uint8_t addr,
    uint32_t val)
{
	urtwn_bb_write(sc, R21A_LSSI_PARAM(chain),
	    SM(R88E_LSSI_PARAM_ADDR, addr) |
	    SM(R92C_LSSI_PARAM_DATA, val));
}

static __inline void
urtwn_rf_setbits(struct urtwn_softc *sc, int chain, uint8_t addr,
    uint32_t clr, uint32_t set)
{
	urtwn_rf_write(sc, chain, addr,
	    (urtwn_rf_read(sc, chain, addr) & ~clr) | set);
}

static uint32_t
urtwn_rf_read(struct urtwn_softc *sc, int chain, uint8_t addr)
{
	return sc->sc_rf_read(sc, chain, addr);
}

static uint32_t
urtwn_r92e_rf_read(struct urtwn_softc *sc, int chain, uint8_t addr)
{
	uint32_t reg[R92C_MAX_CHAINS], val;

	reg[0] = urtwn_bb_read(sc, R92C_HSSI_PARAM2(0));
	if (chain != 0) {
		reg[chain] = urtwn_bb_read(sc, R92C_HSSI_PARAM2(chain));
	}

	urtwn_bb_write(sc, R92C_HSSI_PARAM2(0),
	    reg[0] & ~R92C_HSSI_PARAM2_READ_EDGE);
	urtwn_delay_ms(sc, 1);

	urtwn_bb_write(sc, R92C_HSSI_PARAM2(chain),
	    RW(reg[chain], R92C_HSSI_PARAM2_READ_ADDR, addr) |
	    R92C_HSSI_PARAM2_READ_EDGE);
	urtwn_delay_ms(sc, 1);

	urtwn_bb_write(sc, R92C_HSSI_PARAM2(0),
	    reg[0] | R92C_HSSI_PARAM2_READ_EDGE);
	urtwn_delay_ms(sc, 1);

	if (urtwn_bb_read(sc, R92C_HSSI_PARAM1(chain)) & R92C_HSSI_PARAM1_PI) {
		val = urtwn_bb_read(sc, R92C_HSPI_READBACK(chain));
	} else {
		val = urtwn_bb_read(sc, R92C_LSSI_READBACK(chain));
	}
	return MS(val, R92C_LSSI_READBACK_DATA);
}

static uint32_t
urtwn_r21a_rf_read(struct urtwn_softc *sc, int chain, uint8_t addr)
{
	uint32_t pi_mode, val;

	val = urtwn_bb_read(sc, R21A_HSSI_PARAM1(chain));
	pi_mode = (val & R21A_HSSI_PARAM1_PI) ? 1 : 0;

	urtwn_bb_setbits(sc, R21A_HSSI_PARAM2,
	    R21A_HSSI_PARAM2_READ_ADDR_MASK, addr);
	DELAY(20);

	val = urtwn_bb_read(sc, pi_mode ? R21A_HSPI_READBACK(chain) :
	    R21A_LSSI_READBACK(chain));

	return (MS(val, R92C_LSSI_READBACK_DATA));
}

static int
urtwn_llt_write(struct urtwn_softc *sc, uint32_t addr, uint32_t data)
{
	int ntries;

	usbwifi_isowned_ic(&sc->sc_uw);

	urtwn_write_4(sc, R92C_LLT_INIT,
	    SM(R92C_LLT_INIT_OP, R92C_LLT_INIT_OP_WRITE) |
	    SM(R92C_LLT_INIT_ADDR, addr) |
	    SM(R92C_LLT_INIT_DATA, data));
	/* Wait for write operation to complete. */
	for (ntries = 0; ntries < 20; ntries++) {
		if (MS(urtwn_read_4(sc, R92C_LLT_INIT), R92C_LLT_INIT_OP) ==
		    R92C_LLT_INIT_OP_NO_ACTIVE) {
			/* Done */
			return 0;
		}
		DELAY(5);
	}
	return ETIMEDOUT;
}

static uint8_t
urtwn_efuse_read_1(struct urtwn_softc *sc, uint16_t addr)
{
	uint32_t reg;
	int ntries;

	usbwifi_isowned_ic(&sc->sc_uw);

	reg = urtwn_read_4(sc, R92C_EFUSE_CTRL);
	reg = RW(reg, R92C_EFUSE_CTRL_ADDR, addr);
	reg &= ~R92C_EFUSE_CTRL_VALID;
	urtwn_write_4(sc, R92C_EFUSE_CTRL, reg);

	/* Wait for read operation to complete. */
	for (ntries = 0; ntries < 100; ntries++) {
		reg = urtwn_read_4(sc, R92C_EFUSE_CTRL);
		if (reg & R92C_EFUSE_CTRL_VALID) {
			/* Done */
			return MS(reg, R92C_EFUSE_CTRL_DATA);
		}
		DELAY(5);
	}
	aprint_error_dev(sc->sc_uw.uw_dev,
	    "could not read efuse byte at address 0x%04x\n", addr);
	return 0xff;
}

static void
urtwn_efuse_read(struct urtwn_softc *sc)
{
	uint8_t *rom = (uint8_t *)&sc->rom;
	uint32_t reg;
	uint16_t addr = 0;
	uint8_t off, msk;
	size_t i;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	usbwifi_isowned_ic(&sc->sc_uw);

	urtwn_efuse_switch_power(sc);

	memset(&sc->rom, 0xff, sizeof(sc->rom));
	while (addr < 512) {
		reg = urtwn_efuse_read_1(sc, addr);
		if (reg == 0xff)
			break;
		addr++;
		off = reg >> 4;
		msk = reg & 0xf;
		for (i = 0; i < 4; i++) {
			if (msk & (1U << i))
				continue;

			rom[off * 8 + i * 2 + 0] = urtwn_efuse_read_1(sc, addr);
			addr++;
			rom[off * 8 + i * 2 + 1] = urtwn_efuse_read_1(sc, addr);
			addr++;
		}
	}
#ifdef URTWN_DEBUG
	if (urtwn_debug & DBG_INIT) {
		/* Dump ROM content. */
		printf("%s: %s", device_xname(sc->sc_uw.uw_dev), __func__);
		for (i = 0; i < (int)sizeof(sc->rom); i++)
			printf(":%02x", rom[i]);
		printf("\n");
	}
#endif
}

static void
urtwn_efuse_switch_power(struct urtwn_softc *sc)
{
	uint32_t reg;

	if (ISSET(sc->chip, URTWN_CHIP_21A)){
		urtwn_write_1(sc, R92C_EFUSE_ACCESS, R92C_EFUSE_ACCESS_ON);
	}
	else{
		reg = urtwn_read_2(sc, R92C_SYS_ISO_CTRL);
		if (!(reg & R92C_SYS_ISO_CTRL_PWC_EV12V)) {
			urtwn_write_2(sc, R92C_SYS_ISO_CTRL,
			    reg | R92C_SYS_ISO_CTRL_PWC_EV12V);
		}
	}

	reg = urtwn_read_2(sc, R92C_SYS_FUNC_EN);
	if (!(reg & R92C_SYS_FUNC_EN_ELDR)) {
		urtwn_write_2(sc, R92C_SYS_FUNC_EN,
		    reg | R92C_SYS_FUNC_EN_ELDR);
	}
	reg = urtwn_read_2(sc, R92C_SYS_CLKR);
	if ((reg & (R92C_SYS_CLKR_LOADER_EN | R92C_SYS_CLKR_ANA8M)) !=
	    (R92C_SYS_CLKR_LOADER_EN | R92C_SYS_CLKR_ANA8M)) {
		urtwn_write_2(sc, R92C_SYS_CLKR,
		    reg | R92C_SYS_CLKR_LOADER_EN | R92C_SYS_CLKR_ANA8M);
	}
}

static int
urtwn_read_chipid(struct urtwn_softc *sc)
{
	uint32_t reg;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	if (ISSET(sc->chip, URTWN_CHIP_88E) ||
	    ISSET(sc->chip, URTWN_CHIP_92EU)||
	    ISSET(sc->chip, URTWN_CHIP_21A))
		return 0;

	reg = urtwn_read_4(sc, R92C_SYS_CFG);
	if (reg & R92C_SYS_CFG_TRP_VAUX_EN) {
		/* test chip, not supported */
		return EIO;
	}
	if (reg & R92C_SYS_CFG_TYPE_92C) {
		sc->chip |= URTWN_CHIP_92C;
		/* Check if it is a castrated 8192C. */
		if (MS(urtwn_read_4(sc, R92C_HPON_FSM),
		    R92C_HPON_FSM_CHIP_BONDING_ID) ==
		    R92C_HPON_FSM_CHIP_BONDING_ID_92C_1T2R) {
			sc->chip |= URTWN_CHIP_92C_1T2R;
		}
	}
	if (reg & R92C_SYS_CFG_VENDOR_UMC) {
		sc->chip |= URTWN_CHIP_UMC;
		if (MS(reg, R92C_SYS_CFG_CHIP_VER_RTL) == 0) {
			sc->chip |= URTWN_CHIP_UMC_A_CUT;
		}
	}
	return 0;
}

#ifdef URTWN_DEBUG
static void
urtwn_dump_rom(struct urtwn_softc *sc, struct r92c_rom *rp)
{

	aprint_normal_dev(sc->sc_uw.uw_dev,
	    "id 0x%04x, dbg_sel %#x, vid %#x, pid %#x\n",
	    rp->id, rp->dbg_sel, rp->vid, rp->pid);

	aprint_normal_dev(sc->sc_uw.uw_dev,
	    "usb_opt %#x, ep_setting %#x, usb_phy %#x\n",
	    rp->usb_opt, rp->ep_setting, rp->usb_phy);

	aprint_normal_dev(sc->sc_uw.uw_dev,
	    "macaddr %02x:%02x:%02x:%02x:%02x:%02x\n",
	    rp->macaddr[0], rp->macaddr[1],
	    rp->macaddr[2], rp->macaddr[3],
	    rp->macaddr[4], rp->macaddr[5]);

	aprint_normal_dev(sc->sc_uw.uw_dev,
	    "string %s, subcustomer_id %#x\n",
	    rp->string, rp->subcustomer_id);

	aprint_normal_dev(sc->sc_uw.uw_dev,
	    "cck_tx_pwr c0: %d %d %d, c1: %d %d %d\n",
	    rp->cck_tx_pwr[0][0], rp->cck_tx_pwr[0][1], rp->cck_tx_pwr[0][2],
	    rp->cck_tx_pwr[1][0], rp->cck_tx_pwr[1][1], rp->cck_tx_pwr[1][2]);

	aprint_normal_dev(sc->sc_uw.uw_dev,
	    "ht40_1s_tx_pwr c0 %d %d %d, c1 %d %d %d\n",
	    rp->ht40_1s_tx_pwr[0][0], rp->ht40_1s_tx_pwr[0][1],
	    rp->ht40_1s_tx_pwr[0][2],
	    rp->ht40_1s_tx_pwr[1][0], rp->ht40_1s_tx_pwr[1][1],
	    rp->ht40_1s_tx_pwr[1][2]);

	aprint_normal_dev(sc->sc_uw.uw_dev,
	    "ht40_2s_tx_pwr_diff c0: %d %d %d, c1: %d %d %d\n",
	    rp->ht40_2s_tx_pwr_diff[0] & 0xf, rp->ht40_2s_tx_pwr_diff[1] & 0xf,
	    rp->ht40_2s_tx_pwr_diff[2] & 0xf,
	    rp->ht40_2s_tx_pwr_diff[0] >> 4, rp->ht40_2s_tx_pwr_diff[1] & 0xf,
	    rp->ht40_2s_tx_pwr_diff[2] >> 4);

	aprint_normal_dev(sc->sc_uw.uw_dev,
	    "ht20_tx_pwr_diff c0: %d %d %d, c1: %d %d %d\n",
	    rp->ht20_tx_pwr_diff[0] & 0xf, rp->ht20_tx_pwr_diff[1] & 0xf,
	    rp->ht20_tx_pwr_diff[2] & 0xf,
	    rp->ht20_tx_pwr_diff[0] >> 4, rp->ht20_tx_pwr_diff[1] >> 4,
	    rp->ht20_tx_pwr_diff[2] >> 4);

	aprint_normal_dev(sc->sc_uw.uw_dev,
	    "ofdm_tx_pwr_diff c0: %d %d %d, c1: %d %d %d\n",
	    rp->ofdm_tx_pwr_diff[0] & 0xf, rp->ofdm_tx_pwr_diff[1] & 0xf,
	    rp->ofdm_tx_pwr_diff[2] & 0xf,
	    rp->ofdm_tx_pwr_diff[0] >> 4, rp->ofdm_tx_pwr_diff[1] >> 4,
	    rp->ofdm_tx_pwr_diff[2] >> 4);

	aprint_normal_dev(sc->sc_uw.uw_dev,
	    "ht40_max_pwr_offset c0: %d %d %d, c1: %d %d %d\n",
	    rp->ht40_max_pwr[0] & 0xf, rp->ht40_max_pwr[1] & 0xf,
	    rp->ht40_max_pwr[2] & 0xf,
	    rp->ht40_max_pwr[0] >> 4, rp->ht40_max_pwr[1] >> 4,
	    rp->ht40_max_pwr[2] >> 4);

	aprint_normal_dev(sc->sc_uw.uw_dev,
	    "ht20_max_pwr_offset c0: %d %d %d, c1: %d %d %d\n",
	    rp->ht20_max_pwr[0] & 0xf, rp->ht20_max_pwr[1] & 0xf,
	    rp->ht20_max_pwr[2] & 0xf,
	    rp->ht20_max_pwr[0] >> 4, rp->ht20_max_pwr[1] >> 4,
	    rp->ht20_max_pwr[2] >> 4);

	aprint_normal_dev(sc->sc_uw.uw_dev,
	    "xtal_calib %d, tssi %d %d, thermal %d\n",
	    rp->xtal_calib, rp->tssi[0], rp->tssi[1], rp->thermal_meter);

	aprint_normal_dev(sc->sc_uw.uw_dev,
	    "rf_opt1 %#x, rf_opt2 %#x, rf_opt3 %#x, rf_opt4 %#x\n",
	    rp->rf_opt1, rp->rf_opt2, rp->rf_opt3, rp->rf_opt4);

	aprint_normal_dev(sc->sc_uw.uw_dev,
	    "channnel_plan %d, version %d customer_id %#x\n",
	    rp->channel_plan, rp->version, rp->curstomer_id);
}
#endif

static void
urtwn_read_rom(struct urtwn_softc *sc)
{
	struct ieee80211com *ic = usbwifi_ic(&sc->sc_uw);
	struct r92c_rom *rom = &sc->rom;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	usbwifi_isowned_ic(&sc->sc_uw);

	/* Read full ROM image. */
	urtwn_efuse_read(sc);
#ifdef URTWN_DEBUG
	if (urtwn_debug & DBG_REG)
		urtwn_dump_rom(sc, rom);
#endif

	/* XXX Weird but this is what the vendor driver does. */
	sc->pa_setting = urtwn_efuse_read_1(sc, 0x1fa);
	sc->board_type = MS(rom->rf_opt1, R92C_ROM_RF1_BOARD_TYPE);
	sc->regulatory = MS(rom->rf_opt1, R92C_ROM_RF1_REGULATORY);

	DPRINTFN(DBG_INIT,
	    "PA setting=%#jx, board=%#jx, regulatory=%jd",
	    sc->pa_setting, sc->board_type, sc->regulatory, 0);

	IEEE80211_ADDR_COPY(ic->ic_macaddr, rom->macaddr);
	sc->sc_rf_write = urtwn_r92c_rf_write;
	sc->sc_power_on = urtwn_r92c_power_on;
	sc->sc_dma_init = urtwn_r92c_dma_init;
	sc->sc_rf_read = urtwn_r92e_rf_read;
	sc->sc_postattach = urtwn_nop_softc;
	sc->sc_check_condition = urtwn_r92c_check_condition;
	sc->sc_vap_preattach = urtwn_nop_vappreattach;
	sc->sc_set_media_status = urtwn_nop_softc_int;
	sc->sc_tx_prepare 		= urtwn_r88e_tx_prepare;
}

static void
urtwn_r88e_read_rom(struct urtwn_softc *sc)
{
	struct ieee80211com *ic = usbwifi_ic(&sc->sc_uw);
	uint8_t *rom = sc->r88e_rom;
	uint32_t reg;
	uint16_t addr = 0;
	uint8_t off, msk, tmp;
	int i;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	usbwifi_isowned_ic(&sc->sc_uw);

	off = 0;
	urtwn_efuse_switch_power(sc);

	/* Read full ROM image. */
	memset(&sc->r88e_rom, 0xff, sizeof(sc->r88e_rom));
	while (addr < 4096) {
		reg = urtwn_efuse_read_1(sc, addr);
		if (reg == 0xff)
			break;
		addr++;
		if ((reg & 0x1f) == 0x0f) {
			tmp = (reg & 0xe0) >> 5;
			reg = urtwn_efuse_read_1(sc, addr);
			if ((reg & 0x0f) != 0x0f)
				off = ((reg & 0xf0) >> 1) | tmp;
			addr++;
		} else
			off = reg >> 4;
		msk = reg & 0xf;
		for (i = 0; i < 4; i++) {
			if (msk & (1 << i))
				continue;
			rom[off * 8 + i * 2 + 0] = urtwn_efuse_read_1(sc, addr);
			addr++;
			rom[off * 8 + i * 2 + 1] = urtwn_efuse_read_1(sc, addr);
			addr++;
		}
	}

	addr = 0x10;
	for (i = 0; i < 6; i++)
		sc->cck_tx_pwr[i] = sc->r88e_rom[addr++];
	for (i = 0; i < 5; i++)
		sc->ht40_tx_pwr[i] = sc->r88e_rom[addr++];
	sc->bw20_tx_pwr_diff = (sc->r88e_rom[addr] & 0xf0) >> 4;
	if (sc->bw20_tx_pwr_diff & 0x08)
		sc->bw20_tx_pwr_diff |= 0xf0;
	sc->ofdm_tx_pwr_diff = (sc->r88e_rom[addr] & 0xf);
	if (sc->ofdm_tx_pwr_diff & 0x08)
		sc->ofdm_tx_pwr_diff |= 0xf0;
	sc->regulatory = MS(sc->r88e_rom[0xc1], R92C_ROM_RF1_REGULATORY);

	IEEE80211_ADDR_COPY(ic->ic_macaddr, &sc->r88e_rom[0xd7]);

	if (ISSET(sc->chip, URTWN_CHIP_92EU)) {
		sc->sc_power_on = urtwn_r92e_power_on;
		sc->sc_rf_write = urtwn_r92e_rf_write;
	} else {
		sc->sc_power_on = urtwn_r88e_power_on;
		sc->sc_rf_write = urtwn_r88e_rf_write;
	}
	sc->sc_dma_init = urtwn_r88e_dma_init;
	sc->sc_rf_read = urtwn_r92e_rf_read;
	sc->sc_postattach = urtwn_nop_softc;
	sc->sc_vap_preattach = urtwn_nop_vappreattach;
	sc->sc_check_condition = urtwn_r92c_check_condition;
	sc->sc_set_media_status = urtwn_nop_softc_int;
	sc->sc_tx_prepare 		= urtwn_r88e_tx_prepare;
}

static void
urtwn_r21a_read_rom(struct urtwn_softc *sc){
	struct ieee80211com *ic = usbwifi_ic(&sc->sc_uw);;
	struct urtwn_r21a_data *r21a_data = sc->sc_chip_priv.data;
	uint8_t *buf = (uint8_t *)&r21a_data->r21a_rom;
	struct r21a_rom *rom = &r21a_data->r21a_rom;
	uint32_t reg;
	uint16_t addr = 0;
	uint8_t off, msk, tmp;
	int i, j, k;
	uint8_t pa_type, lna_type_2g, lna_type_5g;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	off = 0;
	urtwn_efuse_switch_power(sc);

	/* Read full ROM image. */
	memset(buf, 0xff, sizeof(r21a_data->r21a_rom));
	while (addr < 512) {
		reg = urtwn_efuse_read_1(sc, addr);
		if (reg == 0xff)
			break;
		addr++;
		if ((reg & 0x1f) == 0x0f) {
			tmp = (reg & 0xe0) >> 5;
			reg = urtwn_efuse_read_1(sc, addr);
			addr++;
			if ((reg & 0x0f) != 0x0f)
				off = ((reg & 0xf0) >> 1) | tmp;
			else
				continue;
		} else
			off = reg >> 4;
		msk = reg & 0xf;
		for (i = 0; i < 4; i++) {
			if (msk & (1 << i))
				continue;
			buf[off * 8 + i * 2 + 0] = urtwn_efuse_read_1(sc, addr);
			addr++;
			buf[off * 8 + i * 2 + 1] = urtwn_efuse_read_1(sc, addr);
			addr++;
		}
	}
#ifdef URTWN_DEBUG
	/* Dump ROM contents. */
	device_printf(sc->sc_uw.uw_dev, "%s:", __func__);
	for (i = 0; i < 512; i++) {
		if (i % 32 == 0)
			printf("\n%03X: ", i);
		else if (i % 4 == 0)
			printf(" ");

		printf("%02X", buf[i]);
	}
	printf("\n");
#endif
	/* Read PA/LNA types. */
	pa_type = URTWN_GET_ROM_VAR(rom->pa_type, 0);
	lna_type_2g = URTWN_GET_ROM_VAR(rom->lna_type_2g, 0);
	lna_type_5g = URTWN_GET_ROM_VAR(rom->lna_type_5g, 0);

	r21a_data->ext_pa_2g = R21A_ROM_IS_PA_EXT_2GHZ(pa_type);
	r21a_data->ext_pa_5g = R21A_ROM_IS_PA_EXT_5GHZ(pa_type);
	r21a_data->ext_lna_2g = R21A_ROM_IS_LNA_EXT(lna_type_2g);
	r21a_data->ext_lna_5g = R21A_ROM_IS_LNA_EXT(lna_type_5g);
	r21a_data->bt_coex = (MS(rom->rf_board_opt, R92C_ROM_RF1_BOARD_TYPE) ==
	    R92C_BOARD_TYPE_HIGHPA);
	r21a_data->bt_ant_num = (rom->rf_bt_opt & R21A_RF_BT_OPT_ANT_NUM);

	if (r21a_data->ext_pa_2g) {
		r21a_data->type_pa_2g =
		    R21A_GET_ROM_PA_TYPE(lna_type_2g, 0) |
		    (R21A_GET_ROM_PA_TYPE(lna_type_2g, 1) << 2);
	}
	if (r21a_data->ext_pa_5g) {
		r21a_data->type_pa_5g =
		    R21A_GET_ROM_PA_TYPE(lna_type_5g, 0) |
		    (R21A_GET_ROM_PA_TYPE(lna_type_5g, 1) << 2);
	}
	if (r21a_data->ext_lna_2g) {
		r21a_data->type_lna_2g =
		    R21A_GET_ROM_LNA_TYPE(lna_type_2g, 0) |
		    (R21A_GET_ROM_LNA_TYPE(lna_type_2g, 1) << 2);
	}
	if (r21a_data->ext_lna_5g) {
		r21a_data->type_lna_5g =
		    R21A_GET_ROM_LNA_TYPE(lna_type_5g, 0) |
		    (R21A_GET_ROM_LNA_TYPE(lna_type_5g, 1) << 2);
	}
	IEEE80211_ADDR_COPY(ic->ic_macaddr, &rom->macaddr_21a);

	sc->sc_power_on = urtwn_r21a_power_on;
	sc->sc_rf_write = urtwn_r21a_rf_write;
	sc->sc_dma_init = urtwn_r21a_dma_init;
	sc->sc_vap_preattach = urtwn_r21au_vap_preattach;
	sc->sc_postattach = urtwn_r21a_postattach;
	sc->sc_check_condition = urtwn_r21a_check_condition;
	sc->sc_rf_read = urtwn_r21a_rf_read;
	sc->sc_set_media_status = urtwn_r21a_set_media_status;
	sc->chan_list_5ghz		= r21a_chan_5ghz;
	sc->chan_num_5ghz		= nitems(r21a_chan_5ghz);
	sc->sc_tx_prepare 		= urtwn_r21a_tx_prepare;
	for (i = 0; i < sc->ntxchains; i++) {
		struct r21a_tx_pwr_2g *pwr_2g = &rom->tx_pwr[i].pwr_2g;
		struct r21a_tx_pwr_5g *pwr_5g = &rom->tx_pwr[i].pwr_5g;
		struct r21a_tx_pwr_diff_2g *pwr_diff_2g =
		    &rom->tx_pwr[i].pwr_diff_2g;
		struct r21a_tx_pwr_diff_5g *pwr_diff_5g =
		    &rom->tx_pwr[i].pwr_diff_5g;

		for (j = 0; j < R21A_GROUP_2G - 1; j++) {
			r21a_data->cck_tx_pwr[i][j] =
			    URTWN_GET_ROM_VAR(pwr_2g->cck[j],
				R21A_DEF_TX_PWR_2G);
			r21a_data->ht40_tx_pwr_2g[i][j] =
			    URTWN_GET_ROM_VAR(pwr_2g->ht40[j],
				R21A_DEF_TX_PWR_2G);
		}
		r21a_data->cck_tx_pwr[i][j] = URTWN_GET_ROM_VAR(pwr_2g->cck[j],
		    R21A_DEF_TX_PWR_2G);

		r21a_data->cck_tx_pwr_diff_2g[i][0] = 0;
		r21a_data->ofdm_tx_pwr_diff_2g[i][0] = URTWN_SIGN4TO8(
		    MS(pwr_diff_2g->ht20_ofdm, LOW_PART));
		r21a_data->bw20_tx_pwr_diff_2g[i][0] = URTWN_SIGN4TO8(
		    MS(pwr_diff_2g->ht20_ofdm, HIGH_PART));
		r21a_data->bw40_tx_pwr_diff_2g[i][0] = 0;

		for (j = 1, k = 0; k < nitems(pwr_diff_2g->diff123); j++, k++) {
			r21a_data->cck_tx_pwr_diff_2g[i][j] = URTWN_SIGN4TO8(
			    MS(pwr_diff_2g->diff123[k].ofdm_cck, LOW_PART));
			r21a_data->ofdm_tx_pwr_diff_2g[i][j] = URTWN_SIGN4TO8(
			    MS(pwr_diff_2g->diff123[k].ofdm_cck, HIGH_PART));
			r21a_data->bw20_tx_pwr_diff_2g[i][j] = URTWN_SIGN4TO8(
			    MS(pwr_diff_2g->diff123[k].ht40_ht20, LOW_PART));
			r21a_data->bw40_tx_pwr_diff_2g[i][j] = URTWN_SIGN4TO8(
			    MS(pwr_diff_2g->diff123[k].ht40_ht20, HIGH_PART));
		}

		for (j = 0; j < R21A_GROUP_5G; j++) {
			r21a_data->ht40_tx_pwr_5g[i][j] =
			    URTWN_GET_ROM_VAR(pwr_5g->ht40[j],
				R21A_DEF_TX_PWR_5G);
		}

		r21a_data->ofdm_tx_pwr_diff_5g[i][0] = URTWN_SIGN4TO8(
		    MS(pwr_diff_5g->ht20_ofdm, LOW_PART));
		r21a_data->ofdm_tx_pwr_diff_5g[i][1] = URTWN_SIGN4TO8(
		    MS(pwr_diff_5g->ofdm_ofdm[0], HIGH_PART));
		r21a_data->ofdm_tx_pwr_diff_5g[i][2] = URTWN_SIGN4TO8(
		    MS(pwr_diff_5g->ofdm_ofdm[0], LOW_PART));
		r21a_data->ofdm_tx_pwr_diff_5g[i][3] = URTWN_SIGN4TO8(
		    MS(pwr_diff_5g->ofdm_ofdm[1], LOW_PART));

		r21a_data->bw20_tx_pwr_diff_5g[i][0] = URTWN_SIGN4TO8(
		    MS(pwr_diff_5g->ht20_ofdm, HIGH_PART));
		r21a_data->bw40_tx_pwr_diff_5g[i][0] = 0;
		for (j = 1, k = 0; k < nitems(pwr_diff_5g->ht40_ht20);
		    j++, k++) {
			r21a_data->bw20_tx_pwr_diff_5g[i][j] = URTWN_SIGN4TO8(
			    MS(pwr_diff_5g->ht40_ht20[k], LOW_PART));
			r21a_data->bw40_tx_pwr_diff_5g[i][j] = URTWN_SIGN4TO8(
			    MS(pwr_diff_5g->ht40_ht20[k], HIGH_PART));
		}

		for (j = 0; j < nitems(pwr_diff_5g->ht80_ht160); j++) {
			r21a_data->bw80_tx_pwr_diff_5g[i][j] = URTWN_SIGN4TO8(
			    MS(pwr_diff_5g->ht80_ht160[j], HIGH_PART));
			r21a_data->bw160_tx_pwr_diff_5g[i][j] = URTWN_SIGN4TO8(
			    MS(pwr_diff_5g->ht80_ht160[j], LOW_PART));
		}
	}
}

/*
 * Initialize rate adaptation in firmware.
 */
static int
urtwn_ra_init(struct ieee80211vap *vap)
{
	static const uint8_t map[] = {
		2, 4, 11, 22, 12, 18, 24, 36, 48, 72, 96, 108
	};
	struct ieee80211com *ic = vap->iv_ic;
	struct urtwn_softc *sc = ic->ic_softc;
	struct ieee80211_node *ni = vap->iv_bss;
	struct ieee80211_rateset *rs = &ni->ni_rates;

	struct r92c_fw_cmd_macid_cfg cmd;
	uint32_t rates, basicrates;
	uint32_t rrsr_mask, rrsr_rate;
	uint8_t mode;
	size_t maxrate, maxbasicrate, i, j;
	int error;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	usbwifi_isowned_ic(&sc->sc_uw);

	/* Get normal and basic rates mask. */
	rates = basicrates = 1;
	maxrate = maxbasicrate = 0;
	for (i = 0; i < rs->rs_nrates; i++) {
		/* Convert 802.11 rate to HW rate index. */
		for (j = 0; j < __arraycount(map); j++) {
			if ((rs->rs_rates[i] & IEEE80211_RATE_VAL) == map[j]) {
				break;
			}
		}
		if (j == __arraycount(map)) {
			/* Unknown rate, skip. */
			continue;
		}

		rates |= 1U << j;
		if (j > maxrate) {
			maxrate = j;
		}

		if (rs->rs_rates[i] & IEEE80211_RATE_BASIC) {
			basicrates |= 1U << j;
			if (j > maxbasicrate) {
				maxbasicrate = j;
			}
		}
	}
	if (ic->ic_curmode == IEEE80211_MODE_11B) {
		mode = R92C_RAID_11B;
	} else if (ic->ic_curmode == IEEE80211_MODE_11G) {
		mode = R92C_RAID_11BG;
	} else /* mode = IEEE80211_MODE_11NG */
		mode = R92C_RAID_11GN;
	DPRINTFN(DBG_INIT, "mode=%#jx", mode, 0, 0, 0);
	DPRINTFN(DBG_INIT, "rates=%#jx, basicrates=%#jx, "
	    "maxrate=%jx, maxbasicrate=%jx",
	    rates, basicrates, maxrate, maxbasicrate);

	if (ni->ni_capinfo & IEEE80211_CAPINFO_SHORT_PREAMBLE) {
		maxbasicrate |= R92C_RATE_SHORTGI;
		maxrate |= R92C_RATE_SHORTGI;
	}

	/* Set rates mask for group addressed frames. */
	cmd.macid = RTWN_MACID_BC | RTWN_MACID_VALID;
	if (ni->ni_capinfo & IEEE80211_CAPINFO_SHORT_PREAMBLE)
		cmd.macid |= RTWN_MACID_SHORTGI;
	cmd.mask = htole32((mode << 28) | basicrates);
	error = urtwn_fw_cmd(sc, R92C_CMD_MACID_CONFIG, &cmd, sizeof(cmd));
	if (error != 0) {
		aprint_error_dev(sc->sc_uw.uw_dev,
		    "could not add broadcast station\n");
		return error;
	}
	/* Set initial MRR rate. */
	DPRINTFN(DBG_INIT, "maxbasicrate=%jd", maxbasicrate, 0, 0, 0);
	urtwn_write_1(sc, R92C_INIDATA_RATE_SEL(RTWN_MACID_BC), maxbasicrate);

	/* Set rates mask for unicast frames. */
	cmd.macid = RTWN_MACID_BSS | RTWN_MACID_VALID;
	if (ni->ni_capinfo & IEEE80211_CAPINFO_SHORT_PREAMBLE)
		cmd.macid |= RTWN_MACID_SHORTGI;
	cmd.mask = htole32((mode << 28) | rates);
	error = urtwn_fw_cmd(sc, R92C_CMD_MACID_CONFIG, &cmd, sizeof(cmd));
	if (error != 0) {
		aprint_error_dev(sc->sc_uw.uw_dev, "could not add BSS station\n");
		return error;
	}
	/* Set initial MRR rate. */
	DPRINTFN(DBG_INIT, "maxrate=%jd", maxrate, 0, 0, 0);
	urtwn_write_1(sc, R92C_INIDATA_RATE_SEL(RTWN_MACID_BSS), maxrate);

#if notyet
	/* NNN appears to have no fixed rate anywhere. */
	rrsr_rate = ic->ic_fixed_rate;
	if (rrsr_rate == -1)
#endif
		rrsr_rate = 11;

	rrsr_mask = 0xffff >> (15 - rrsr_rate);
	urtwn_write_2(sc, R92C_RRSR, rrsr_mask);

#if notyet
	/* Indicate highest supported rate. */
	ni->ni_txrate = rs->rs_nrates - 1;
#endif
	return 0;
}

static int
urtwn_get_nettype(struct urtwn_softc *sc)
{
	struct ieee80211com *ic = usbwifi_ic(&sc->sc_uw);
	int type;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	switch (ic->ic_opmode) {
	case IEEE80211_M_STA:
		type = R92C_MSR_INFRA;
		break;

	case IEEE80211_M_IBSS:
		type = R92C_MSR_ADHOC;
		break;

	default:
		type = R92C_MSR_AP;
		break;
	}

	return type;
}

static void
urtwn_set_nettype0_msr(struct urtwn_softc *sc, uint8_t type)
{
	uint8_t reg;

	URTWNHIST_FUNC();
	URTWNHIST_CALLARGS("type=%jd", type, 0, 0, 0);

	usbwifi_isowned_ic(&sc->sc_uw);

	reg = urtwn_read_1(sc, R92C_CR + 2) & 0x0c;
	urtwn_write_1(sc, R92C_CR + 2, reg | type);
}

static void __attribute__((unused))
urtwn_set_mode(struct urtwn_softc *sc, uint8_t mode, int id)
{
	urtwn_setbits_1(sc, R92C_MSR, ~R92C_MSR_MASK << id * 2, mode << id * 2);
}

static void
urtwn_tsf_sync_enable(struct urtwn_softc *sc, struct ieee80211_node *ni)
{
	uint64_t tsf;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	usbwifi_isowned_ic(&sc->sc_uw);

	/* Enable TSF synchronization. */
	urtwn_write_1(sc, R92C_BCN_CTRL,
	    urtwn_read_1(sc, R92C_BCN_CTRL) & ~R92C_BCN_CTRL_DIS_TSF_UDT0);

	/* Correct TSF */
	urtwn_write_1(sc, R92C_BCN_CTRL,
	    urtwn_read_1(sc, R92C_BCN_CTRL) & ~R92C_BCN_CTRL_EN_BCN);

	/* Set initial TSF. */
	tsf = ni->ni_tstamp.tsf;
	tsf = le64toh(tsf);
	tsf = tsf - (tsf % (ni->ni_intval * IEEE80211_DUR_TU));
	tsf -= IEEE80211_DUR_TU;
	urtwn_write_4(sc, R92C_TSFTR + 0, (uint32_t)tsf);
	urtwn_write_4(sc, R92C_TSFTR + 4, (uint32_t)(tsf >> 32));

	urtwn_write_1(sc, R92C_BCN_CTRL,
	    urtwn_read_1(sc, R92C_BCN_CTRL) | R92C_BCN_CTRL_EN_BCN);
}

static void
urtwn_set_led(struct urtwn_softc *sc, int led, int on)
{
	uint8_t reg;

	URTWNHIST_FUNC();
	URTWNHIST_CALLARGS("led=%jd, on=%jd", led, on, 0, 0);

	usbwifi_isowned_ic(&sc->sc_uw);

	if (led == URTWN_LED_LINK) {
		if (ISSET(sc->chip, URTWN_CHIP_92EU)) {
			urtwn_write_1(sc, 0x64, urtwn_read_1(sc, 0x64) & 0xfe);
			reg = urtwn_read_1(sc, R92C_LEDCFG1) & R92E_LEDSON;
			urtwn_write_1(sc, R92C_LEDCFG1, reg |
			    (R92C_LEDCFG0_DIS << 1));
			if (on) {
				reg = urtwn_read_1(sc, R92C_LEDCFG1) &
				    R92E_LEDSON;
				urtwn_write_1(sc, R92C_LEDCFG1, reg);
			}
		} else if (ISSET(sc->chip, URTWN_CHIP_88E)) {
			reg = urtwn_read_1(sc, R92C_LEDCFG2) & 0xf0;
			urtwn_write_1(sc, R92C_LEDCFG2, reg | 0x60);
			if (!on) {
				reg = urtwn_read_1(sc, R92C_LEDCFG2) & 0x90;
				urtwn_write_1(sc, R92C_LEDCFG2,
				    reg | R92C_LEDCFG0_DIS);
				reg = urtwn_read_1(sc, R92C_MAC_PINMUX_CFG);
				urtwn_write_1(sc, R92C_MAC_PINMUX_CFG,
				    reg & 0xfe);
			}
		} else if (ISSET(sc->chip, URTWN_CHIP_21A)) {
			urtwn_write_1(sc, R92C_LEDCFG2,
			    R21A_LEDCFG2_ENA | (on ? 0 : R92C_LEDCFG0_DIS));
		} else {
			reg = urtwn_read_1(sc, R92C_LEDCFG0) & 0x70;
			if (!on) {
				reg |= R92C_LEDCFG0_DIS;
			}
			urtwn_write_1(sc, R92C_LEDCFG0, reg);
		}
		sc->ledlink = on;	/* Save LED state. */
	}
}

static void
urtwn_calib_to(void *arg)
{
	struct urtwn_softc *sc = arg;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	if (usbwifi_isdying(&sc->sc_uw))
		return;

	/* Do it in a process context. */
	urtwn_do_async(sc, urtwn_calib_to_cb, NULL, 0);
}

/* ARGSUSED */
static void
urtwn_calib_to_cb(struct urtwn_softc *sc, void *arg)
{
	struct r92c_fw_cmd_rssi cmd;
	struct r92e_fw_cmd_rssi cmde;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	usbwifi_lock_ic(&sc->sc_uw);
	if (!sc->sc_uw.uw_ic.ic_nrunning) {
		usbwifi_unlock_ic(&sc->sc_uw);
		return;
	}
	if (!ISSET(sc->sc_uw.uw_flags, URTWN_FLAG_FWREADY)) {
		usbwifi_unlock_ic(&sc->sc_uw);
		goto restart_timer;
	}
	if (sc->avg_pwdb != -1) {
		/* Indicate Rx signal strength to FW for rate adaptation. */
		memset(&cmd, 0, sizeof(cmd));
		memset(&cmde, 0, sizeof(cmde));
		cmd.macid = 0;	/* BSS. */
		cmde.macid = 0; /* BSS. */
		cmd.pwdb = sc->avg_pwdb;
		cmde.pwdb = sc->avg_pwdb;
		DPRINTFN(DBG_RF, "sending RSSI command avg=%jd",
		    sc->avg_pwdb, 0, 0, 0);
		if (!ISSET(sc->chip, URTWN_CHIP_92EU)) {
			urtwn_fw_cmd(sc, R92C_CMD_RSSI_SETTING, &cmd,
			    sizeof(cmd));
		} else {
			urtwn_fw_cmd(sc, R92E_CMD_RSSI_REPORT, &cmde,
			    sizeof(cmde));
		}
	}

	/* Do temperature compensation. */
	urtwn_temp_calib(sc);
	usbwifi_unlock_ic(&sc->sc_uw);

 restart_timer:
	if (!usbwifi_isdying(&sc->sc_uw)) {
		/* Restart calibration timer. */
		callout_schedule(&sc->sc_calib_to, hz);
	}
}

static void
power_control(struct urtwn_softc *sc, bool lowpower)
{

	if (!ISSET(sc->chip, URTWN_CHIP_92C)) {
		struct r92e_fw_cmd_setpwrmode cmd;
		memset(&cmd, 0, sizeof cmd);
		if (lowpower) {
			cmd.mode = FWMODE_LOW_POWER;
			cmd.smartps = SRTPS_LOW_POWER;
			cmd.pwr_state = PS_RFON;
		} else {
			cmd.mode = FWMODE_ACTIVE;
			cmd.smartps = SRTPS_LOW_POWER;
		}
		cmd.awake_int = 1;
		if (ISSET(sc->chip, URTWN_CHIP_21A))
			urtwn_r88e_fw_cmd(sc, R92E_CMD_SET_PWRMODE, &cmd, sizeof(cmd));
		else
			urtwn_fw_cmd(sc, R92E_CMD_SET_PWRMODE, &cmd, sizeof(cmd));
	} else {
		struct r92c_fw_cmd_setpwrmode cmd;
		memset(&cmd, 0, sizeof cmd);
		if (lowpower) {
			cmd.mode = FWMODE_LOW_POWER;
			cmd.smartps = SRTPS_LOW_POWER;
		} else {
			cmd.mode = FWMODE_ACTIVE;
			cmd.smartps = SRTPS_LOW_POWER;
		}
		cmd.bcn_time = 0;
		urtwn_fw_cmd(sc, R92C_CMD_SET_PWRMODE, &cmd, sizeof(cmd));
	}
	urtwn_delay_ms(sc, 200);
}

static void
urtwn_newassoc(struct ieee80211_node *ni, int isnew)
{
	struct ieee80211com *ic = ni->ni_ic;
	struct urtwn_softc *sc = ic->ic_softc;
	URTWNHIST_FUNC();
	URTWNHIST_CALLARGS("new node %06jx%06jx",
	    ni->ni_macaddr[0] << 2 |
	    ni->ni_macaddr[1] << 1 |
	    ni->ni_macaddr[2],
	    ni->ni_macaddr[3] << 2 |
	    ni->ni_macaddr[4] << 1 |
	    ni->ni_macaddr[5],
	    0, 0);

	/* start with lowest Tx rate */
	ni->ni_txrate = 0;
	urtwn_set_media_status(sc, 0);
}

/*
 * A VAP changes state.
 * This is called with thread context and the 'ic' lock held.
 */
static int
urtwn_newstate(struct ieee80211vap *vap, enum ieee80211_state nstate, int arg)
{
	struct urtwn_vap *uvap = (struct urtwn_vap*)vap;
	struct urtwn_softc *sc = vap->iv_ic->ic_softc;
	struct ieee80211com *ic = vap->iv_ic;
	struct ieee80211_node *ni;
	enum ieee80211_state ostate = vap->iv_state;
	uint32_t reg;
	uint8_t sifs_time, msr, early_newstate;

	URTWNHIST_FUNC();
	URTWNHIST_CALLARGS("nstate=%jd, arg=%jd, ostate=%jd",
	    nstate, arg, ostate, 0);

	callout_stop(&sc->sc_calib_to);
	if (sc->sc_uw.uw_pri != NULL)
		usb_rem_task_wait(sc->sc_uw.uw_udev, &sc->sc_task,
		    USB_TASKQ_DRIVER, usbwifi_mutex_ic(&sc->sc_uw));

	if (vap->iv_bss->ni_chan == IEEE80211_CHAN_ANYC &&
	    ostate == IEEE80211_S_INIT && nstate == IEEE80211_S_RUN) {
		/* need to call iv_newstate() firstly */
		int error = uvap->newstate(vap, nstate, arg);
		if (error != 0)
			return (error);

		early_newstate = 1;
	} else
		early_newstate = 0;


	switch (ostate) {
	case IEEE80211_S_INIT:
		break;

	case IEEE80211_S_SCAN:
		if (nstate != IEEE80211_S_SCAN) {
			/*
			 * End of scanning
			 */
			/* flush 4-AC Queue after site_survey */
			urtwn_write_1(sc, R92C_TXPAUSE, 0x0);

			/* Allow Rx from our BSSID only. */
			urtwn_write_4(sc, R92C_RCR,
			    urtwn_read_4(sc, R92C_RCR) |
			      R92C_RCR_CBSSID_DATA | R92C_RCR_CBSSID_BCN);
		}
		break;

	case IEEE80211_S_AUTH:
	case IEEE80211_S_ASSOC:
		break;

	case IEEE80211_S_RUN:
		if (nstate == IEEE80211_S_RUN || nstate == IEEE80211_S_SLEEP)
			break;
		/* Turn link LED off. */
		urtwn_set_led(sc, URTWN_LED_LINK, 0);

		/* Set media status to 'No Link'. */
		urtwn_set_nettype0_msr(sc, R92C_MSR_NOLINK);

		/* Stop Rx of data frames. */
		urtwn_write_2(sc, R92C_RXFLTMAP2, 0);

		/* Reset TSF. */
		urtwn_write_1(sc, R92C_DUAL_TSF_RST, 0x03);

		/* Disable TSF synchronization. */
		urtwn_write_1(sc, R92C_BCN_CTRL,
		    urtwn_read_1(sc, R92C_BCN_CTRL) |
		      R92C_BCN_CTRL_DIS_TSF_UDT0);

#if 0
		/* Back to 20MHz mode */
		urtwn_set_chan(sc, ic->ic_curchan,
		    IEEE80211_HTINFO_2NDCHAN_NONE);
#endif
		if (ic->ic_opmode == IEEE80211_M_IBSS ||
		    ic->ic_opmode == IEEE80211_M_HOSTAP) {
			/* Stop BCN */
			urtwn_write_1(sc, R92C_BCN_CTRL,
			    urtwn_read_1(sc, R92C_BCN_CTRL) &
			    ~(R92C_BCN_CTRL_EN_BCN | R92C_BCN_CTRL_TXBCN_RPT));
		}

		/* Reset EDCA parameters. */
		urtwn_write_4(sc, R92C_EDCA_VO_PARAM, 0x002f3217);
		urtwn_write_4(sc, R92C_EDCA_VI_PARAM, 0x005e4317);
		urtwn_write_4(sc, R92C_EDCA_BE_PARAM, 0x00105320);
		urtwn_write_4(sc, R92C_EDCA_BK_PARAM, 0x0000a444);

		/* flush all cam entries */
		urtwn_cam_init(sc);
		break;
	case IEEE80211_S_SLEEP:
		if (nstate == IEEE80211_S_SLEEP)
			break;
		power_control(sc, false);
		break;
	case IEEE80211_S_CAC:
	case IEEE80211_S_CSA:
		printf ("URTWN UNKNOWN oSTATE: %d\n", ostate);
		/* NNN what do we do in these states? XXX */
		break;
	}

	switch (nstate) {
	case IEEE80211_S_INIT:
		/* Turn link LED off. */
		urtwn_set_led(sc, URTWN_LED_LINK, 0);
		break;

	case IEEE80211_S_SCAN:
		if (ostate == IEEE80211_S_SCAN)
			break;
		if (ostate != IEEE80211_S_SCAN) {
			if (ISSET(sc->chip, URTWN_CHIP_21A)){
				urtwn_setbits_1(sc, R92C_TXPAUSE, 0, R92C_TX_QUEUE_AC);
				/* Make link LED blink during scan. */
				urtwn_set_led(sc, URTWN_LED_LINK, !sc->ledlink);
			} else {
				/*
				 * Begin of scanning
				 */

				/* Set gain for scanning. */
				reg = urtwn_bb_read(sc, R92C_OFDM0_AGCCORE1(0));
				reg = RW(reg, R92C_OFDM0_AGCCORE1_GAIN, 0x20);
				urtwn_bb_write(sc, R92C_OFDM0_AGCCORE1(0), reg);

				if (!ISSET(sc->chip, URTWN_CHIP_88E)) {
					reg = urtwn_bb_read(sc, R92C_OFDM0_AGCCORE1(1));
					reg = RW(reg, R92C_OFDM0_AGCCORE1_GAIN, 0x20);
					urtwn_bb_write(sc, R92C_OFDM0_AGCCORE1(1), reg);
				}

				/* Set media status to 'No Link'. */
				urtwn_set_nettype0_msr(sc, R92C_MSR_NOLINK);

				/* Allow Rx from any BSSID. */
				urtwn_write_4(sc, R92C_RCR,
				    urtwn_read_4(sc, R92C_RCR) &
				    ~(R92C_RCR_CBSSID_DATA | R92C_RCR_CBSSID_BCN));

				/* Stop Rx of data frames. */
				urtwn_write_2(sc, R92C_RXFLTMAP2, 0);

				/* Disable update TSF */
				urtwn_write_1(sc, R92C_BCN_CTRL,
				    urtwn_read_1(sc, R92C_BCN_CTRL) |
				      R92C_BCN_CTRL_DIS_TSF_UDT0);
					/* Make link LED blink during scan. */
				urtwn_set_led(sc, URTWN_LED_LINK, !sc->ledlink);

				/* Pause AC Tx queues. */
				urtwn_write_1(sc, R92C_TXPAUSE,
				    urtwn_read_1(sc, R92C_TXPAUSE) | 0x0f);
			}
		}

#if 0
		urtwn_set_chan(sc, ic->ic_curchan,
		    IEEE80211_HTINFO_2NDCHAN_NONE);
#endif
		break;

	case IEEE80211_S_AUTH:
		if (ostate == IEEE80211_S_AUTH)
			break;
		/* Set initial gain under link. */
		reg = urtwn_bb_read(sc, R92C_OFDM0_AGCCORE1(0));
		reg = RW(reg, R92C_OFDM0_AGCCORE1_GAIN, 0x32);
		urtwn_bb_write(sc, R92C_OFDM0_AGCCORE1(0), reg);

		if (!ISSET(sc->chip, URTWN_CHIP_88E) &&
			!ISSET(sc->chip, URTWN_CHIP_21A)) {
			reg = urtwn_bb_read(sc, R92C_OFDM0_AGCCORE1(1));
			reg = RW(reg, R92C_OFDM0_AGCCORE1_GAIN, 0x32);
			urtwn_bb_write(sc, R92C_OFDM0_AGCCORE1(1), reg);
		}

		/* Set media status to 'No Link'. */
		urtwn_set_nettype0_msr(sc, R92C_MSR_NOLINK);

		/* Allow Rx from any BSSID. */
		urtwn_write_4(sc, R92C_RCR,
		    urtwn_read_4(sc, R92C_RCR) &
		      ~(R92C_RCR_CBSSID_DATA | R92C_RCR_CBSSID_BCN));

#if 0
		urtwn_set_chan(sc, ic->ic_curchan,
		    IEEE80211_HTINFO_2NDCHAN_NONE);
#endif
		break;

	case IEEE80211_S_ASSOC:
		break;

	case IEEE80211_S_RUN:
		if (ostate == IEEE80211_S_RUN)
			break;
		if (ostate == IEEE80211_S_SLEEP)
			goto restart_calib;

		ni = vap->iv_bss;

#if 0
		/* XXX: Set 20MHz mode */
		urtwn_set_chan(sc, ic->ic_curchan,
		    IEEE80211_HTINFO_2NDCHAN_NONE);
#endif

		if (ic->ic_opmode == IEEE80211_M_MONITOR) {
#if 0
			/* Back to 20MHz mode */
			urtwn_set_chan(sc, ic->ic_curchan,
			    IEEE80211_HTINFO_2NDCHAN_NONE);
#endif

			/* Set media status to 'No Link'. */
			urtwn_set_nettype0_msr(sc, R92C_MSR_NOLINK);

			/* Enable Rx of data frames. */
			urtwn_write_2(sc, R92C_RXFLTMAP2, 0xffff);

			/* Allow Rx from any BSSID. */
			urtwn_write_4(sc, R92C_RCR,
			    urtwn_read_4(sc, R92C_RCR) &
			    ~(R92C_RCR_CBSSID_DATA | R92C_RCR_CBSSID_BCN));

			/* Accept Rx data/control/management frames */
			urtwn_write_4(sc, R92C_RCR,
			    urtwn_read_4(sc, R92C_RCR) |
			    R92C_RCR_ADF | R92C_RCR_ACF | R92C_RCR_AMF);

			/* Turn link LED on. */
			urtwn_set_led(sc, URTWN_LED_LINK, 1);
			break;
		}

		/* Set media status to 'Associated'. */
		urtwn_set_nettype0_msr(sc, urtwn_get_nettype(sc));

		/* Set BSSID. */
		urtwn_write_4(sc, R92C_BSSID + 0, LE_READ_4(&ni->ni_bssid[0]));
		urtwn_write_4(sc, R92C_BSSID + 4, LE_READ_2(&ni->ni_bssid[4]));

		if (ic->ic_curmode == IEEE80211_MODE_11B) {
			urtwn_write_1(sc, R92C_INIRTS_RATE_SEL, 0);
		} else if (ic->ic_curmode == IEEE80211_MODE_11G) {
			/* 802.11b/g */
			urtwn_write_1(sc, R92C_INIRTS_RATE_SEL, 3);
		} else /* IEEE_MODE_11NG */
			urtwn_write_1(sc, R92C_INIRTS_RATE_SEL, 12); /* MCS 0 */


		/* Enable Rx of data frames. */
		urtwn_write_2(sc, R92C_RXFLTMAP2, 0xffff);

		/* Set beacon interval. */
		urtwn_write_2(sc, R92C_BCN_INTERVAL, ni->ni_intval);

		msr = urtwn_read_1(sc, R92C_MSR);
		msr &= R92C_MSR_MASK;
		switch (ic->ic_opmode) {
		case IEEE80211_M_STA:
			/* Allow Rx from our BSSID only. */
			urtwn_write_4(sc, R92C_RCR,
			    urtwn_read_4(sc, R92C_RCR) |
			      R92C_RCR_CBSSID_DATA | R92C_RCR_CBSSID_BCN);

			/* Enable TSF synchronization. */
			urtwn_tsf_sync_enable(sc, ni);

			msr |= R92C_MSR_INFRA;
			break;
		case IEEE80211_M_HOSTAP:
			urtwn_write_2(sc, R92C_BCNTCFG, 0x000f);

			/* Allow Rx from any BSSID. */
			urtwn_write_4(sc, R92C_RCR,
			    urtwn_read_4(sc, R92C_RCR) &
			    ~(R92C_RCR_CBSSID_DATA | R92C_RCR_CBSSID_BCN));

			/* Reset TSF timer to zero. */
			reg = urtwn_read_4(sc, R92C_TCR);
			reg &= ~0x01;
			urtwn_write_4(sc, R92C_TCR, reg);
			reg |= 0x01;
			urtwn_write_4(sc, R92C_TCR, reg);

			msr |= R92C_MSR_AP;
			break;
		default:
			msr |= R92C_MSR_ADHOC;
			break;
		}
		urtwn_write_1(sc, R92C_MSR, msr);

		sifs_time = 10;
		urtwn_write_1(sc, R92C_SIFS_CCK + 1, sifs_time);
		urtwn_write_1(sc, R92C_SIFS_OFDM + 1, sifs_time);
		urtwn_write_1(sc, R92C_SPEC_SIFS + 1, sifs_time);
		urtwn_write_1(sc, R92C_MAC_SPEC_SIFS + 1, sifs_time);
		urtwn_write_1(sc, R92C_R2T_SIFS + 1, sifs_time);
		urtwn_write_1(sc, R92C_T2T_SIFS + 1, sifs_time);

		/* Initialize rate adaptation. */
		if (ISSET(sc->chip, URTWN_CHIP_88E) ||
		    ISSET(sc->chip, URTWN_CHIP_92EU)||
		    ISSET(sc->chip, URTWN_CHIP_21A))
			ni->ni_txrate = ni->ni_rates.rs_nrates - 1;
		else
			urtwn_ra_init(vap);

		/* Turn link LED on. */
		urtwn_set_led(sc, URTWN_LED_LINK, 1);

		/* Reset average RSSI. */
		sc->avg_pwdb = -1;

		/* Reset temperature calibration state machine. */
		sc->thcal_state = 0;
		sc->thcal_lctemp = 0;

restart_calib:
		/* Start periodic calibration. */
		if (!usbwifi_isdying(&sc->sc_uw))
			callout_schedule(&sc->sc_calib_to, hz);
		break;
	case IEEE80211_S_SLEEP:
		if (ostate != IEEE80211_S_SLEEP)
			power_control(sc, true);
		/* Start periodic calibration. */
		if (!usbwifi_isdying(&sc->sc_uw))
			callout_schedule(&sc->sc_calib_to, hz);
		break;
	case IEEE80211_S_CAC:
	case IEEE80211_S_CSA:
		/* NNN what do we do in these states? XXX */
		printf ("URTWN UNKNOWN nSTATE: %d\n", nstate);
		break;
	}

	return (early_newstate ? 0 : uvap->newstate(vap, nstate, arg));
}

static int
urtwn_r21au_newstate(struct ieee80211vap *vap, enum ieee80211_state nstate, int arg)
{
	struct urtwn_softc *sc = vap->iv_ic->ic_softc;
	struct ieee80211com *ic = vap->iv_ic;
	struct urtwn_r21a_data *r21a_data = sc->sc_chip_priv.data;
	int error;
	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	IEEE80211_UNLOCK(ic);
	usbwifi_lock_ic(&sc->sc_uw);

	error = 0;
	if (nstate == IEEE80211_S_CAC &&
	    !(r21a_data->flags & R21A_RADAR_ENABLED)) {
		error = urtwn_r21au_dfs_radar_enable(sc);
		if (error != 0) {
			device_printf(sc->sc_uw.uw_dev,
			    "%s: cannot enable radar detection\n", __func__);
			goto fail;
		}
		r21a_data->flags |= R21A_RADAR_ENABLED;

		DPRINTFN(DBG_FN,"%s: radar detection was enabled\n", __func__, 0, 0, 0);

		taskqueue_enqueue_timeout(sc->sc_tq,
		    &r21a_data->chan_check, URTWN_R21AU_RADAR_CHECK_PERIOD);
	}

	if ((nstate < IEEE80211_S_CAC || nstate == IEEE80211_S_CSA) &&
	    (r21a_data->flags & R21A_RADAR_ENABLED)) {
		taskqueue_cancel_timeout(sc->sc_tq, &r21a_data->chan_check,
		    NULL);

		r21a_data->flags &= ~R21A_RADAR_ENABLED;
		urtwn_r21au_dfs_radar_disable(sc);
		DPRINTFN(DBG_FN,"%s: radar detection was disabled\n", __func__, 0, 0, 0);
	}

fail:
	usbwifi_unlock_ic(&sc->sc_uw);
	IEEE80211_LOCK(ic);

	if (error != 0)
		return (error);

	return (r21a_data->newstate(vap, nstate, arg));
}

static int
urtwn_wme_update(struct ieee80211com *ic)
{
	struct urtwn_softc *sc = ic->ic_softc;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	/* don't override default WME values if WME is not actually enabled */
	if (!(ic->ic_flags & IEEE80211_F_WME))
		return 0;

	/* Do it in a process context. */
	urtwn_do_async(sc, urtwn_wme_update_cb, NULL, 0);
	return 0;
}

static void
urtwn_wme_update_cb(struct urtwn_softc *sc, void *arg)
{
	static const uint16_t ac2reg[WME_NUM_AC] = {
		R92C_EDCA_BE_PARAM,
		R92C_EDCA_BK_PARAM,
		R92C_EDCA_VI_PARAM,
		R92C_EDCA_VO_PARAM
	};
	struct ieee80211com *ic = usbwifi_ic(&sc->sc_uw);
	const struct wmeParams *wmep;
	int ac, aifs, slottime;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	usbwifi_lock_ic(&sc->sc_uw);
	slottime = (ic->ic_flags & IEEE80211_F_SHSLOT) ? 9 : 20;
	for (ac = 0; ac < WME_NUM_AC; ac++) {
		wmep = &ic->ic_wme.wme_chanParams.cap_wmeParams[ac];
		/* AIFS[AC] = AIFSN[AC] * aSlotTime + aSIFSTime. */
		aifs = wmep->wmep_aifsn * slottime + (IEEE80211_IS_CHAN_5GHZ(ic->ic_curchan) ?
			IEEE80211_DUR_OFDM_SIFS : IEEE80211_DUR_SIFS);
		urtwn_write_4(sc, ac2reg[ac],
		    SM(R92C_EDCA_PARAM_TXOP, wmep->wmep_txopLimit) |
		    SM(R92C_EDCA_PARAM_ECWMIN, wmep->wmep_logcwmin) |
		    SM(R92C_EDCA_PARAM_ECWMAX, wmep->wmep_logcwmax) |
		    SM(R92C_EDCA_PARAM_AIFS, aifs));
	}
	usbwifi_unlock_ic(&sc->sc_uw);
}

static void
urtwn_update_avgrssi(struct urtwn_softc *sc, int rate, int8_t rssi)
{
	int pwdb;

	URTWNHIST_FUNC();
	URTWNHIST_CALLARGS("rate=%jd, rsst=%jd", rate, rssi, 0, 0);

	/* Convert antenna signal to percentage. */
	if (rssi <= -100 || rssi >= 20)
		pwdb = 0;
	else if (rssi >= 0)
		pwdb = 100;
	else
		pwdb = 100 + rssi;
	if (!ISSET(sc->chip, URTWN_CHIP_88E)) {
		if (rate <= 3) {
			/* CCK gain is smaller than OFDM/MCS gain. */
			pwdb += 6;
			if (pwdb > 100)
				pwdb = 100;
			if (pwdb <= 14)
				pwdb -= 4;
			else if (pwdb <= 26)
				pwdb -= 8;
			else if (pwdb <= 34)
				pwdb -= 6;
			else if (pwdb <= 42)
				pwdb -= 2;
		}
	}
	if (sc->avg_pwdb == -1) /* Init. */
		sc->avg_pwdb = pwdb;
	else if (sc->avg_pwdb < pwdb)
		sc->avg_pwdb = ((sc->avg_pwdb * 19 + pwdb) / 20) + 1;
	else
		sc->avg_pwdb = ((sc->avg_pwdb * 19 + pwdb) / 20);

	DPRINTFN(DBG_RF, "rate=%jd rssi=%jd PWDB=%jd EMA=%jd",
		     rate, rssi, pwdb, sc->avg_pwdb);
}

static int8_t
urtwn_get_rssi(struct urtwn_softc *sc, int rate, void *physt)
{
	static const int8_t cckoff[] = { 16, -12, -26, -46 };
	struct r92c_rx_phystat *phy;
	struct r92c_rx_cck *cck;
	uint8_t rpt;
	int8_t rssi;

	URTWNHIST_FUNC();
	URTWNHIST_CALLARGS("rate=%jd", rate, 0, 0, 0);

	if (rate <= 3) {
		cck = (struct r92c_rx_cck *)physt;
		if (ISSET(sc->sc_uw.uw_flags, URTWN_FLAG_CCK_HIPWR)) {
			rpt = (cck->agc_rpt >> 5) & 0x3;
			rssi = (cck->agc_rpt & 0x1f) << 1;
		} else {
			rpt = (cck->agc_rpt >> 6) & 0x3;
			rssi = cck->agc_rpt & 0x3e;
		}
		rssi = cckoff[rpt] - rssi;
	} else {	/* OFDM/HT. */
		phy = (struct r92c_rx_phystat *)physt;
		rssi = ((le32toh(phy->phydw1) >> 1) & 0x7f) - 110;
	}
	return rssi;
}

static int8_t
urtwn_r88e_get_rssi(struct urtwn_softc *sc, int rate, void *physt)
{
	static const int8_t cckoff[] = { 20, 14, 10, -4, -16, -22, -38, -40 };
	struct r88e_rx_phystat *phy;
	uint8_t rpt;
	int8_t rssi;

	URTWNHIST_FUNC();
	URTWNHIST_CALLARGS("rate=%jd", rate, 0, 0, 0);

	phy = (struct r88e_rx_phystat *)physt;

	if (rate <= 3) {
		rpt = (phy->agc_rpt >> 5) & 0x7;
		rssi = (phy->agc_rpt & 0x1f) << 1;
		if (sc->sc_uw.uw_flags & URTWN_FLAG_CCK_HIPWR) {
			if (rpt == 2)
				rssi -= 6;
		}
		rssi = (phy->agc_rpt & 0x1f) > 27 ? -94 : cckoff[rpt] - rssi;
	} else {        /* OFDM/HT. */
		rssi = ((le32toh(phy->sq_rpt) >> 1) & 0x7f) - 110;
	}
	return rssi;
}

static int8_t
urtwn_r21a_get_rssi(struct urtwn_softc *sc, int rate, void *physt)
{
	struct r92c_rx_phystat *phy;
	struct r21a_rx_phystat *cck;
	uint8_t lna_idx;
	int8_t rssi;

	URTWNHIST_FUNC();
	URTWNHIST_CALLARGS("rate=%jd", rate, 0, 0, 0);

	rssi = 0;
	if (rate <= 3) {
		cck = (struct r21a_rx_phystat *)physt;
		lna_idx = (cck->cfosho[0] & 0xe0) >> 5;
		rssi = -6 - 2*(cck->cfosho[0] & 0x1f);	/* Pout - (2 * VGA_idx) */

		switch (lna_idx) {
		case 5:
			rssi -= 32;
			break;
		case 4:
			rssi -= 24;
			break;
		case 2:
			rssi -= 11;
			break;
		case 1:
			rssi += 5;
			break;
		case 0:
			rssi += 21;
			break;
		}
	} else {	/* OFDM/HT. */
		phy = (struct r92c_rx_phystat *)physt;
		rssi = ((le32toh(phy->phydw1) >> 1) & 0x7f) - 110;
	}
	return rssi;
}
/*
 * Handle a single frame during reception, pass on to usbwifi_enqueue()
 * for processing.
 */
static void
urtwn_rx_frame(struct urtwn_softc *sc, uint8_t *buf, int pktlen)
{
	struct ieee80211com *ic = usbwifi_ic(&sc->sc_uw);
	struct r92c_rx_desc_usb *stat;
	uint32_t rxdw0, rxdw3;
	uint8_t rate;
	int8_t rssi = 0;
	int infosz;
	stat = (struct r92c_rx_desc_usb *)buf;

	URTWNHIST_FUNC();
	URTWNHIST_CALLARGS("buf=%#jx, pktlen=%jd", (intptr_t)stat, pktlen, 0, 0);

	rxdw0 = le32toh(stat->rxdw0);
	rxdw3 = le32toh(stat->rxdw3);

	if (__predict_false(rxdw0 & (R92C_RXDW0_CRCERR | R92C_RXDW0_ICVERR))) {
		/*
		 * This should not happen since we setup our Rx filter
		 * to not receive these frames.
		 */
		DPRINTFN(DBG_RX, "CRC error", 0, 0, 0, 0);
		ieee80211_stat_add(&ic->ic_ierrors, 1);
		return;
	}

	/*
	 * XXX: This will drop most control packets.  Do we really
	 * want this in IEEE80211_M_MONITOR mode?
	 */
	pktlen = MS(rxdw0, R92C_RXDW0_PKTLEN);
	if (__predict_false(pktlen < (int)sizeof(struct ieee80211_frame_ack))) {
		DPRINTFN(DBG_RX, "packet too short %jd", pktlen, 0, 0, 0);
		ieee80211_stat_add(&ic->ic_ierrors, 1);
		return;
	}
	if (__predict_false(pktlen > MCLBYTES)) {
		DPRINTFN(DBG_RX, "packet too big %jd", pktlen, 0, 0, 0);
		ieee80211_stat_add(&ic->ic_ierrors, 1);
		return;
	}

	rate = MS(rxdw3, R92C_RXDW3_RATE);
	infosz = MS(rxdw0, R92C_RXDW0_INFOSZ) * 8;

	/* Get RSSI from PHY status descriptor if present. */
	if (infosz != 0 && (rxdw0 & R92C_RXDW0_PHYST)) {
		if (ISSET(sc->chip, URTWN_CHIP_21A))
			rssi = urtwn_r21a_get_rssi(sc, rate, &stat[1]);
		else if (!ISSET(sc->chip, URTWN_CHIP_92C))
			rssi = urtwn_r88e_get_rssi(sc, rate, &stat[1]);
		else
			rssi = urtwn_get_rssi(sc, rate, &stat[1]);
		/* Update our average RSSI. */
		urtwn_update_avgrssi(sc, rate, rssi);
	}

	DPRINTFN(DBG_RX, "Rx frame len=%jd rate=%jd infosz=%jd rssi=%jd",
	    pktlen, rate, infosz, rssi);

	/* update radiotap data if needed */
	if (__predict_false(ic->ic_flags_ext & IEEE80211_FEXT_BPF)) {
		struct urtwn_rx_radiotap_header *tap = &sc->sc_rxtap;

		tap->wr_flags = 0;
		if (!(rxdw3 & R92C_RXDW3_HT)) {
			switch (rate) {
			/* CCK. */
			case  0: tap->wr_rate =   2; break;
			case  1: tap->wr_rate =   4; break;
			case  2: tap->wr_rate =  11; break;
			case  3: tap->wr_rate =  22; break;
			/* OFDM. */
			case  4: tap->wr_rate =  12; break;
			case  5: tap->wr_rate =  18; break;
			case  6: tap->wr_rate =  24; break;
			case  7: tap->wr_rate =  36; break;
			case  8: tap->wr_rate =  48; break;
			case  9: tap->wr_rate =  72; break;
			case 10: tap->wr_rate =  96; break;
			case 11: tap->wr_rate = 108; break;
			}
		} else if (rate >= 12) {	/* MCS0~15. */
			/* Bit 7 set means HT MCS instead of rate. */
			tap->wr_rate = 0x80 | (rate - 12);
		}
		tap->wr_dbm_antsignal = rssi;
		tap->wr_chan_freq = htole16(ic->ic_curchan->ic_freq);
		tap->wr_chan_flags = htole16(ic->ic_curchan->ic_flags);
	}

	/* pass net load up the stack */
	usbwifi_enqueue(&sc->sc_uw, (uint8_t *)&stat[1] + infosz,
	    pktlen, rssi + 150, 0, 0, 0);
}

__unused static int __noinline 
urtwn_r21au_classify_intr(struct urtwn_softc *sc, void *buf, int len)
{
	struct r92c_rx_desc_usb *stat = buf;
	uint32_t rxdw2 = le32toh(stat->rxdw2);

	if (rxdw2 & R21A_RXDW2_RPT_C2H) {
		int pos = sizeof(struct r92c_rx_desc_usb);
		/* Check if Rx descriptor + command id/sequence fits. */
		if (len < pos + 2)	/* unknown, skip */
			return (RTWN_RX_DATA);

		if (((uint8_t *)buf)[pos] == R21A_C2H_TX_REPORT)
			return (RTWN_RX_TX_REPORT);
		else
			return (RTWN_RX_OTHER);
	} else
		return (RTWN_RX_DATA);
}


__unused static void __noinline
urtwn_r21a_handle_c2h_report(struct urtwn_softc *sc, uint8_t *buf, int len)
{

	URTWNHIST_FUNC(); URTWNHIST_CALLED();
	/* Skip Rx descriptor. */
	buf += sizeof(struct r92c_rx_desc_usb);
	len -= sizeof(struct r92c_rx_desc_usb);

	if (len < 2) {
		device_printf(sc->sc_uw.uw_dev, "C2H report too short (len %d)\n", len);
		return;
	}
	len -= 2;

	switch (buf[0]) {	/* command id */
	case R21A_C2H_TX_REPORT:
		/* NOTREACHED */
		KASSERT(0);
		break;
	case R21A_C2H_IQK_FINISHED:
		DPRINTFN(DBG_FN, "FW IQ calibration finished\n", 0, 0, 0, 0);
		break;
	default:
		device_printf(sc->sc_uw.uw_dev,
		    "%s: C2H report %u was not handled\n",
		    __func__, buf[0]);
	}
}

static void
urtwn_rx_loop(struct usbwifi *uw, struct usbwifi_chain *c,
    uint32_t len)
{
	struct urtwn_softc *sc = usbwifi_softc(c->uwc_uw);
	uint8_t *buf = c->uwc_buf;
	struct r92c_rx_desc_usb *stat;
	uint32_t rxdw0;
	uint32_t totlen, pktlen, infosz, npkts;

	URTWNHIST_FUNC();
	URTWNHIST_CALLARGS("chain=%#jx, len=%jd", (intptr_t)c, len, 0, 0);

	/* Get the number of encapsulated frames. */
	stat = (struct r92c_rx_desc_usb *)buf;
	if (ISSET(sc->chip, URTWN_CHIP_92EU))
		npkts = MS(le32toh(stat->rxdw2), R92E_RXDW2_PKTCNT);
	else
		npkts = MS(le32toh(stat->rxdw2), R92C_RXDW2_PKTCNT);

	DPRINTFN(DBG_RX, "Rx %jd frames in one chunk", npkts, 0, 0, 0);
	/* Process all of them. */
	while (npkts-- > 0) {
	// while (len >= sizeof(*stat)) {
		if (__predict_false(len < (int)sizeof(*stat))) {
			DPRINTFN(DBG_RX, "len(%d) is shorter than header",
			    len, 0, 0, 0);
			break;
		}
		stat = (struct r92c_rx_desc_usb *)buf;
		rxdw0 = le32toh(stat->rxdw0);

		pktlen = MS(rxdw0, R92C_RXDW0_PKTLEN);
		if (__predict_false(pktlen == 0)) {
			DPRINTFN(DBG_RX, "pktlen is 0 byte", 0, 0, 0, 0);
			break;
		}

		infosz = MS(rxdw0, R92C_RXDW0_INFOSZ) * 8;

		/* Make sure everything fits in xfer. */
		totlen = sizeof(*stat) + infosz + pktlen;
		DPRINTFN(DBG_RX, "totlen=%d len=%d", totlen, len, 0, 0);
		if (__predict_false(totlen > len)) {
			DPRINTFN(DBG_RX, "pktlen %jd(stat+%jd+%jd) > %jd",
			    totlen, infosz, pktlen, len);
			break;
		}

		/* Process 802.11 frame. */
		urtwn_rx_frame(sc, buf, pktlen);

		/* Next chunk is 128-byte aligned. */
		if (ISSET(sc->chip, URTWN_CHIP_21A)){
			if (totlen < len)
				totlen = roundup2(totlen, 8);
		}
		else
			totlen = roundup2(totlen, 128);
		buf += totlen;
		len -= totlen;
	}
	
}

static uint8_t
urtwn_get_cipher(u_int ic_cipher)
{
	uint8_t cipher;

	switch (ic_cipher) {
	case IEEE80211_CIPHER_NONE:
		cipher = R92C_TXDW1_CIPHER_NONE;
		break;
	case IEEE80211_CIPHER_WEP:
	case IEEE80211_CIPHER_TKIP:
		cipher = R92C_TXDW1_CIPHER_RC4;
		break;
	case IEEE80211_CIPHER_AES_CCM:
		cipher = R92C_TXDW1_CIPHER_AES;
		break;
	default:
		KASSERT(0);
		return (R92C_TXDW1_CIPHER_SM4);
	}

	return (cipher);
}


static void
urtwn_r21a_tx_set_ht40(struct urtwn_softc *sc, struct r21a_tx_desc_usb *txd, struct ieee80211_node *ni)
{
	/* XXX 80 Mhz */
	if (ni->ni_chan != IEEE80211_CHAN_ANYC &&
	    IEEE80211_IS_CHAN_HT40(ni->ni_chan)) {
		int prim_chan;

		if (IEEE80211_IS_CHAN_HT40U(ni->ni_chan))
			prim_chan = (R21A_TXDW5_PRIM_CHAN_20_80_2);
		else
			prim_chan = (R21A_TXDW5_PRIM_CHAN_20_80_3);

		txd->txdw5 |= htole32(SM(R21A_TXDW5_DATA_BW,
		    R21A_TXDW5_DATA_BW40));
		txd->txdw5 |= htole32(SM(R21A_TXDW5_DATA_PRIM_CHAN,
		    prim_chan));
	}
}

static void
urtwn_r21a_tx_protection(struct urtwn_softc *sc, struct r21a_tx_desc_usb *txd,
    enum ieee80211_protmode mode, uint8_t ridx)
{
	struct ieee80211com *ic = usbwifi_ic(&sc->sc_uw);
	uint8_t rate;

	switch (mode) {
	case IEEE80211_PROT_CTSONLY:
		txd->txdw3 |= htole32(R21A_TXDW3_CTS2SELF);
		break;
	case IEEE80211_PROT_RTSCTS:
		txd->txdw3 |= htole32(R21A_TXDW3_RTSEN);
		break;
	default:
		break;
	}

	if (mode == IEEE80211_PROT_CTSONLY ||
	    mode == IEEE80211_PROT_RTSCTS) {
		if (ridx >= RTWN_RIDX_HT_MCS(0))
			rate = rtwn_ctl_mcsrate(ic->ic_rt, ridx);
		else
			rate = ieee80211_ctl_rate(ic->ic_rt, ridx2rate[ridx]);
		ridx = rate2ridx(IEEE80211_RV(rate));

		txd->txdw4 |= htole32(SM(R21A_TXDW4_RTSRATE, ridx));
		/* RTS rate fallback limit (max). */
		txd->txdw4 |= htole32(SM(R21A_TXDW4_RTSRATE_FB_LMT, 0xf));

		if (RTWN_RATE_IS_CCK(ridx) && ridx != RTWN_RIDX_CCK1 &&
		    (ic->ic_flags & IEEE80211_F_SHPREAMBLE))
			txd->txdw5 |= htole32(R21A_TXDW5_RTS_SHORT);
	}
}

static void
urtwn_r21a_tx_raid(struct urtwn_softc *sc, struct r21a_tx_desc_usb *txd,
    struct ieee80211_node *ni, int ismcast)
{
	struct ieee80211com *ic = usbwifi_ic(&sc->sc_uw);
	struct ieee80211vap *vap = ni->ni_vap;
	struct ieee80211_channel *chan;
	enum ieee80211_phymode mode;
	uint8_t raid;

	chan = (ni->ni_chan != IEEE80211_CHAN_ANYC) ?
		ni->ni_chan : ic->ic_curchan;
	mode = ieee80211_chan2mode(chan);

	/* NB: group addressed frames are done at 11bg rates for now */
	if (ismcast || !(ni->ni_flags & IEEE80211_NODE_HT)) {
		switch (mode) {
		case IEEE80211_MODE_11A:
		case IEEE80211_MODE_11B:
		case IEEE80211_MODE_11G:
			break;
		case IEEE80211_MODE_11NA:
			mode = IEEE80211_MODE_11A;
			break;
		case IEEE80211_MODE_11NG:
			mode = IEEE80211_MODE_11G;
			break;
		default:
			device_printf(sc->sc_uw.uw_dev, "unknown mode(1) %d!\n",
			    ic->ic_curmode);
			return;
		}
	}

	switch (mode) {
	case IEEE80211_MODE_11A:
		raid = R21A_RAID_11G;
		break;
	case IEEE80211_MODE_11B:
		raid = R21A_RAID_11B;
		break;
	case IEEE80211_MODE_11G:
		if (vap->iv_flags & IEEE80211_F_PUREG)
			raid = R21A_RAID_11G;
		else
			raid = R21A_RAID_11BG;
		break;
	case IEEE80211_MODE_11NA:
		if (sc->ntxchains == 1)
			raid = R21A_RAID_11GN_1;
		else
			raid = R21A_RAID_11GN_2;
		break;
	case IEEE80211_MODE_11NG:
		if (sc->ntxchains == 1) {
			if (IEEE80211_IS_CHAN_HT40(chan))
				raid = R21A_RAID_11BGN_1_40;
			else
				raid = R21A_RAID_11BGN_1;
		} else {
			if (IEEE80211_IS_CHAN_HT40(chan))
				raid = R21A_RAID_11BGN_2_40;
			else
				raid = R21A_RAID_11BGN_2;
		}
		break;
	default:
		/* TODO: 80 MHz / 11ac */
		device_printf(sc->sc_uw.uw_dev, "unknown mode(2) %d!\n", mode);
		return;
	}

	txd->txdw1 |= htole32(SM(R21A_TXDW1_RAID, raid));
}
static void
urtwn_r21a_tx_set_sgi(struct urtwn_softc *sc, struct r21a_tx_desc_usb *txd, struct ieee80211_node *ni)
{
	struct ieee80211vap *vap = ni->ni_vap;

	if ((vap->iv_flags_ht & IEEE80211_FHT_SHORTGI20) &&	/* HT20 */
	    (ni->ni_htcap & IEEE80211_HTCAP_SHORTGI20))
		txd->txdw5 |= htole32(R21A_TXDW5_DATA_SHORT);
	else if (ni->ni_chan != IEEE80211_CHAN_ANYC &&		/* HT40 */
	    IEEE80211_IS_CHAN_HT40(ni->ni_chan) &&
	    (ni->ni_htcap & IEEE80211_HTCAP_SHORTGI40) &&
	    (vap->iv_flags_ht & IEEE80211_FHT_SHORTGI40))
		txd->txdw5 |= htole32(R21A_TXDW5_DATA_SHORT);
}

static void
urtwn_r21a_tx_set_ldpc(struct urtwn_softc *sc, struct r21a_tx_desc_usb *txd,
    struct ieee80211_node *ni)
{
	struct ieee80211vap *vap = ni->ni_vap;

	if ((vap->iv_flags_ht & IEEE80211_FHT_LDPC_TX) &&
	    (ni->ni_htcap & IEEE80211_HTCAP_LDPC))
		txd->txdw5 |= htole32(R21A_TXDW5_DATA_LDPC);
}

/* Copy packet into a USB transfer and return length used */
static unsigned
urtwn_tx_prepare(struct usbwifi *uw, struct usbwifi_chain *chain,
    uint8_t qid)
{
	struct urtwn_softc *sc = usbwifi_softc(uw);
	return sc->sc_tx_prepare(uw, chain, qid);
}
static unsigned
urtwn_r88e_tx_prepare(struct usbwifi *uw, struct usbwifi_chain *chain,
    uint8_t qid)
{
	
	URTWNHIST_FUNC(); URTWNHIST_CALLED();
	URTWNHIST_CALLARGS("qid=%d, chain=%zd", qid, (uintptr_t)chain, 0, 0);
	struct ieee80211_node *ni = chain->uwc_ni;
	struct mbuf *m = chain->uwc_mbuf;
	struct ieee80211com *ic = usbwifi_ic(uw);
	struct urtwn_softc *sc = usbwifi_softc(uw);
	struct ieee80211_frame *wh;
	struct r92c_tx_desc_usb *txd;
	unsigned i, padsize, txd_len;
	uint16_t seq, sum;
	uint8_t type, tid;
	int hasqos;

	wh = mtod(m, struct ieee80211_frame *);
	type = wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK;
	txd_len = sizeof(*txd);

	if (!ISSET(sc->chip, URTWN_CHIP_92EU))
		txd_len = 32;

	if (wh->i_fc[1] & IEEE80211_FC1_PROTECTED) {
		if (ieee80211_crypto_encap(ni, m) == NULL) {
			DPRINTFN(DBG_TX, "failed to encrypt packet",
			    0, 0, 0, 0);
			return 0;
		}

		/* packet header may have moved, reset our local pointer */
		wh = mtod(m, struct ieee80211_frame *);
	}

	/* non-qos data frames */
	tid = R92C_TXDW1_QSEL_BE;
	if ((hasqos = IEEE80211_QOS_HAS_SEQ(wh))) {
		/* data frames in 11n mode */
		uint8_t *frm;
		uint16_t qos;

		frm = ieee80211_getqos(wh);
		qos = le16toh(*(const uint16_t *)frm);
		tid = qos & IEEE80211_QOS_TID;
	} else if (type != IEEE80211_FC0_TYPE_DATA) {
		tid = R92C_TXDW1_QSEL_MGNT;
	}

	if (((txd_len + m->m_pkthdr.len) % 64) == 0) /* XXX: 64 */
		padsize = 8;
	else
		padsize = 0;

	if (ISSET(sc->chip, URTWN_CHIP_92EU))
		padsize = 0;

	/* Fill Tx descriptor. */
	txd = (struct r92c_tx_desc_usb *)chain->uwc_buf;
	memset(txd, 0, txd_len + padsize);

	txd->txdw0 |= htole32(
	    SM(R92C_TXDW0_PKTLEN, m->m_pkthdr.len) |
	    SM(R92C_TXDW0_OFFSET, txd_len));
	if (!ISSET(sc->chip, URTWN_CHIP_92EU)) {
		txd->txdw0 |= htole32(
		    R92C_TXDW0_OWN | R92C_TXDW0_FSG | R92C_TXDW0_LSG);
	}

	if (IEEE80211_IS_MULTICAST(wh->i_addr1))
		txd->txdw0 |= htole32(R92C_TXDW0_BMCAST);

	/* fix pad field */
	if (padsize > 0) {
		DPRINTFN(DBG_TX, "padding: size=%ju", padsize, 0, 0, 0);
		txd->txdw1 |= htole32(SM(R92C_TXDW1_PKTOFF, (padsize / 8)));
	}

	if (!IEEE80211_IS_MULTICAST(wh->i_addr1) &&
	    type == IEEE80211_FC0_TYPE_DATA) {
		uint8_t raid;

		if (ic->ic_curmode == IEEE80211_MODE_11B) {
			raid = R92C_RAID_11B;
			txd->txdw5 |= htole32(SM(R92C_TXDW5_DATARATE, 0));
		} else if (ic->ic_curmode == IEEE80211_MODE_11G) {
			raid = R92C_RAID_11BG;
			txd->txdw5 |= htole32(SM(R92C_TXDW5_DATARATE, 11));
		} else {	/* IEEE80211_MODE_11NG */
			raid = R92C_RAID_11GN;
			txd->txdw5 |= htole32(SM(R92C_TXDW5_DATARATE, 19));
			txd->txdw5 |= htole32(R92C_TXDW5_SGI);
		}

		DPRINTFN(DBG_TX,
		    "data packet: tid=%jd, raid=%jd", tid, raid, 0, 0);

		if (!ISSET(sc->chip, URTWN_CHIP_92C)) {
			txd->txdw1 |= htole32(
			    SM(R88E_TXDW1_MACID, RTWN_MACID_BSS) |
			    SM(R92C_TXDW1_QSEL, tid) |
			    SM(R92C_TXDW1_RAID, raid) |
			    R92C_TXDW1_AGGBK);
		} else
			txd->txdw1 |= htole32(
			    SM(R92C_TXDW1_MACID, RTWN_MACID_BSS) |
			    SM(R92C_TXDW1_QSEL, tid) |
			    SM(R92C_TXDW1_RAID, raid) |
			    R92C_TXDW1_AGGBK);

		if (ISSET(sc->chip, URTWN_CHIP_88E))
			txd->txdw2 |= htole32(R88E_TXDW2_AGGBK);
		if (ISSET(sc->chip, URTWN_CHIP_92EU))
			txd->txdw3 |= htole32(R92E_TXDW3_AGGBK);

		if (hasqos) {
			txd->txdw4 |= htole32(R92C_TXDW4_QOS);
		}

		if (ic->ic_flags & IEEE80211_F_USEPROT) {
			/* for 11g */
			if (ic->ic_protmode == IEEE80211_PROT_CTSONLY) {
				txd->txdw4 |= htole32(R92C_TXDW4_CTS2SELF |
				    R92C_TXDW4_HWRTSEN);
			} else if (ic->ic_protmode == IEEE80211_PROT_RTSCTS) {
				txd->txdw4 |= htole32(R92C_TXDW4_RTSEN |
				    R92C_TXDW4_HWRTSEN);
			}
		}
		/* Send RTS at OFDM24. */
		txd->txdw4 |= htole32(SM(R92C_TXDW4_RTSRATE, 8));
		txd->txdw5 |= htole32(0x0001ff00);
		/* Send data at OFDM54. */
		if (ISSET(sc->chip, URTWN_CHIP_88E))
			txd->txdw5 |= htole32(0x13 & 0x3f);
	} else if (type == IEEE80211_FC0_TYPE_MGT) {
		DPRINTFN(DBG_TX, "mgmt packet", 0, 0, 0, 0);
		txd->txdw1 |= htole32(
		    SM(R92C_TXDW1_MACID, RTWN_MACID_BSS) |
		    SM(R92C_TXDW1_QSEL, R92C_TXDW1_QSEL_MGNT) |
		    SM(R92C_TXDW1_RAID, R92C_RAID_11B));

		/* Force CCK1. */
		txd->txdw4 |= htole32(R92C_TXDW4_DRVRATE);
		/* Use 1Mbps */
		txd->txdw5 |= htole32(SM(R92C_TXDW5_DATARATE, 0));
	} else {
		/* broadcast or multicast packets */
		DPRINTFN(DBG_TX, "bc or mc packet", 0, 0, 0, 0);
		txd->txdw1 |= htole32(
		    SM(R92C_TXDW1_MACID, RTWN_MACID_BC) |
		    SM(R92C_TXDW1_RAID, R92C_RAID_11B));

		/* Force CCK1. */
		txd->txdw4 |= htole32(R92C_TXDW4_DRVRATE);
		/* Use 1Mbps */
		txd->txdw5 |= htole32(SM(R92C_TXDW5_DATARATE, 0));
	}
	/* Set sequence number */
	seq = LE_READ_2(&wh->i_seq[0]) >> IEEE80211_SEQ_SEQ_SHIFT;
	if (!ISSET(sc->chip, URTWN_CHIP_92EU)) {
		txd->txdseq |= htole16(seq);

		if (!hasqos) {
			/* Use HW sequence numbering for non-QoS frames. */
			txd->txdw4  |= htole32(R92C_TXDW4_HWSEQ);
			txd->txdseq |= htole16(R92C_HWSEQ_EN);
		}
	} else {
		txd->txdseq2 |= htole16((seq & R92E_HWSEQ_MASK) <<
		    R92E_HWSEQ_SHIFT);
		if (!hasqos) {
			/* Use HW sequence numbering for non-QoS frames. */
			txd->txdw4  |= htole32(R92C_TXDW4_HWSEQ);
			txd->txdw7  |= htole16(R92C_HWSEQ_EN);
		}
	}

	/* Compute Tx descriptor checksum. */
	sum = 0;
	for (i = 0; i < R92C_TXDESC_SUMSIZE / 2; i++)
		sum ^= ((uint16_t *)txd)[i];
	txd->txdsum = sum;	/* NB: already little endian. */

	m_copydata(m, 0, m->m_pkthdr.len, (char *)&txd[0] + txd_len + padsize);

	return txd_len + m->m_pkthdr.len + padsize;
}

static unsigned
urtwn_r21a_tx_prepare(struct usbwifi *uw, struct usbwifi_chain *chain,
    uint8_t qid)
{
	struct ieee80211_node *ni = chain->uwc_ni;
	struct ieee80211vap *vap =  ni->ni_vap;
	struct ieee80211com *ic = usbwifi_ic(uw);
	struct ieee80211_frame *wh;
	struct urtwn_softc *sc = usbwifi_softc(uw);
	const struct ieee80211_txparam *tp = ni->ni_txparms;
	struct ieee80211_key *k = NULL;
	struct mbuf *m = chain->uwc_mbuf;
	struct r21a_tx_desc_usb *txd;
	unsigned i, txd_len;
	u_int cipher;
	uint16_t sum;
	uint8_t type, tid, rate, ridx, qsel,qos;
	int hasqos, ismcast, macid;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	wh = mtod(m, struct ieee80211_frame *);
	type = wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK;
	hasqos = IEEE80211_QOS_HAS_SEQ(wh);
	ismcast = IEEE80211_IS_MULTICAST(wh->i_addr1);
	txd_len = sizeof(*txd);

	/* Select TX ring for this frame. */
	if (hasqos) {
		qos = ((const struct ieee80211_qosframe *)wh)->i_qos[0];
		tid = qos & IEEE80211_QOS_TID;
	} else {
		qos = 0;
		tid = 0;
	}

	/* Choose a TX rate index. */
	if (type == IEEE80211_FC0_TYPE_MGT ||
	    type == IEEE80211_FC0_TYPE_CTL ||
	    (m->m_flags & M_EAPOL) != 0)
		rate = tp->mgmtrate;
	else if (ismcast)
		rate = tp->mcastrate;
	else if (tp->ucastrate != IEEE80211_FIXED_RATE_NONE)
		rate = tp->ucastrate;
	else {
		(void) ieee80211_ratectl_rate(ni, NULL, 0);
		rate = ni->ni_txrate;
	}

	ridx = rate2ridx(rate);

	cipher = IEEE80211_CIPHER_NONE;
	if (wh->i_fc[1] & IEEE80211_FC1_PROTECTED) {
		k = ieee80211_crypto_encap(ni, m);
		if (k == NULL) {
			device_printf(uw->uw_dev,
			    "ieee80211_crypto_encap returns NULL.\n");
			return (ENOBUFS);
		}
		if (!(k->wk_flags & IEEE80211_KEY_SWCRYPT))
			cipher = k->wk_cipher->ic_cipher;

		/* in case packet header moved, reset pointer */
		wh = mtod(m, struct ieee80211_frame *);
	}
	/* Fill Tx descriptor. */

	txd = (struct r21a_tx_desc_usb *)chain->uwc_buf;
	memset(txd, 0,  txd_len);
	txd->flags0 |= R21A_FLAGS0_LSG | R21A_FLAGS0_FSG | R21A_FLAGS0_OWN;
	if (ismcast)
		txd->flags0 |= R21A_FLAGS0_BMCAST;

	if (!ismcast) {
		/* Unicast frame, check if an ACK is expected. */
		if (!qos || (qos & IEEE80211_QOS_ACKPOLICY) !=
		    IEEE80211_QOS_ACKPOLICY_NOACK) {
			txd->txdw4 = htole32(R21A_TXDW4_RETRY_LMT_ENA);
			txd->txdw4 |= htole32(SM(R21A_TXDW4_RETRY_LMT,
			    tp->maxretry));
		}

		macid = 0; /* BSS. */
		if (type == IEEE80211_FC0_TYPE_DATA) {
			qsel = tid % RTWN_MAX_TID;

			if (m->m_flags & M_AMPDU_MPDU) {
				txd->txdw2 |= htole32(R21A_TXDW2_AGGEN);
				txd->txdw2 |= htole32(SM(R21A_TXDW2_AMPDU_DEN,
				    vap->iv_ampdu_density));
				txd->txdw3 |= htole32(SM(R21A_TXDW3_MAX_AGG,
				    0x1f));	/* XXX */
			} else
				txd->txdw2 |= htole32(R21A_TXDW2_AGGBK);

			txd->txdw2 |= htole32(R21A_TXDW2_SPE_RPT);

			if (RTWN_RATE_IS_CCK(ridx) && ridx != RTWN_RIDX_CCK1 &&
			    (ic->ic_flags & IEEE80211_F_SHPREAMBLE))
				txd->txdw5 |= htole32(R21A_TXDW5_DATA_SHORT);

			if (ridx >= RTWN_RIDX_HT_MCS(0)) {
				urtwn_r21a_tx_set_ht40(sc, txd, ni);
				urtwn_r21a_tx_set_sgi(sc, txd, ni);
				urtwn_r21a_tx_set_ldpc(sc, txd, ni);
			}

			if (rate & IEEE80211_RATE_MCS) {
				urtwn_r21a_tx_protection(sc, txd,
				    ic->ic_htprotmode, ridx);
			} else if (ic->ic_flags & IEEE80211_F_USEPROT)
				urtwn_r21a_tx_protection(sc, txd, ic->ic_protmode, ridx);

			/* Data rate fallback limit (max). */
			txd->txdw4 |= htole32(SM(R21A_TXDW4_DATARATE_FB_LMT,
			    0x1f));

		} else {/* IEEE80211_FC0_TYPE_MGT */
			DPRINTFN(DBG_TX, "mgmt packet", 0, 0, 0, 0);
			qsel = R21A_TXDW1_QSEL_MGNT;		
		}
	} else {
		macid = 1;
		qsel = R21A_TXDW1_QSEL_MGNT;
	}

	txd->txdw1 |= htole32(SM(R21A_TXDW1_QSEL, qsel));
	txd->txdw1 |= htole32(SM(R21A_TXDW1_MACID, macid));
	txd->txdw4 |= htole32(SM(R21A_TXDW4_DATARATE, ridx));
	/* XXX recheck for non-21au */
	txd->txdw6 |= htole32(SM(R21A_TXDW6_MBSSID, 0));
	urtwn_r21a_tx_raid(sc, txd, ni, ismcast);

	/* Force this rate if needed. */
	txd->txdw3 |= htole32(R21A_TXDW3_DRVRATE);

	if (!hasqos) {
		/* Use HW sequence numbering for non-QoS frames. */
		txd->txdw8 |= htole32(R21A_TXDW8_HWSEQ_EN);
		txd->txdw3 |= htole32(SM(R21A_TXDW3_SEQ_SEL, 0));
	} else {
		uint16_t seqno;

		if (m->m_flags & M_AMPDU_MPDU) {
			seqno = ni->ni_txseqs[tid];
			ni->ni_txseqs[tid]++;
		} else
			seqno = M_SEQNO_GET(m) % IEEE80211_SEQ_RANGE;

		/* Set sequence number. */
		txd->txdw9 |= htole32(SM(R21A_TXDW9_SEQ, seqno));
	}
	txd->txdw1 = htole32(SM(R21A_TXDW1_CIPHER, urtwn_get_cipher(cipher)));

	txd->pktlen = htole16(m->m_pkthdr.len);
	txd->offset = txd_len;
	txd->flags0 |= R21A_FLAGS0_OWN;
	/* Compute Tx descriptor checksum. */
	sum = 0;
	for (i = 0; i < R92C_TXDESC_SUMSIZE / 2; i++)
		sum ^= ((uint16_t *)txd)[i];
	txd->txdsum = sum;	/* NB: already little endian. */

	m_copydata(m, 0, m->m_pkthdr.len, (caddr_t)&txd[1]);
	return txd_len + m->m_pkthdr.len;
}

static void
urtwn_watchdog(void *arg)
{
	struct urtwn_softc *sc = arg;
	struct ieee80211com *ic = usbwifi_ic(&sc->sc_uw);

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	if (sc->tx_timer > 0) {
		if (--sc->tx_timer == 0) {
			device_printf(sc->sc_uw.uw_dev, "device timeout\n");
			ieee80211_stat_add(&ic->ic_oerrors, 1);
			ieee80211_restart_all(ic);
			return;
		}
		callout_schedule(&sc->sc_watchdog_to, hz);
	}
}

static __inline void
urtwn_postattach(struct urtwn_softc *sc)
{

	return sc->sc_postattach(sc);
}

static __inline void
urtwn_vap_preattach(struct urtwn_softc *sc, struct ieee80211vap *vap)
{

	return sc->sc_vap_preattach(sc, vap);
}

static void
urtwn_r21au_vap_preattach(struct urtwn_softc *sc, struct ieee80211vap *vap)
{
	struct urtwn_r21a_data *r21a_data = sc->sc_chip_priv.data;

	/*
	struct urtwn_vap *uvap = (struct urtwn_vap*)vap;
	struct ifnet *ifp = vap->iv_ifp;

	ifp->if_capabilities = IFCAP_RXCSUM | IFCAP_RXCSUM_IPV6;
	ifp->if_capenable |= IFCAP_RXCSUM;
	ifp->if_capenable |= IFCAP_RXCSUM_IPV6;
	*/

	/* Install DFS newstate handler (non-monitor vaps only). */
	r21a_data->newstate = vap->iv_newstate;
	vap->iv_newstate = urtwn_r21au_newstate;
}

/*
 * Create a VAP node for use with the urtwn driver.
 * We do not use the common struct ieee80211vap, but a derived
 * structure so we can hook the newstate function by our own
 * version (and update some LED states on transitions). 
 */
static struct ieee80211vap *
urtwn_vap_create(struct ieee80211com *ic,  const char name[IFNAMSIZ],
    int unit, enum ieee80211_opmode opmode, int flags,
    const uint8_t bssid[IEEE80211_ADDR_LEN],
    const uint8_t macaddr[IEEE80211_ADDR_LEN])
{
	struct urtwn_vap *vap;
	struct urtwn_softc *sc = ic->ic_softc;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();
	URTWNHIST_CALLARGS("name=%s opmode=%d", name, opmode,0,0);

	/* Allocate the vap and setup. */
	vap = kmem_zalloc(sizeof(*vap), KM_SLEEP);
	if (ieee80211_vap_setup(ic, &vap->vap, name, unit, opmode,
	    flags | IEEE80211_CLONE_NOBEACONS, bssid) != 0) {
		kmem_free(vap, sizeof(*vap));
		return NULL;
	}

	urtwn_vap_preattach(sc, &vap->vap);

	/* Override state transition machine. */
	vap->newstate = vap->vap.iv_newstate;
	vap->vap.iv_newstate = urtwn_newstate;
	vap->vap.iv_debug = 0xffffffff;

	/* Finish setup */
	ieee80211_vap_attach(&vap->vap, ieee80211_media_change,
	    ieee80211_media_status, macaddr);

	ic->ic_opmode = opmode;

	return &vap->vap;
}

static void
urtwn_vap_delete(struct ieee80211vap *vap)
{
	struct urtwn_vap *my_vap = (struct urtwn_vap *)vap;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	ieee80211_vap_detach(vap);
	kmem_free(my_vap, sizeof(*my_vap));
}

static void
urtwn_scan_start(struct ieee80211com *ic)
{
	//uint32_t reg;
	//int s;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	IEEE80211_LOCK(ic);
	ic->ic_flags |= IEEE80211_F_SCAN;
	IEEE80211_UNLOCK(ic);
	/*
	 * Not sure what to do here yet.  Try #1:  do what was in the
	 * state machine.  NNN
	 */
#if NOTWITHSTATEMACHINEOVERRIDE
	/*
	 * Begin of scanning
	 */
        
	usbwifi_lock_ic(&sc->sc_uw);

	/* Set gain for scanning. */
	reg = urtwn_bb_read(sc, R92C_OFDM0_AGCCORE1(0));
	reg = RW(reg, R92C_OFDM0_AGCCORE1_GAIN, 0x20);
	urtwn_bb_write(sc, R92C_OFDM0_AGCCORE1(0), reg);

	if (!ISSET(sc->chip, URTWN_CHIP_88E)) {
		reg = urtwn_bb_read(sc, R92C_OFDM0_AGCCORE1(1));
		reg = RW(reg, R92C_OFDM0_AGCCORE1_GAIN, 0x20);
		urtwn_bb_write(sc, R92C_OFDM0_AGCCORE1(1), reg);
	}

	/* Set media status to 'No Link'. */
	urtwn_set_nettype0_msr(sc, R92C_MSR_NOLINK);

	/* Allow Rx from any BSSID. */
	urtwn_write_4(sc, R92C_RCR,
	    urtwn_read_4(sc, R92C_RCR) &
	    ~(R92C_RCR_CBSSID_DATA | R92C_RCR_CBSSID_BCN));

	/* Stop Rx of data frames. */
	urtwn_write_2(sc, R92C_RXFLTMAP2, 0);

	/* Disable update TSF */
	urtwn_write_1(sc, R92C_BCN_CTRL,
	    urtwn_read_1(sc, R92C_BCN_CTRL) |
	    R92C_BCN_CTRL_DIS_TSF_UDT0);

	/* Make link LED blink during scan. */
	urtwn_set_led(sc, URTWN_LED_LINK, !sc->ledlink);
        
	/* Pause AC Tx queues. */
	
	urtwn_write_1(sc, R92C_TXPAUSE,
	    urtwn_read_1(sc, R92C_TXPAUSE) | 0x0f);

	urtwn_set_chan(sc, ic->ic_curchan,
	    IEEE80211_HTINFO_2NDCHAN_NONE);

	usbwifi_unlock_ic(&sc->sc_uw);
#endif
} 

static void
urtwn_scan_end(struct ieee80211com *ic)
{

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	IEEE80211_LOCK(ic);
	ic->ic_flags &= ~IEEE80211_F_SCAN;
	IEEE80211_UNLOCK(ic);

#ifdef NOTWITHSTATEMACHINEOVERRIDE
	/*
	 * End of scanning
	 */

	usbwifi_lock_ic(&sc->sc_uw);
        
	/* flush 4-AC Queue after site_survey */
	urtwn_write_1(sc, R92C_TXPAUSE, 0x0);
        
	/* Allow Rx from our BSSID only. */
	urtwn_write_4(sc, R92C_RCR,
	    urtwn_read_4(sc, R92C_RCR) |
	    R92C_RCR_CBSSID_DATA | R92C_RCR_CBSSID_BCN);

	/* Turn link LED off. */
	urtwn_set_led(sc, URTWN_LED_LINK, 0);

	usbwifi_unlock_ic(&sc->sc_uw);
#endif
} 

static void
urtwn_r21a_scan_start(struct ieee80211com *ic)
{
	struct urtwn_softc *sc = ic->ic_softc;
	struct urtwn_r21a_data *r21a_data = sc->sc_chip_priv.data;
	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	IEEE80211_LOCK(ic);
	ic->ic_flags |= IEEE80211_F_SCAN;
	IEEE80211_UNLOCK(ic);

	usbwifi_lock_ic(&sc->sc_uw);

	if (r21a_data->flags & R21A_RADAR_ENABLED) {
		while (taskqueue_cancel_timeout(sc->sc_tq,
		    &r21a_data->chan_check, NULL) != 0) {
			taskqueue_drain_timeout(sc->sc_tq,
			    &r21a_data->chan_check);
		}
		urtwn_r21au_dfs_radar_disable(sc);
		DPRINTFN(DBG_FN, "%s: radar detection was (temporarily) disabled\n",
		    __func__, 0, 0, 0);
	}
	
	/* Pause beaconing. */
	urtwn_setbits_4(sc, R92C_RCR, R92C_RCR_CBSSID_BCN, 0);
	urtwn_set_led(sc, URTWN_LED_LINK, 1);
	
	usbwifi_unlock_ic(&sc->sc_uw);
}

static void
urtwn_r21a_scan_end(struct ieee80211com *ic)
{
	struct urtwn_softc *sc = ic->ic_softc;
	struct urtwn_r21a_data *r21a_data = sc->sc_chip_priv.data;
	int error;
	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	IEEE80211_LOCK(ic);
	ic->ic_flags &= ~IEEE80211_F_SCAN;
	IEEE80211_UNLOCK(ic);

	usbwifi_lock_ic(&sc->sc_uw);
	if (r21a_data->flags & R21A_RADAR_ENABLED) {
		error = urtwn_r21au_dfs_radar_enable(sc);
		if (error != 0) {
			device_printf(sc->sc_uw.uw_dev,
			    "%s: cannot re-enable radar detection\n",
			    __func__);

			/* XXX */
			ieee80211_restart_all(ic);
			return;
		}
		DPRINTFN(DBG_FN,
		    "%s: radar detection was re-enabled\n", __func__, 0, 0, 0);

		taskqueue_enqueue_timeout(sc->sc_tq,
		    &r21a_data->chan_check, URTWN_R21AU_RADAR_CHECK_PERIOD);
	}

	urtwn_setbits_4(sc, R92C_RCR, 0, R92C_RCR_CBSSID_BCN);
	urtwn_set_led(sc, URTWN_LED_LINK, 0);
	
	usbwifi_unlock_ic(&sc->sc_uw);
}

static void
urtwn_set_channel(struct ieee80211com *ic)
{
	struct urtwn_softc *sc = ic->ic_softc;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	usbwifi_lock_ic(&sc->sc_uw);
	if(ISSET(sc->chip, URTWN_CHIP_21A))
		urtwn_r21a_set_chan(sc, ic->ic_curchan);
	else
		urtwn_set_chan(sc, ic->ic_curchan);
	usbwifi_unlock_ic(&sc->sc_uw);
} 

static void
urtwn_get_radiocaps(struct ieee80211com *ic,
    int maxchans, int *nchans, struct ieee80211_channel chans[])
{
	struct urtwn_softc *sc = ic->ic_softc;
	uint8_t bands[IEEE80211_MODE_BYTES];

	memset(bands, 0, sizeof(bands));
	setbit(bands, IEEE80211_MODE_11B);
	setbit(bands, IEEE80211_MODE_11G);
	setbit(bands, IEEE80211_MODE_11NG);
	ieee80211_add_channels_default_2ghz(chans, maxchans, nchans, bands, 0);

	if (sc->chan_num_5ghz != 0){
		setbit(bands, IEEE80211_MODE_11A);
		setbit(bands, IEEE80211_MODE_11NA);
		ieee80211_add_channel_list_5ghz(chans, maxchans, nchans,
		    sc->chan_list_5ghz, sc->chan_num_5ghz, bands, 0);
	}
}

static __inline int
urtwn_power_on(struct urtwn_softc *sc)
{

	return sc->sc_power_on(sc);
}

static int
urtwn_r92c_power_on(struct urtwn_softc *sc)
{
	uint32_t reg;
	int ntries;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	usbwifi_isowned_ic(&sc->sc_uw);

	/* Wait for autoload done bit. */
	for (ntries = 0; ntries < 1000; ntries++) {
		if (urtwn_read_1(sc, R92C_APS_FSMCO) & R92C_APS_FSMCO_PFM_ALDN)
			break;
		DELAY(5);
	}
	if (ntries == 1000) {
		aprint_error_dev(sc->sc_uw.uw_dev,
		    "timeout waiting for chip autoload\n");
		return ETIMEDOUT;
	}

	/* Unlock ISO/CLK/Power control register. */
	urtwn_write_1(sc, R92C_RSV_CTRL, 0);
	DELAY(5);
	/* Move SPS into PWM mode. */
	urtwn_write_1(sc, R92C_SPS0_CTRL, 0x2b);
	DELAY(5);

	reg = urtwn_read_1(sc, R92C_LDOV12D_CTRL);
	if (!(reg & R92C_LDOV12D_CTRL_LDV12_EN)) {
		urtwn_write_1(sc, R92C_LDOV12D_CTRL,
		    reg | R92C_LDOV12D_CTRL_LDV12_EN);
		DELAY(100);
		urtwn_write_1(sc, R92C_SYS_ISO_CTRL,
		    urtwn_read_1(sc, R92C_SYS_ISO_CTRL) &
		    ~R92C_SYS_ISO_CTRL_MD2PP);
	}

	/* Auto enable WLAN. */
	urtwn_write_2(sc, R92C_APS_FSMCO,
	    urtwn_read_2(sc, R92C_APS_FSMCO) | R92C_APS_FSMCO_APFM_ONMAC);
	for (ntries = 0; ntries < 1000; ntries++) {
		if (!(urtwn_read_2(sc, R92C_APS_FSMCO) &
		    R92C_APS_FSMCO_APFM_ONMAC))
			break;
		DELAY(100);
	}
	if (ntries == 1000) {
		aprint_error_dev(sc->sc_uw.uw_dev,
		    "timeout waiting for MAC auto ON\n");
		return ETIMEDOUT;
	}

	/* Enable radio, GPIO and LED functions. */
	KASSERT((R92C_APS_FSMCO_AFSM_HSUS | R92C_APS_FSMCO_PDN_EN |
	    R92C_APS_FSMCO_PFM_ALDN) == 0x0812);
	urtwn_write_2(sc, R92C_APS_FSMCO,
	    R92C_APS_FSMCO_AFSM_HSUS |
	    R92C_APS_FSMCO_PDN_EN |
	    R92C_APS_FSMCO_PFM_ALDN);

	/* Release RF digital isolation. */
	urtwn_write_2(sc, R92C_SYS_ISO_CTRL,
	    urtwn_read_2(sc, R92C_SYS_ISO_CTRL) & ~R92C_SYS_ISO_CTRL_DIOR);

	/* Initialize MAC. */
	urtwn_write_1(sc, R92C_APSD_CTRL,
	    urtwn_read_1(sc, R92C_APSD_CTRL) & ~R92C_APSD_CTRL_OFF);
	for (ntries = 0; ntries < 200; ntries++) {
		if (!(urtwn_read_1(sc, R92C_APSD_CTRL) &
		    R92C_APSD_CTRL_OFF_STATUS))
			break;
		DELAY(5);
	}
	if (ntries == 200) {
		aprint_error_dev(sc->sc_uw.uw_dev,
		    "timeout waiting for MAC initialization\n");
		return ETIMEDOUT;
	}

	/* Enable MAC DMA/WMAC/SCHEDULE/SEC blocks. */
	reg = urtwn_read_2(sc, R92C_CR);
	reg |= R92C_CR_HCI_TXDMA_EN | R92C_CR_HCI_RXDMA_EN |
	    R92C_CR_TXDMA_EN | R92C_CR_RXDMA_EN | R92C_CR_PROTOCOL_EN |
	    R92C_CR_SCHEDULE_EN | R92C_CR_MACTXEN | R92C_CR_MACRXEN |
	    R92C_CR_ENSEC;
	urtwn_write_2(sc, R92C_CR, reg);

	urtwn_write_1(sc, 0xfe10, 0x19);

	urtwn_delay_ms(sc, 1);

	return 0;
}

static int
urtwn_r92e_power_on(struct urtwn_softc *sc)
{
	uint32_t reg;
	uint32_t val;
	int ntries;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	usbwifi_isowned_ic(&sc->sc_uw);

	/* Enable radio, GPIO and LED functions. */
	KASSERT((R92C_APS_FSMCO_AFSM_HSUS | R92C_APS_FSMCO_PDN_EN |
	    R92C_APS_FSMCO_PFM_ALDN) == 0x0812);
	urtwn_write_2(sc, R92C_APS_FSMCO,
	    R92C_APS_FSMCO_AFSM_HSUS |
	    R92C_APS_FSMCO_PDN_EN |
	    R92C_APS_FSMCO_PFM_ALDN);

	if (urtwn_read_4(sc, R92E_SYS_CFG1_8192E) & R92E_SPSLDO_SEL) {
		/* LDO. */
		urtwn_write_1(sc, R92E_LDO_SWR_CTRL, 0xc3);
	} else {
		urtwn_write_2(sc, R92C_SYS_SWR_CTRL2, urtwn_read_2(sc,
		    R92C_SYS_SWR_CTRL2) & 0xffff);
		urtwn_write_1(sc, R92E_LDO_SWR_CTRL, 0x83);
	}

	for (ntries = 0; ntries < 2; ntries++) {
		urtwn_write_1(sc, R92C_AFE_PLL_CTRL,
		    urtwn_read_1(sc, R92C_AFE_PLL_CTRL));
		urtwn_write_2(sc, R92C_AFE_CTRL4,
		    urtwn_read_2(sc, R92C_AFE_CTRL4));
	}

	/* Reset BB. */
	urtwn_write_1(sc, R92C_SYS_FUNC_EN,
	    urtwn_read_1(sc, R92C_SYS_FUNC_EN) & ~(R92C_SYS_FUNC_EN_BBRSTB |
		R92C_SYS_FUNC_EN_BB_GLB_RST));

	urtwn_write_1(sc, R92C_AFE_XTAL_CTRL + 2, urtwn_read_1(sc,
	    R92C_AFE_XTAL_CTRL + 2) | 0x80);

	/* Disable HWPDN. */
	urtwn_write_2(sc, R92C_APS_FSMCO,
	    urtwn_read_2(sc, R92C_APS_FSMCO) & ~R92C_APS_FSMCO_APDM_HPDN);

	/* Disable WL suspend. */
	urtwn_write_2(sc, R92C_APS_FSMCO,
	    urtwn_read_2(sc, R92C_APS_FSMCO) & ~(R92C_APS_FSMCO_AFSM_PCIE |
		R92C_APS_FSMCO_AFSM_HSUS));

	urtwn_write_4(sc, R92C_APS_FSMCO,
	    urtwn_read_4(sc, R92C_APS_FSMCO) | R92C_APS_FSMCO_RDY_MACON);
	urtwn_write_2(sc, R92C_APS_FSMCO,
	    urtwn_read_2(sc, R92C_APS_FSMCO) | R92C_APS_FSMCO_APFM_ONMAC);
	for (ntries = 0; ntries < 10000; ntries++) {
		val = urtwn_read_2(sc, R92C_APS_FSMCO) &
		    R92C_APS_FSMCO_APFM_ONMAC;
		if (val == 0x0)
			break;
		DELAY(260);
	}
	if (ntries == 10000) {
		aprint_error_dev(sc->sc_uw.uw_dev,
		    "timeout waiting for chip power up\n");
		return ETIMEDOUT;
	}

	urtwn_write_2(sc, R92C_CR, 0x00);
	reg = urtwn_read_2(sc, R92C_CR);
	reg |= R92C_CR_HCI_TXDMA_EN | R92C_CR_HCI_RXDMA_EN |
	    R92C_CR_TXDMA_EN | R92C_CR_RXDMA_EN | R92C_CR_PROTOCOL_EN |
	    R92C_CR_SCHEDULE_EN | R92C_CR_ENSEC;
	urtwn_write_2(sc, R92C_CR, reg);

	urtwn_write_1(sc, 0xfe10, 0x19);

	urtwn_delay_ms(sc, 1);

	return 0;
}

static int
urtwn_r88e_power_on(struct urtwn_softc *sc)
{
	uint32_t reg;
	uint8_t val;
	int ntries;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	usbwifi_isowned_ic(&sc->sc_uw);

	/* Wait for power ready bit. */
	for (ntries = 0; ntries < 5000; ntries++) {
		val = urtwn_read_1(sc, 0x6) & 0x2;
		if (val == 0x2)
			break;
		DELAY(10);
	}
	if (ntries == 5000) {
		aprint_error_dev(sc->sc_uw.uw_dev,
		    "timeout waiting for chip power up\n");
		return ETIMEDOUT;
	}

	/* Reset BB. */
	urtwn_write_1(sc, R92C_SYS_FUNC_EN,
	urtwn_read_1(sc, R92C_SYS_FUNC_EN) & ~(R92C_SYS_FUNC_EN_BBRSTB |
	    R92C_SYS_FUNC_EN_BB_GLB_RST));

	urtwn_write_1(sc, 0x26, urtwn_read_1(sc, 0x26) | 0x80);

	/* Disable HWPDN. */
	urtwn_write_1(sc, 0x5, urtwn_read_1(sc, 0x5) & ~0x80);

	/* Disable WL suspend. */
	urtwn_write_1(sc, 0x5, urtwn_read_1(sc, 0x5) & ~0x18);

	urtwn_write_1(sc, 0x5, urtwn_read_1(sc, 0x5) | 0x1);
	for (ntries = 0; ntries < 5000; ntries++) {
		if (!(urtwn_read_1(sc, 0x5) & 0x1))
			break;
		DELAY(10);
	}
	if (ntries == 5000)
		return ETIMEDOUT;

	/* Enable LDO normal mode. */
	urtwn_write_1(sc, 0x23, urtwn_read_1(sc, 0x23) & ~0x10);

	/* Enable MAC DMA/WMAC/SCHEDULE/SEC blocks. */
	urtwn_write_2(sc, R92C_CR, 0);
	reg = urtwn_read_2(sc, R92C_CR);
	reg |= R92C_CR_HCI_TXDMA_EN | R92C_CR_HCI_RXDMA_EN |
	    R92C_CR_TXDMA_EN | R92C_CR_RXDMA_EN | R92C_CR_PROTOCOL_EN |
	    R92C_CR_SCHEDULE_EN | R92C_CR_ENSEC | R92C_CR_CALTMR_EN;
	urtwn_write_2(sc, R92C_CR, reg);

	return 0;
}

static int
urtwn_r21a_power_on(struct urtwn_softc *sc)
{
	int ntries;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	/* Clear suspend and power down bits.*/
	urtwn_setbits_1_shift(sc, R92C_APS_FSMCO,
	   R92C_APS_FSMCO_AFSM_HSUS | R92C_APS_FSMCO_APDM_HPDN , 0, 1);

	/* Disable GPIO9 as EXT WAKEUP. */
	urtwn_setbits_1(sc, R92C_GPIO_INTM + 2, 0x01, 0);

	/* Enable WL suspend */
	urtwn_setbits_1_shift(sc, R92C_APS_FSMCO,
	    R92C_APS_FSMCO_AFSM_HSUS | R92C_APS_FSMCO_AFSM_PCIE, 0, 1);

	/* Enable LDOA12 MACRO block for all interfaces. */
	urtwn_setbits_1(sc, R92C_LDOA15_CTRL, 0, R92C_LDOA15_CTRL_EN);
	
	/* Disable BT_GPS_SEL pins. */
	urtwn_setbits_1(sc, 0x067, 0x10, 0);

	/* 1 ms delay. */
	DELAY(1000);

	/* Release analog Ips to digital isolation. */
	urtwn_setbits_1(sc, R92C_SYS_ISO_CTRL,
	    R92C_SYS_ISO_CTRL_IP2MAC, 0);

	/* Disable SW LPS and WL suspend. */
	urtwn_setbits_1_shift(sc, R92C_APS_FSMCO,
	    R92C_APS_FSMCO_APFM_RSM |
	    R92C_APS_FSMCO_AFSM_HSUS |
	    R92C_APS_FSMCO_AFSM_PCIE, 0, 1);

	/* Wait for power ready bit. */
	for (ntries = 0; ntries < 5000; ntries++) {
		if (urtwn_read_4(sc, R92C_APS_FSMCO) & R92C_APS_FSMCO_SUS_HOST)
			break;
		DELAY(10);
	}
	if (ntries == 5000) {
		aprint_error_dev(sc->sc_uw.uw_dev,
		    "timeout waiting for chip power up\n");
		return ETIMEDOUT;
	}

	/* Release WLON reset. */
	urtwn_setbits_1_shift(sc, R92C_APS_FSMCO, 0,
	    R92C_APS_FSMCO_RDY_MACON, 2);

	/* Disable HWPDN. */
	urtwn_setbits_1_shift(sc, R92C_APS_FSMCO,
	    R92C_APS_FSMCO_APDM_HPDN, 0, 1);

	/* Disable WL suspend. */
	urtwn_setbits_1_shift(sc, R92C_APS_FSMCO,
	    R92C_APS_FSMCO_AFSM_HSUS | R92C_APS_FSMCO_AFSM_PCIE, 0, 1);

	urtwn_setbits_1_shift(sc, R92C_APS_FSMCO, 0,
	    R92C_APS_FSMCO_APFM_ONMAC, 1);

	for (ntries = 0; ntries < 5000; ntries++) {
		if (!(urtwn_read_2(sc, R92C_APS_FSMCO) &
		    R92C_APS_FSMCO_APFM_ONMAC))
			break;
		DELAY(10);
	}
	if (ntries == 5000)
		return (ETIMEDOUT);

	/* Switch DPDT_SEL_P output from WL BB. */
	urtwn_setbits_1(sc, R92C_LEDCFG3, 0, 0x01);

	/* switch for PAPE_G/PAPE_A from WL BB; switch LNAON from WL BB. */
	urtwn_setbits_1(sc, 0x067, 0, 0x30);

	urtwn_setbits_1(sc, 0x025, 0x40, 0);

	/* Enable falling edge triggering interrupt. */
	urtwn_setbits_1(sc, R92C_GPIO_INTM + 1, 0, 0x02);

	/* Enable GPIO9 interrupt mode. */
	urtwn_setbits_1(sc, 0x063, 0, 0x02);

	/* Enable GPIO9 input mode. */
	urtwn_setbits_1(sc, 0x062, 0x02, 0);

	/* Enable HSISR GPIO interrupt. */
	urtwn_setbits_1(sc, R92C_HSIMR, 0, 0x01);

	/* Enable HSISR GPIO9 interrupt. */
	urtwn_setbits_1(sc, R92C_HSIMR + 2, 0, 0x02);

	/* XTAL trim. */
	urtwn_setbits_1(sc, R92C_AFE_CTRL3 + 2, 0xFF, 0x82);

	urtwn_setbits_1(sc, R92C_AFE_MISC, 0, 0x40);

	/* Enable MAC DMA/WMAC/SCHEDULE/SEC blocks. */
	urtwn_write_2(sc, R92C_CR, 0x0000);
	urtwn_setbits_2(sc, R92C_CR, 0,
	    R92C_CR_HCI_TXDMA_EN | R92C_CR_TXDMA_EN |
	    R92C_CR_HCI_RXDMA_EN | R92C_CR_RXDMA_EN |
	    R92C_CR_PROTOCOL_EN | R92C_CR_SCHEDULE_EN |
	    R92C_CR_ENSEC | R92C_CR_CALTMR_EN);

	if (urtwn_read_4(sc, R92C_SYS_CFG) & R92C_SYS_CFG_TRP_BT_EN)
		urtwn_setbits_1(sc, R92C_LDO_SWR_CTRL, 0, 0x40);

	return (0);
}

static void
urtwn_r21a_power_off(struct urtwn_softc *sc){
	int ntries;
	
	/* Stop Rx. */
	urtwn_write_1(sc, R92C_CR, 0);

	/* Move card to Low Power state. */
	/* Block all Tx queues. */
	urtwn_write_1(sc, R92C_TXPAUSE, R92C_TX_QUEUE_ALL);

	for (ntries = 0; ntries < 10; ntries++) {
		/* Should be zero if no packet is transmitting. */
		if (urtwn_read_4(sc, R88E_SCH_TXCMD) == 0)
			break;

		DELAY(5000);
	}
	if (ntries == 10) {
		device_printf(sc->sc_uw.uw_dev, "%s: failed to block Tx queues\n",
		    __func__);
		return;
	}

	/* CCK and OFDM are disabled, and clock are gated. */
	urtwn_setbits_1(sc, R92C_SYS_FUNC_EN, R92C_SYS_FUNC_EN_BBRSTB, 0);

	DELAY(1);

	urtwn_setbits_1(sc, R92C_SYS_FUNC_EN, R92C_SYS_FUNC_EN_BB_GLB_RST, 0);

	/* Reset MAC TRX. */
	urtwn_write_1(sc, R92C_CR,
	    R92C_CR_HCI_TXDMA_EN | R92C_CR_HCI_RXDMA_EN);

	/* check if removed later. (?) */
	urtwn_setbits_1_shift(sc, R92C_CR, R92C_CR_ENSEC, 0, 1);

	/* Respond TxOK to scheduler */
	urtwn_setbits_1(sc, R92C_DUAL_TSF_RST, 0, R92C_DUAL_TSF_RST_TXOK);

	/* Reset MCU. */
	urtwn_setbits_1_shift(sc, R92C_SYS_FUNC_EN, R92C_SYS_FUNC_EN_CPUEN,
	    0, 1);
	urtwn_write_1(sc, R92C_MCUFWDL, 0);

	/* Move card to Disabled state. */
	/* Turn off RF. */
	urtwn_write_1(sc, R92C_RF_CTRL, 0);

	urtwn_setbits_1(sc, R92C_LEDCFG3, 0x01, 0);

	/* Enable rising edge triggering interrupt. */
	urtwn_setbits_1(sc, R92C_GPIO_INTM + 1, 0x02, 0);

	/* Release WLON reset. */
	urtwn_setbits_1_shift(sc, R92C_APS_FSMCO, 0,
	    R92C_APS_FSMCO_RDY_MACON, 2);

	/* Turn off MAC by HW state machine */
	urtwn_setbits_1_shift(sc, R92C_APS_FSMCO, 0, R92C_APS_FSMCO_APFM_OFF,
	    1);
	for (ntries = 0; ntries < 10; ntries++) {
		/* Wait until it will be disabled. */
		if ((urtwn_read_2(sc, R92C_APS_FSMCO) &
		    R92C_APS_FSMCO_APFM_OFF) == 0)
			break;

		DELAY(5000);
	}
	/* If firmware in ram code, do reset */
	if (urtwn_read_1(sc, R92C_MCUFWDL) & R92C_MCUFWDL_RAM_DL_SEL){
		urtwn_r21a_fw_reset(sc);
	}
	if (ntries == 10) {
		device_printf(sc->sc_uw.uw_dev, "%s: could not turn off MAC\n",
		    __func__);
		return;
	}

	/* Analog Ips to digital isolation. */
	urtwn_setbits_1(sc, R92C_SYS_ISO_CTRL, 0, R92C_SYS_ISO_CTRL_IP2MAC);

	/* Disable LDOA12 MACRO block. */
	urtwn_setbits_1(sc, R92C_LDOA15_CTRL, R92C_LDOA15_CTRL_EN, 0);

	/* Enable WL suspend. */
	urtwn_setbits_1_shift(sc, R92C_APS_FSMCO, R92C_APS_FSMCO_AFSM_PCIE,
	    R92C_APS_FSMCO_AFSM_HSUS, 1);

	/* Enable GPIO9 as EXT WAKEUP. */
	urtwn_setbits_1(sc, R92C_GPIO_INTM + 2, 0, 0x01);	
}

static int __noinline
urtwn_llt_init(struct urtwn_softc *sc)
{
	size_t i, page_count, pktbuf_count;
	uint32_t val;
	int error;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	usbwifi_isowned_ic(&sc->sc_uw);

	if (sc->chip & URTWN_CHIP_88E)
		page_count = R88E_TX_PAGE_COUNT;
	else if (sc->chip & URTWN_CHIP_92EU)
		page_count = R92E_TX_PAGE_COUNT;
	else if (sc->chip & URTWN_CHIP_21A)
		page_count = R21A_TX_PAGE_COUNT;
	else
		page_count = R92C_TX_PAGE_COUNT;

	if (sc->chip & URTWN_CHIP_88E)
		pktbuf_count = R88E_TXPKTBUF_COUNT;
	else if (sc->chip & URTWN_CHIP_92EU)
		pktbuf_count = R88E_TXPKTBUF_COUNT;
	else if (sc->chip & URTWN_CHIP_21A)
		pktbuf_count = R21A_TXPKTBUF_COUNT;
	else
		pktbuf_count = R92C_TXPKTBUF_COUNT;

	if (sc->chip & URTWN_CHIP_92EU) {
		val = urtwn_read_4(sc, R92E_AUTO_LLT) | R92E_AUTO_LLT_EN;
		urtwn_write_4(sc, R92E_AUTO_LLT, val);
		DELAY(100);
		val = urtwn_read_4(sc, R92E_AUTO_LLT);
		if (val & R92E_AUTO_LLT_EN)
			return EIO;
		return 0;
	}

	/* Reserve pages [0; page_count]. */
	for (i = 0; i < page_count; i++) {
		if ((error = urtwn_llt_write(sc, i, i + 1)) != 0)
			return error;
	}
	/* NB: 0xff indicates end-of-list. */
	if ((error = urtwn_llt_write(sc, i, 0xff)) != 0)
		return error;
	/*
	 * Use pages [page_count + 1; pktbuf_count - 1]
	 * as ring buffer.
	 */
	for (++i; i < pktbuf_count - 1; i++) {
		if ((error = urtwn_llt_write(sc, i, i + 1)) != 0)
			return error;
	}
	/* Make the last page point to the beginning of the ring buffer. */
	error = urtwn_llt_write(sc, i, page_count + 1);
	return error;
}

static void
urtwn_fw_reset(struct urtwn_softc *sc)
{
	uint16_t reg;
	int ntries;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	usbwifi_isowned_ic(&sc->sc_uw);

	/* Tell 8051 to reset itself. */
	urtwn_write_1(sc, R92C_HMETFR + 3, 0x20);
	/* Init firmware commands ring. */
	sc->fwcur = 0;

	/* Wait until 8051 resets by itself. */
	for (ntries = 0; ntries < 100; ntries++) {
		reg = urtwn_read_2(sc, R92C_SYS_FUNC_EN);
		if (!(reg & R92C_SYS_FUNC_EN_CPUEN))
			return;
		DELAY(50);
	}
	/* Force 8051 reset. */
	urtwn_write_2(sc, R92C_SYS_FUNC_EN,
	    urtwn_read_2(sc, R92C_SYS_FUNC_EN) & ~R92C_SYS_FUNC_EN_CPUEN);
}

static void
urtwn_r88e_fw_reset(struct urtwn_softc *sc)
{
	uint16_t reg;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	usbwifi_isowned_ic(&sc->sc_uw);

	if (ISSET(sc->chip, URTWN_CHIP_92EU)) {
		reg = urtwn_read_2(sc, R92C_RSV_CTRL) & ~R92E_RSV_MIO_EN;
		urtwn_write_2(sc,R92C_RSV_CTRL, reg);
		DELAY(50);
	}

	reg = urtwn_read_2(sc, R92C_SYS_FUNC_EN);
	urtwn_write_2(sc, R92C_SYS_FUNC_EN, reg & ~R92C_SYS_FUNC_EN_CPUEN);
	DELAY(50);

	urtwn_write_2(sc, R92C_SYS_FUNC_EN, reg | R92C_SYS_FUNC_EN_CPUEN);
	DELAY(50);

	if (ISSET(sc->chip, URTWN_CHIP_92EU)) {
		reg = urtwn_read_2(sc, R92C_RSV_CTRL) | R92E_RSV_MIO_EN;
		urtwn_write_2(sc,R92C_RSV_CTRL, reg);
		DELAY(50);
	}

	/* Init firmware commands ring. */
	sc->fwcur = 0;
}

static void
urtwn_r21a_fw_reset(struct urtwn_softc *sc)
{
	URTWNHIST_FUNC(); URTWNHIST_CALLED();
	/* Reset MCU IO wrapper. */
	urtwn_setbits_1(sc, R92C_RSV_CTRL, 0x02, 0);
	urtwn_setbits_1(sc, R92C_RSV_CTRL + 1, 0x01, 0);

	urtwn_setbits_1_shift(sc, R92C_SYS_FUNC_EN,
	    R92C_SYS_FUNC_EN_CPUEN, 0, 1);

	/* Enable MCU IO wrapper. */
	urtwn_setbits_1(sc, R92C_RSV_CTRL, 0x02, 0);
	urtwn_setbits_1(sc, R92C_RSV_CTRL + 1, 0, 0x01);

	urtwn_setbits_1_shift(sc, R92C_SYS_FUNC_EN,
	    0, R92C_SYS_FUNC_EN_CPUEN, 1);
}

static int
urtwn_fw_loadpage(struct urtwn_softc *sc, int page, uint8_t *buf, int len)
{
	uint32_t reg;
	int off, mlen, error = 0;

	URTWNHIST_FUNC();
	URTWNHIST_CALLARGS("page=%jd, buf=%#jx, len=%jd", page,
	    (uintptr_t)buf, len, 0);

	reg = urtwn_read_4(sc, R92C_MCUFWDL);
	reg = RW(reg, R92C_MCUFWDL_PAGE, page);
	urtwn_write_4(sc, R92C_MCUFWDL, reg);

	off = R92C_FW_START_ADDR;
	while (len > 0) {
		if (len > 196)
			mlen = 196;
		else if (len > 4)
			mlen = 4;
		else
			mlen = 1;
		error = urtwn_write_region(sc, off, buf, mlen);
		if (error != 0)
			break;
		off += mlen;
		buf += mlen;
		len -= mlen;
	}
	return error;
}

static int __noinline
urtwn_load_firmware(struct urtwn_softc *sc)
{
	firmware_handle_t fwh;
	const struct r92c_fw_hdr *hdr;
	const char *name;
	u_char *fw, *ptr;
	size_t len;
	int mlen, ntries, page, error;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	usbwifi_isowned_ic(&sc->sc_uw);

	/* Read firmware image from the filesystem. */
	if (ISSET(sc->chip, URTWN_CHIP_88E))
		name = "rtl8188eufw.bin";
	else if (ISSET(sc->chip, URTWN_CHIP_92EU))
		name = "rtl8192eefw.bin";
	else if (ISSET(sc->chip, URTWN_CHIP_21A))
		name = "rtl8821aufw.bin";
	else if ((sc->chip & (URTWN_CHIP_UMC_A_CUT | URTWN_CHIP_92C)) ==
	    URTWN_CHIP_UMC_A_CUT)
		name = "rtl8192cfwU.bin";
	else
		name = "rtl8192cfw.bin";
	if ((error = firmware_open("if_urtwn", name, &fwh)) != 0) {
		aprint_error_dev(sc->sc_uw.uw_dev,
		    "failed load firmware of file %s (error %d)\n", name,
		    error);
		return error;
	}
	const size_t fwlen = len = firmware_get_size(fwh);
	aprint_debug_dev(sc->sc_uw.uw_dev, "firmware: %s\n", name);
	fw = firmware_malloc(len);
	if (fw == NULL) {
		aprint_error_dev(sc->sc_uw.uw_dev,
		    "failed to allocate firmware memory\n");
		firmware_close(fwh);
		return ENOMEM;
	}
	error = firmware_read(fwh, 0, fw, len);
	firmware_close(fwh);
	if (error != 0) {
		aprint_error_dev(sc->sc_uw.uw_dev,
		    "failed to read firmware (error %d)\n", error);
		firmware_free(fw, fwlen);
		return error;
	}

	len = fwlen;
	ptr = fw;
	hdr = (const struct r92c_fw_hdr *)ptr;
	/* Check if there is a valid FW header and skip it. */
	if ((le16toh(hdr->signature) >> 4) == 0x88c ||
	    (le16toh(hdr->signature) >> 4) == 0x88e ||
	    (le16toh(hdr->signature) >> 4) == 0x92e ||
	    (le16toh(hdr->signature) >> 4) == 0x92c ||
	    (le16toh(hdr->signature) >> 4) == 0x210) {

		sc->fwver = le16toh(hdr->version);
		DPRINTFN(DBG_INIT, "FW V%jd.%jd",
		    le16toh(hdr->version), le16toh(hdr->subversion), 0, 0);
		DPRINTFN(DBG_INIT, "%02jd-%02jd %02jd:%02jd",
		    hdr->month, hdr->date, hdr->hour, hdr->minute);
		ptr += sizeof(*hdr);
		len -= sizeof(*hdr);
	}

	if (urtwn_read_1(sc, R92C_MCUFWDL) & R92C_MCUFWDL_RAM_DL_SEL) {
		/* Reset MCU ready status */
		urtwn_write_1(sc, R92C_MCUFWDL, 0);
		if (ISSET(sc->chip, URTWN_CHIP_88E) ||
		    ISSET(sc->chip, URTWN_CHIP_92EU))
			urtwn_r88e_fw_reset(sc);
		else if ISSET(sc->chip, URTWN_CHIP_21A)
			urtwn_r21a_fw_reset(sc);
		else
			urtwn_fw_reset(sc);
	}
	if (!ISSET(sc->chip, URTWN_CHIP_88E) &&
	    !ISSET(sc->chip, URTWN_CHIP_92EU)&&
	    !ISSET(sc->chip, URTWN_CHIP_21A)) {
		urtwn_write_2(sc, R92C_SYS_FUNC_EN,
		    urtwn_read_2(sc, R92C_SYS_FUNC_EN) |
		    R92C_SYS_FUNC_EN_CPUEN);
	}

	/* download enabled */
	urtwn_setbits_1(sc, R92C_MCUFWDL, 0, R92C_MCUFWDL_EN);

	urtwn_setbits_1_shift(sc, R92C_MCUFWDL, 0x08,
		    0, 2);

	/* Reset the FWDL checksum. */
	urtwn_write_1(sc, R92C_MCUFWDL,
	    urtwn_read_1(sc, R92C_MCUFWDL) | R92C_MCUFWDL_CHKSUM_RPT);

	DELAY(50);
	/* download firmware */
	for (page = 0; len > 0; page++) {
		mlen = MIN(len, R92C_FW_PAGE_SIZE);
		error = urtwn_fw_loadpage(sc, page, ptr, mlen);
		if (error != 0) {
			aprint_error_dev(sc->sc_uw.uw_dev,
			    "could not load firmware page %d\n", page);
			goto fail;
		}
		ptr += mlen;
		len -= mlen;
	}

	/* Wait for checksum report. */
	for (ntries = 0; ntries < 1000; ntries++) {
		if (urtwn_read_4(sc, R92C_MCUFWDL) & R92C_MCUFWDL_CHKSUM_RPT)
			break;
		DELAY(5);
	}
	if (ntries == 1000) {
		aprint_error_dev(sc->sc_uw.uw_dev,
		    "timeout waiting for checksum report\n");
		error = ETIMEDOUT;
		goto fail;
	}

	/* download disable */
	urtwn_setbits_1(sc, R92C_MCUFWDL, R92C_MCUFWDL_EN, 0);
	if (!ISSET(sc->chip, URTWN_CHIP_21A))
		urtwn_write_1(sc, R92C_MCUFWDL + 1, 0);

	/* Wait for firmware readiness. */
	urtwn_setbits_4(sc, R92C_MCUFWDL, R92C_MCUFWDL_WINTINI_RDY,
	    R92C_MCUFWDL_RDY);
	if (ISSET(sc->chip, URTWN_CHIP_88E) ||
	    ISSET(sc->chip, URTWN_CHIP_92EU))
		urtwn_r88e_fw_reset(sc);
	else if ISSET(sc->chip, URTWN_CHIP_21A)
		urtwn_r21a_fw_reset(sc);

	for (ntries = 0; ntries < 6000; ntries++) {
		if (urtwn_read_4(sc, R92C_MCUFWDL) & R92C_MCUFWDL_WINTINI_RDY)
			break;
		DELAY(5);
	}
	if (ntries == 6000) {
		aprint_error_dev(sc->sc_uw.uw_dev,
		    "timeout waiting for firmware readiness\n");
		error = ETIMEDOUT;
		goto fail;
	}

	DPRINTFN(DBG_INIT, "firmware up and running", 0, 0, 0, 0);

 fail:
	firmware_free(fw, fwlen);
	return error;
}

static __inline int
urtwn_check_condition(struct urtwn_softc *sc, const uint8_t cond[])
{
	return sc->sc_check_condition(sc, cond);
}

static int
urtwn_r92c_check_condition(struct urtwn_softc *sc, const uint8_t cond[])
{
	uint8_t mask;
	int i;

	if (cond[0] == 0)
		return (1);

	URTWNHIST_FUNC(); URTWNHIST_CALLED();
	DPRINTFN(DBG_FN,
	    "%s: condition byte 0: %02X; chip %02X, board %02X\n",
	    __func__, cond[0], sc->chip, sc->board_type);

	if (!(sc->chip & URTWN_CHIP_92C)) {
		if (sc->board_type == R92C_BOARD_TYPE_HIGHPA)
			mask = R92C_COND_RTL8188RU;
		else if (sc->board_type == R92C_BOARD_TYPE_MINICARD)
			mask = R92C_COND_RTL8188CE;
		else
			mask = R92C_COND_RTL8188CU;
	} else {
		if (sc->board_type == R92C_BOARD_TYPE_MINICARD)
			mask = R92C_COND_RTL8192CE;
		else
			mask = R92C_COND_RTL8192CU;
	}

	for (i = 0; i < RTWN_MAX_CONDITIONS && cond[i] != 0; i++)
		if ((cond[i] & mask) == mask)
			return (1);

	return (0);
}

static int
urtwn_r21a_check_condition(struct urtwn_softc *sc, const uint8_t cond[])
{
	struct urtwn_r21a_data *r21a_data = sc->sc_chip_priv.data;
	uint8_t mask;

	int i;
	URTWNHIST_FUNC(); URTWNHIST_CALLED();
	DPRINTFN(DBG_FN,
	    "%s: condition byte 0: %02X; ext 5ghz pa/lna %d/%d\n",
	    __func__, cond[0], r21a_data->ext_pa_5g, r21a_data->ext_lna_5g);

	if (cond[0] == 0)
		return (1);

	mask = 0;
	if (r21a_data->ext_pa_5g)
		mask |= R21A_COND_EXT_PA_5G;
	if (r21a_data->ext_lna_5g)
		mask |= R21A_COND_EXT_LNA_5G;
	if (r21a_data->bt_coex)
		mask |= R21A_COND_BT;
	if (!r21a_data->ext_pa_2g && !r21a_data->ext_lna_2g &&
	    !r21a_data->ext_pa_5g && !r21a_data->ext_lna_5g && !r21a_data->bt_coex)
		mask = R21A_COND_BOARD_DEF;

	if (mask == 0)
		return (0);

	for (i = 0; i < RTWN_MAX_CONDITIONS && cond[i] != 0; i++)
		if (cond[i] == mask)
			return (1);

	return (0);
}

static void
urtwn_r21au_arfb_init(struct urtwn_softc *sc)
{
	/* ARFB table 9 for 11ac 5G 2SS. */
	urtwn_write_4(sc, R21A_ARFR_5G(0), 0x00000010);
	urtwn_write_4(sc, R21A_ARFR_5G(0) + 4, 0xfffff000);

	/* ARFB table 10 for 11ac 5G 1SS. */
	urtwn_write_4(sc, R21A_ARFR_5G(1), 0x00000010);
	urtwn_write_4(sc, R21A_ARFR_5G(1) + 4, 0x003ff000);

	/* ARFB table 11 for 11ac 2G 1SS. */
	urtwn_write_4(sc, R21A_ARFR_2G(0), 0x00000015);
	urtwn_write_4(sc, R21A_ARFR_2G(0) + 4, 0x003ff000);

	/* ARFB table 12 for 11ac 2G 2SS. */
	urtwn_write_4(sc, R21A_ARFR_2G(1), 0x00000015);
	urtwn_write_4(sc, R21A_ARFR_2G(1) + 4, 0xffcff000);
}

static void
urtwn_r21au_init_ampdu(struct urtwn_softc *sc){

	// const uint8_t dma_count = R21A_DMA_MODE | SM(R21A_BURST_CNT, 3);

	/* Rx interval (USB3). */
	urtwn_write_1(sc, 0xf050, 0x01);

	/* burst length = 4 */
	urtwn_write_2(sc, R92C_RXDMA_STATUS, 0x7400);

	urtwn_write_1(sc, R92C_RXDMA_STATUS + 1, 0xf5);

	/* Setup AMPDU aggregation. */
	urtwn_write_1(sc, R21A_AMPDU_MAX_TIME, 0x5e);
	urtwn_write_4(sc, R21A_AMPDU_MAX_LENGTH, 0xffffffff);

	/* 80 MHz clock (again?) */
	urtwn_write_1(sc, R92C_USTIME_TSF, 0x50);
	urtwn_write_1(sc, R92C_USTIME_EDCA, 0x50);

	if ((urtwn_read_1(sc, R92C_USB_INFO) & 0x30) == 0) {
		/* Set burst packet length to 512 B. */
		// urtwn_setbits_1(sc, R21A_RXDMA_PRO, R21A_BURST_SZ_M,
		//     dma_count | SM(R21A_BURST_SZ, R21A_BURST_SZ_USB2));
		urtwn_setbits_1(sc, R21A_RXDMA_PRO, 0x20, 0x1e);
	} else {
		/* Set burst packet length to 64 B. */
		// urtwn_setbits_1(sc, R21A_RXDMA_PRO, R21A_BURST_SZ_M,
		//     dma_count | SM(R21A_BURST_SZ, R21A_BURST_SZ_USB1));
		urtwn_setbits_1(sc, R21A_RXDMA_PRO, 0x10, 0x2e);
	}
	
	/* Enable single packet AMPDU. */
	urtwn_setbits_1(sc, R21A_HT_SINGLE_AMPDU, 0,
	    R21A_HT_SINGLE_AMPDU_PKT_ENA);

	/* 11K packet length for VHT. */
	urtwn_write_1(sc, R92C_RX_PKT_LIMIT, 0x18);

	urtwn_write_1(sc, R92C_PIFS, 0);

	urtwn_write_2(sc, R92C_MAX_AGGR_NUM, 0x1f1f);

	urtwn_write_1(sc, R92C_FWHW_TXQ_CTRL,
	    R92C_FWHW_TXQ_CTRL_AMPDU_RTY_NEW);
	urtwn_write_4(sc, R92C_FAST_EDCA_CTRL, 0x03087777);

	/* Do not reset MAC. */
	urtwn_setbits_1(sc, R92C_RSV_CTRL, 0, 0x60);

	urtwn_r21au_arfb_init(sc);
}

static __inline int
urtwn_dma_init(struct urtwn_softc *sc)
{

	return sc->sc_dma_init(sc);
}

static int
urtwn_r92c_dma_init(struct urtwn_softc *sc)
{
	int hashq, hasnq, haslq, nqueues, nqpages, nrempages;
	uint32_t reg;
	int error;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	usbwifi_isowned_ic(&sc->sc_uw);

	/* Initialize LLT table. */
	error = urtwn_llt_init(sc);
	if (error != 0)
		return error;

	/* Get Tx queues to USB endpoints mapping. */
	hashq = hasnq = haslq = 0;
	reg = urtwn_read_2(sc, R92C_USB_EP + 1);
	DPRINTFN(DBG_INIT, "USB endpoints mapping %#jx", reg, 0, 0, 0);
	if (MS(reg, R92C_USB_EP_HQ) != 0)
		hashq = 1;
	if (MS(reg, R92C_USB_EP_NQ) != 0)
		hasnq = 1;
	if (MS(reg, R92C_USB_EP_LQ) != 0)
		haslq = 1;
	nqueues = hashq + hasnq + haslq;
	if (nqueues == 0)
		return EIO;
	/* Get the number of pages for each queue. */
	nqpages = (R92C_TX_PAGE_COUNT - R92C_PUBQ_NPAGES) / nqueues;
	/* The remaining pages are assigned to the high priority queue. */
	nrempages = (R92C_TX_PAGE_COUNT - R92C_PUBQ_NPAGES) % nqueues;

	/* Set number of pages for normal priority queue. */
	urtwn_write_1(sc, R92C_RQPN_NPQ, hasnq ? nqpages : 0);
	urtwn_write_4(sc, R92C_RQPN,
	    /* Set number of pages for public queue. */
	    SM(R92C_RQPN_PUBQ, R92C_PUBQ_NPAGES) |
	    /* Set number of pages for high priority queue. */
	    SM(R92C_RQPN_HPQ, hashq ? nqpages + nrempages : 0) |
	    /* Set number of pages for low priority queue. */
	    SM(R92C_RQPN_LPQ, haslq ? nqpages : 0) |
	    /* Load values. */
	    R92C_RQPN_LD);

	urtwn_write_1(sc, R92C_TXPKTBUF_BCNQ_BDNY, R92C_TX_PAGE_BOUNDARY);
	urtwn_write_1(sc, R92C_TXPKTBUF_MGQ_BDNY, R92C_TX_PAGE_BOUNDARY);
	urtwn_write_1(sc, R92C_TXPKTBUF_WMAC_LBK_BF_HD, R92C_TX_PAGE_BOUNDARY);
	urtwn_write_1(sc, R92C_TRXFF_BNDY, R92C_TX_PAGE_BOUNDARY);
	urtwn_write_1(sc, R92C_TDECTRL + 1, R92C_TX_PAGE_BOUNDARY);

	/* Set queue to USB pipe mapping. */
	reg = urtwn_read_2(sc, R92C_TRXDMA_CTRL);
	reg &= ~R92C_TRXDMA_CTRL_QMAP_M;
	if (nqueues == 1) {
		if (hashq) {
			reg |= R92C_TRXDMA_CTRL_QMAP_HQ;
		} else if (hasnq) {
			reg |= R92C_TRXDMA_CTRL_QMAP_NQ;
		} else {
			reg |= R92C_TRXDMA_CTRL_QMAP_LQ;
		}
	} else if (nqueues == 2) {
		/* All 2-endpoints configs have a high priority queue. */
		if (!hashq) {
			return EIO;
		}
		if (hasnq) {
			reg |= R92C_TRXDMA_CTRL_QMAP_HQ_NQ;
		} else {
			reg |= R92C_TRXDMA_CTRL_QMAP_HQ_LQ;
		}
	} else {
		reg |= R92C_TRXDMA_CTRL_QMAP_3EP;
	}
	urtwn_write_2(sc, R92C_TRXDMA_CTRL, reg);

	/* Set Tx/Rx transfer page boundary. */
	urtwn_write_2(sc, R92C_TRXFF_BNDY + 2, 0x27ff);

	/* Set Tx/Rx transfer page size. */
	urtwn_write_1(sc, R92C_PBP,
	    SM(R92C_PBP_PSRX, R92C_PBP_128) | SM(R92C_PBP_PSTX, R92C_PBP_128));
	return 0;
}

static int
urtwn_r88e_dma_init(struct urtwn_softc *sc)
{
	usb_interface_descriptor_t *id;
	uint32_t reg;
	int nqueues;
	int error;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	usbwifi_isowned_ic(&sc->sc_uw);

	/* Initialize LLT table. */
	error = urtwn_llt_init(sc);
	if (error != 0)
		return error;

	/* Get Tx queues to USB endpoints mapping. */
	id = usbd_get_interface_descriptor(sc->sc_uw.uw_iface);
	nqueues = id->bNumEndpoints - 1;
	if (nqueues == 0)
		return EIO;

	/* Set number of pages for normal priority queue. */
	urtwn_write_2(sc, R92C_RQPN_NPQ, 0);
	urtwn_write_2(sc, R92C_RQPN_NPQ, 0x000d);
	urtwn_write_4(sc, R92C_RQPN, 0x808e000d);

	urtwn_write_1(sc, R92C_TXPKTBUF_BCNQ_BDNY, R88E_TX_PAGE_BOUNDARY);
	urtwn_write_1(sc, R92C_TXPKTBUF_MGQ_BDNY, R88E_TX_PAGE_BOUNDARY);
	urtwn_write_1(sc, R92C_TXPKTBUF_WMAC_LBK_BF_HD, R88E_TX_PAGE_BOUNDARY);
	urtwn_write_1(sc, R92C_TRXFF_BNDY, R88E_TX_PAGE_BOUNDARY);
	urtwn_write_1(sc, R92C_TDECTRL + 1, R88E_TX_PAGE_BOUNDARY);

	/* Set queue to USB pipe mapping. */
	reg = urtwn_read_2(sc, R92C_TRXDMA_CTRL);
	reg &= ~R92C_TRXDMA_CTRL_QMAP_M;
	if (nqueues == 1)
		reg |= R92C_TRXDMA_CTRL_QMAP_LQ;
	else if (nqueues == 2)
		reg |= R92C_TRXDMA_CTRL_QMAP_HQ_NQ;
	else
		reg |= R92C_TRXDMA_CTRL_QMAP_3EP;
	urtwn_write_2(sc, R92C_TRXDMA_CTRL, reg);

	/* Set Tx/Rx transfer page boundary. */
	urtwn_write_2(sc, R92C_TRXFF_BNDY + 2, 0x23ff);

	/* Set Tx/Rx transfer page size. */
	urtwn_write_1(sc, R92C_PBP,
	    SM(R92C_PBP_PSRX, R92C_PBP_128) | SM(R92C_PBP_PSTX, R92C_PBP_128));

	return 0;
}

static int
urtwn_r21a_dma_init(struct urtwn_softc *sc)
{
	int hasnq, haslq, nqueues, nqpages, nrempages;
	uint32_t reg;
	int error;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	usbwifi_isowned_ic(&sc->sc_uw);

	/* Initialize LLT table. */
	error = urtwn_llt_init(sc);
	if (error != 0)
		return error;

	hasnq = haslq = 0;
	switch (sc->ntx) {
	case 4:
	case 3:
		haslq = 1;
		/* FALLTHROUGH */
	case 2:
		hasnq = 1;
		/* FALLTHROUGH */
	default:
		break;
	}
	nqueues = 1 + hasnq + haslq;

	/* Get the number of pages for each queue. */
	nqpages = (R21A_TX_PAGE_COUNT - R21A_PUBQ_NPAGES) / nqueues;

	/*
	 * The remaining pages are assigned to the high priority
	 * queue.
	 */
	nrempages = (R21A_TX_PAGE_COUNT - R21A_PUBQ_NPAGES) % nqueues;

	urtwn_write_1(sc, R92C_RQPN_NPQ, hasnq ? nqpages : 0);
	urtwn_write_4(sc, R92C_RQPN,
	    /* Set number of pages for public queue. */
	    SM(R92C_RQPN_PUBQ, R21A_PUBQ_NPAGES) |
	    /* Set number of pages for high priority queue. */
	    SM(R92C_RQPN_HPQ, nqpages + nrempages) |
	    /* Set number of pages for low priority queue. */
	    SM(R92C_RQPN_LPQ, haslq ? nqpages : 0) |
	    /* Load values. */
	    R92C_RQPN_LD);

	/* Initialize TX buffer boundary. */
	urtwn_write_1(sc, R92C_TXPKTBUF_BCNQ_BDNY, R21A_TX_PAGE_COUNT + 1);
	urtwn_write_1(sc, R92C_TXPKTBUF_MGQ_BDNY, R21A_TX_PAGE_COUNT + 1);
	urtwn_write_1(sc, R92C_TXPKTBUF_WMAC_LBK_BF_HD, R21A_TX_PAGE_COUNT + 1);
	urtwn_write_1(sc, R92C_TRXFF_BNDY, R21A_TX_PAGE_COUNT + 1);
	urtwn_write_1(sc, R92C_TDECTRL + 1, R21A_TX_PAGE_COUNT + 1);

	urtwn_write_1(sc, R88E_TXPKTBUF_BCNQ1_BDNY,
	    R21A_BCNQ0_BOUNDARY);
	urtwn_write_1(sc, R21A_DWBCN1_CTRL + 1,
	    R21A_BCNQ0_BOUNDARY);
	urtwn_setbits_1(sc, R21A_DWBCN1_CTRL + 2, 0, 0x2);

	switch (nqueues) {
	case 1:
		reg = R92C_TRXDMA_CTRL_QMAP_HQ;
		break;
	case 2:
		reg = R92C_TRXDMA_CTRL_QMAP_HQ_NQ;
		break;
	default:
		reg = R92C_TRXDMA_CTRL_QMAP_3EP;
		break;
	}
	/* Set queue to USB pipe mapping. */
	urtwn_setbits_2(sc, R92C_TRXDMA_CTRL, R92C_TRXDMA_CTRL_QMAP_M, reg);

	/* Set Tx/Rx transfer page boundary. */
	urtwn_write_2(sc, R92C_TRXFF_BNDY + 2, 0x3e7f);
	return 0;
}

static void __noinline
urtwn_mac_init(struct urtwn_softc *sc)
{
	size_t i;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	usbwifi_isowned_ic(&sc->sc_uw);

	/* Write MAC initialization values. */
	if (ISSET(sc->chip, URTWN_CHIP_88E)) {
		for (i = 0; i < __arraycount(rtl8188eu_mac); i++)
			urtwn_write_1(sc, rtl8188eu_mac[i].reg,
			    rtl8188eu_mac[i].val);
	} else if (ISSET(sc->chip, URTWN_CHIP_92EU)) {
		for (i = 0; i < __arraycount(rtl8192eu_mac); i++)
			urtwn_write_1(sc, rtl8192eu_mac[i].reg,
			    rtl8192eu_mac[i].val);
	} else if (ISSET(sc->chip, URTWN_CHIP_21A)) {
		for (i = 0; i < __arraycount(rtl8821au_mac); i++)
			urtwn_write_1(sc, rtl8821au_mac[i].reg,
			    rtl8821au_mac[i].val);
	} else {
		for (i = 0; i < __arraycount(rtl8192cu_mac); i++)
			urtwn_write_1(sc, rtl8192cu_mac[i].reg,
			    rtl8192cu_mac[i].val);
	}
}

static void __noinline
urtwn_mrr_init(struct urtwn_softc *sc)
{
	int i;

	/* Drop rate index by 1 per retry. */
	for (i = 0; i < R92C_DARFRC_SIZE; i++) {
		urtwn_write_1(sc, R92C_DARFRC + i, i + 1);
		// urtwn_write_1(sc, R92C_RARFRC + i, i + 1);
	}
}


static void __noinline
urtwn_bb_init(struct urtwn_softc *sc)
{
	const struct rtwn_bb_prog *prog;
	uint32_t reg;
	uint8_t crystalcap;
	size_t i;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	usbwifi_isowned_ic(&sc->sc_uw);

	/* Enable BB and RF. */
	urtwn_write_2(sc, R92C_SYS_FUNC_EN,
	    urtwn_read_2(sc, R92C_SYS_FUNC_EN) |
	    R92C_SYS_FUNC_EN_BBRSTB | R92C_SYS_FUNC_EN_BB_GLB_RST |
	    R92C_SYS_FUNC_EN_DIO_RF);

	if (!ISSET(sc->chip, URTWN_CHIP_88E) &&
	    !ISSET(sc->chip, URTWN_CHIP_92EU) &&
	    !ISSET(sc->chip, URTWN_CHIP_21A)) {
		urtwn_write_1(sc, R92C_AFE_PLL_CTRL, 0x83);
		urtwn_write_1(sc, R92C_AFE_PLL_CTRL + 1, 0xdb);
	}

	urtwn_write_1(sc, R92C_RF_CTRL,
	    R92C_RF_CTRL_EN | R92C_RF_CTRL_RSTB | R92C_RF_CTRL_SDMRSTB);
	urtwn_write_1(sc, R92C_SYS_FUNC_EN,
	    R92C_SYS_FUNC_EN_USBA | R92C_SYS_FUNC_EN_USBD |
	    R92C_SYS_FUNC_EN_BB_GLB_RST | R92C_SYS_FUNC_EN_BBRSTB);

	if (ISSET(sc->chip, URTWN_CHIP_21A)) {
		/* PathB RF Power On. */
		urtwn_write_1(sc, R21A_RF_B_CTRL,
		    R92C_RF_CTRL_EN | R92C_RF_CTRL_RSTB | R92C_RF_CTRL_SDMRSTB);
	}

	if (!ISSET(sc->chip, URTWN_CHIP_88E) &&
	    !ISSET(sc->chip, URTWN_CHIP_92EU) &&
	    !ISSET(sc->chip, URTWN_CHIP_21A)) {
		urtwn_write_1(sc, R92C_LDOHCI12_CTRL, 0x0f);
		urtwn_write_1(sc, 0x15, 0xe9);
		urtwn_write_1(sc, R92C_AFE_XTAL_CTRL + 1, 0x80);
	}

	/* Select BB programming based on board type. */
	if (ISSET(sc->chip, URTWN_CHIP_88E))
		prog = &rtl8188eu_bb_prog;
	else if (ISSET(sc->chip, URTWN_CHIP_92EU))
		prog = &rtl8192eu_bb_prog;
	else if (!(sc->chip & URTWN_CHIP_92C)) {
		if (sc->board_type == R92C_BOARD_TYPE_MINICARD) {
			prog = &rtl8188ce_bb_prog;
		} else if (sc->board_type == R92C_BOARD_TYPE_HIGHPA) {
			prog = &rtl8188ru_bb_prog;
		} else {
			prog = &rtl8188cu_bb_prog;
		}
	} else {
		if (sc->board_type == R92C_BOARD_TYPE_MINICARD) {
			prog = &rtl8192ce_bb_prog;
		} else {
			prog = &rtl8192cu_bb_prog;
		}
	}
	/* Write BB initialization values. */
	for (i = 0; i < prog->count; i++) {
		/* additional delay depend on registers */
		switch (prog->regs[i]) {
		case 0xfe:
			urtwn_delay_ms(sc, 50);
			break;
		case 0xfd:
			urtwn_delay_ms(sc, 5);
			break;
		case 0xfc:
			urtwn_delay_ms(sc, 1);
			break;
		case 0xfb:
			DELAY(50);
			break;
		case 0xfa:
			DELAY(5);
			break;
		case 0xf9:
			DELAY(1);
			break;
		}
		urtwn_bb_write(sc, prog->regs[i], prog->vals[i]);
		DELAY(1);
	}

	if (sc->chip & URTWN_CHIP_92C_1T2R) {
		/* 8192C 1T only configuration. */
		reg = urtwn_bb_read(sc, R92C_FPGA0_TXINFO);
		reg = (reg & ~0x00000003) | 0x2;
		urtwn_bb_write(sc, R92C_FPGA0_TXINFO, reg);

		reg = urtwn_bb_read(sc, R92C_FPGA1_TXINFO);
		reg = (reg & ~0x00300033) | 0x00200022;
		urtwn_bb_write(sc, R92C_FPGA1_TXINFO, reg);

		reg = urtwn_bb_read(sc, R92C_CCK0_AFESETTING);
		reg = (reg & ~0xff000000) | (0x45 << 24);
		urtwn_bb_write(sc, R92C_CCK0_AFESETTING, reg);

		reg = urtwn_bb_read(sc, R92C_OFDM0_TRXPATHENA);
		reg = (reg & ~0x000000ff) | 0x23;
		urtwn_bb_write(sc, R92C_OFDM0_TRXPATHENA, reg);

		reg = urtwn_bb_read(sc, R92C_OFDM0_AGCPARAM1);
		reg = (reg & ~0x00000030) | (1 << 4);
		urtwn_bb_write(sc, R92C_OFDM0_AGCPARAM1, reg);

		reg = urtwn_bb_read(sc, 0xe74);
		reg = (reg & ~0x0c000000) | (2 << 26);
		urtwn_bb_write(sc, 0xe74, reg);
		reg = urtwn_bb_read(sc, 0xe78);
		reg = (reg & ~0x0c000000) | (2 << 26);
		urtwn_bb_write(sc, 0xe78, reg);
		reg = urtwn_bb_read(sc, 0xe7c);
		reg = (reg & ~0x0c000000) | (2 << 26);
		urtwn_bb_write(sc, 0xe7c, reg);
		reg = urtwn_bb_read(sc, 0xe80);
		reg = (reg & ~0x0c000000) | (2 << 26);
		urtwn_bb_write(sc, 0xe80, reg);
		reg = urtwn_bb_read(sc, 0xe88);
		reg = (reg & ~0x0c000000) | (2 << 26);
		urtwn_bb_write(sc, 0xe88, reg);
	}

	/* Write AGC values. */
	for (i = 0; i < prog->agccount; i++) {
		if (ISSET(sc->chip, URTWN_CHIP_21A))
			urtwn_bb_write(sc, 0x81c, prog->agcvals[i]);
		else
			urtwn_bb_write(sc, R92C_OFDM0_AGCRSSITABLE, prog->agcvals[i]);
		DELAY(1);
	}

	if (ISSET(sc->chip, URTWN_CHIP_88E) ||
	    ISSET(sc->chip, URTWN_CHIP_92EU)) {
		urtwn_bb_write(sc, R92C_OFDM0_AGCCORE1(0), 0x69553422);
		DELAY(1);
		urtwn_bb_write(sc, R92C_OFDM0_AGCCORE1(0), 0x69553420);
		DELAY(1);
	} else if (ISSET(sc->chip, URTWN_CHIP_21A)){
		for (i = 0; i < sc->nrxchains; i++) {
			urtwn_bb_write(sc, R21A_INITIAL_GAIN(i), 0x22);
			DELAY(1);
			urtwn_bb_write(sc, R21A_INITIAL_GAIN(i), 0x20);
			DELAY(1);
		}
	}

	if (ISSET(sc->chip, URTWN_CHIP_92EU)) {
		crystalcap = sc->r88e_rom[0xb9];
		if (crystalcap == 0x00)
			crystalcap = 0x20;
		crystalcap &= 0x3f;
		reg = urtwn_bb_read(sc, R92C_AFE_CTRL3);
		urtwn_bb_write(sc, R92C_AFE_CTRL3,
		    RW(reg, R92C_AFE_XTAL_CTRL_ADDR,
		    crystalcap | crystalcap << 6));
		urtwn_write_4(sc, R92C_AFE_XTAL_CTRL, 0xf81fb);
	} else if (ISSET(sc->chip, URTWN_CHIP_88E)) {
		crystalcap = sc->r88e_rom[0xb9];
		if (crystalcap == 0xff)
			crystalcap = 0x20;
		crystalcap &= 0x3f;
		reg = urtwn_bb_read(sc, R92C_AFE_XTAL_CTRL);
		urtwn_bb_write(sc, R92C_AFE_XTAL_CTRL,
		    RW(reg, R92C_AFE_XTAL_CTRL_ADDR,
		    crystalcap | crystalcap << 6));
	} else if (ISSET(sc->chip, URTWN_CHIP_21A)){
		crystalcap = sc->sc_chip_priv.data->r21a_rom.crystalcap & 0x3f;
		reg = urtwn_bb_read(sc, R92C_AFE_CTRL3);
		reg = RW(reg, R21A_MAC_PHY_CRYSTALCAP, crystalcap | (crystalcap << 6));
		urtwn_bb_write(sc, R92C_AFE_CTRL3, reg);
	} else {
		if (urtwn_bb_read(sc, R92C_HSSI_PARAM2(0)) &
		    R92C_HSSI_PARAM2_CCK_HIPWR) {
			SET(sc->sc_uw.uw_flags, URTWN_FLAG_CCK_HIPWR);
		}
	}
}

static void __noinline
urtwn_r21a_bb_init(struct urtwn_softc *sc)
{
	const struct rtwn_bb_prog *prog;
	uint32_t reg;
	uint8_t crystalcap;
	size_t i;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	usbwifi_isowned_ic(&sc->sc_uw);

	urtwn_setbits_1(sc, R92C_SYS_FUNC_EN, 0, R92C_SYS_FUNC_EN_USBA);

	/* Enable BB and RF. */
	urtwn_setbits_1(sc, R92C_SYS_FUNC_EN, 0,
	    R92C_SYS_FUNC_EN_BBRSTB | R92C_SYS_FUNC_EN_BB_GLB_RST);

	/* PathA RF Power On. */
	urtwn_write_1(sc, R92C_RF_CTRL,
	    R92C_RF_CTRL_EN | R92C_RF_CTRL_RSTB | R92C_RF_CTRL_SDMRSTB);

	/* PathB RF Power On. */
	urtwn_write_1(sc, R21A_RF_B_CTRL,
	    R92C_RF_CTRL_EN | R92C_RF_CTRL_RSTB | R92C_RF_CTRL_SDMRSTB);

	prog = &rtl8821au_bb_prog[0];
	uint8_t cond[] = { R21A_COND_EXT_PA_5G | R21A_COND_EXT_LNA_5G, 0 };
	if (!urtwn_r21a_check_condition(sc, cond)){
		prog = &rtl8821au_bb_prog[1];
	}
	
	/* Write BB initialization values. */
	for (i = 0; i < prog->count; i++) {
		urtwn_bb_write(sc, prog->regs[i], prog->vals[i]);
		DELAY(1);
	}

	/* Write AGC values. */
	for (i = 0; i < prog->agccount; i++) {
		urtwn_bb_write(sc, 0x81c, prog->agcvals[i]);
		DELAY(1);
	}

	for (i = 0; i < sc->nrxchains; i++) {
		urtwn_bb_write(sc, R21A_INITIAL_GAIN(i), 0x22);
		DELAY(1);
		urtwn_bb_write(sc, R21A_INITIAL_GAIN(i), 0x20);
		DELAY(1);
	}

	crystalcap = sc->sc_chip_priv.data->r21a_rom.crystalcap & 0x3f;
	reg = urtwn_bb_read(sc, R92C_AFE_CTRL3);
	reg = RW(reg, R21A_MAC_PHY_CRYSTALCAP, crystalcap | (crystalcap << 6));
	urtwn_bb_write(sc, R92C_AFE_CTRL3, reg);
}

static void __noinline
urtwn_rf_init(struct urtwn_softc *sc)
{
	const struct rtwn_rf_prog *prog;
	uint32_t reg, mask, saved;
	size_t i, j, idx;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	/* Select RF programming based on board type. */
	if (ISSET(sc->chip, URTWN_CHIP_88E))
		prog = rtl8188eu_rf_prog;
	else if (ISSET(sc->chip, URTWN_CHIP_92EU))
		prog = rtl8192eu_rf_prog;
	else if (ISSET(sc->chip, URTWN_CHIP_21A)) 
		prog = rtl8821au_rf_prog;
	else if (!(sc->chip & URTWN_CHIP_92C)) {
		if (sc->board_type == R92C_BOARD_TYPE_MINICARD) {
			prog = rtl8188ce_rf_prog;
		} else if (sc->board_type == R92C_BOARD_TYPE_HIGHPA) {
			prog = rtl8188ru_rf_prog;
		} else {
			prog = rtl8188cu_rf_prog;
		}
	} else {
		prog = rtl8192ce_rf_prog;
	}

	for (i = 0; i < sc->nrxchains; i++) {
		/* Save RF_ENV control type. */
		idx = i / 2;
		mask = 0xffffU << ((i % 2) * 16);
		saved = urtwn_bb_read(sc, R92C_FPGA0_RFIFACESW(idx)) & mask;

		/* Set RF_ENV enable. */
		reg = urtwn_bb_read(sc, R92C_FPGA0_RFIFACEOE(i));
		reg |= 0x100000;
		urtwn_bb_write(sc, R92C_FPGA0_RFIFACEOE(i), reg);
		DELAY(50);

		/* Set RF_ENV output high. */
		reg = urtwn_bb_read(sc, R92C_FPGA0_RFIFACEOE(i));
		reg |= 0x10;
		urtwn_bb_write(sc, R92C_FPGA0_RFIFACEOE(i), reg);
		DELAY(50);

		/* Set address and data lengths of RF registers. */
		reg = urtwn_bb_read(sc, R92C_HSSI_PARAM2(i));
		reg &= ~R92C_HSSI_PARAM2_ADDR_LENGTH;
		urtwn_bb_write(sc, R92C_HSSI_PARAM2(i), reg);
		DELAY(50);
		reg = urtwn_bb_read(sc, R92C_HSSI_PARAM2(i));
		reg &= ~R92C_HSSI_PARAM2_DATA_LENGTH;
		urtwn_bb_write(sc, R92C_HSSI_PARAM2(i), reg);
		DELAY(50);

		/* Write RF initialization values for this chain. */
		for (j = 0; j < prog[i].count; j++) {
			if (prog[i].regs[j] >= 0xf9 &&
			    prog[i].regs[j] <= 0xfe) {
				/*
				 * These are fake RF registers offsets that
				 * indicate a delay is required.
				 */
				urtwn_delay_ms(sc, 50);
				continue;
			}
			urtwn_rf_write(sc, i, prog[i].regs[j], prog[i].vals[j]);
			DELAY(5);
		}

		/* Restore RF_ENV control type. */
		reg = urtwn_bb_read(sc, R92C_FPGA0_RFIFACESW(idx)) & ~mask;
		urtwn_bb_write(sc, R92C_FPGA0_RFIFACESW(idx), reg | saved);
	}

	if ((sc->chip & (URTWN_CHIP_UMC_A_CUT | URTWN_CHIP_92C)) ==
	    URTWN_CHIP_UMC_A_CUT) {
		urtwn_rf_write(sc, 0, R92C_RF_RX_G1, 0x30255);
		urtwn_rf_write(sc, 0, R92C_RF_RX_G2, 0x50a00);
	}

	/* Cache RF register CHNLBW. */
	for (i = 0; i < 2; i++) {
		sc->rf_chnlbw[i] = urtwn_rf_read(sc, i, R92C_RF_CHNLBW);
	}
}

static int __noinline
urtwn_rf_init_chain(struct urtwn_softc *sc,
	const struct rtwn_rf_prog *prog, int chain)
{
	int i,j;
	for (i = 0; prog[i].regs != NULL; i++) {
		const struct rtwn_rf_prog *rf_prog = &prog[i];

		while (!urtwn_check_condition(sc, rf_prog->cond)) {
			KASSERT(rf_prog->next != NULL);
			rf_prog = rf_prog->next;
		}
		for (j = 0; j < rf_prog->count; j++) {
			if (rf_prog->regs[j] > 0xf8) {
				if (rf_prog->vals[j] < 1000)
					DELAY(rf_prog->vals[j]);
				else
					urtwn_delay_ms(sc, rf_prog->vals[j]/1000 + 1);
				continue;
			}
			urtwn_rf_write(sc, chain, rf_prog->regs[j], rf_prog->vals[j]);
			DELAY(1);
		}
	}
	return i;
}
static void __noinline
urtwn_r21a_rf_init(struct urtwn_softc *sc)
{
	int i, chain;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	for (chain = 0, i = 0; chain < sc->nrxchains; chain++, i++) {
		i += urtwn_rf_init_chain(sc, &rtl8821au_rf_prog[i], chain);
	}
}

static void __noinline
urtwn_cam_init(struct urtwn_softc *sc)
{
	uint32_t content, command;
	uint8_t idx;
	size_t i;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	usbwifi_isowned_ic(&sc->sc_uw);
	if (ISSET(sc->chip, URTWN_CHIP_92EU))
		return;

	for (idx = 0; idx < R21A_CAM_ENTRY_COUNT; idx++) {
		content = (idx & 3)
		    | (R92C_CAM_ALGO_AES << R92C_CAM_ALGO_S)
		    | R92C_CAM_VALID;

		command = R92C_CAMCMD_POLLING
		    | R92C_CAMCMD_WRITE
		    | R92C_CAM_CTL0(idx);

		urtwn_write_4(sc, R92C_CAMWRITE, content);
		urtwn_write_4(sc, R92C_CAMCMD, command);
	}

	for (idx = 0; idx < R21A_CAM_ENTRY_COUNT; idx++) {
		for (i = 0; i < /* CAM_CONTENT_COUNT */ 8; i++) {
			if (i == 0) {
				content = (idx & 3)
				    | (R92C_CAM_ALGO_AES << R92C_CAM_ALGO_S)
				    | R92C_CAM_VALID;
			} else {
				content = 0;
			}

			command = R92C_CAMCMD_POLLING
			    | R92C_CAMCMD_WRITE
			    | R92C_CAM_CTL0(idx)
			    | i;

			urtwn_write_4(sc, R92C_CAMWRITE, content);
			urtwn_write_4(sc, R92C_CAMCMD, command);
		}
	}

	/* Invalidate all CAM entries. */
	urtwn_write_4(sc, R92C_CAMCMD, R92C_CAMCMD_POLLING | R92C_CAMCMD_CLR);
}

static void __noinline
urtwn_pa_bias_init(struct urtwn_softc *sc)
{
	uint8_t reg;
	size_t i;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	usbwifi_isowned_ic(&sc->sc_uw);

	for (i = 0; i < sc->nrxchains; i++) {
		if (sc->pa_setting & (1U << i))
			continue;

		urtwn_rf_write(sc, i, R92C_RF_IPA, 0x0f406);
		urtwn_rf_write(sc, i, R92C_RF_IPA, 0x4f406);
		urtwn_rf_write(sc, i, R92C_RF_IPA, 0x8f406);
		urtwn_rf_write(sc, i, R92C_RF_IPA, 0xcf406);
	}
	if (!(sc->pa_setting & 0x10)) {
		reg = urtwn_read_1(sc, 0x16);
		reg = (reg & ~0xf0) | 0x90;
		urtwn_write_1(sc, 0x16, reg);
	}
}

static void __noinline
urtwn_rxfilter_init(struct urtwn_softc *sc)
{

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	usbwifi_isowned_ic(&sc->sc_uw);

	/* Initialize Rx filter. */
	/* TODO: use better filter for monitor mode. */
	urtwn_write_4(sc, R92C_RCR,
	    R92C_RCR_AAP | R92C_RCR_APM | R92C_RCR_AM | R92C_RCR_AB |
	    R92C_RCR_APP_ICV | R92C_RCR_AMF | R92C_RCR_HTC_LOC_CTRL |
	    R92C_RCR_APP_MIC | R92C_RCR_APP_PHYSTS |
	    R21A_RCR_DIS_CHK_14 | R21A_RCR_VHT_ACK);
	/* Accept all multicast frames. */
	urtwn_write_4(sc, R92C_MAR + 0, 0xffffffff);
	urtwn_write_4(sc, R92C_MAR + 4, 0xffffffff);
	/* Reject all control frames. */
	urtwn_write_2(sc, R92C_RXFLTMAP1, 0x0000);
	/* Accept all data frames. */
	urtwn_write_2(sc, R92C_RXFLTMAP2, 0xffff);
	/* Accept all management frames. */
	urtwn_write_2(sc, R92C_RXFLTMAP0, 0xffff);
}

static void __noinline
urtwn_edca_init(struct urtwn_softc *sc)
{

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	usbwifi_isowned_ic(&sc->sc_uw);

	/* set spec SIFS (used in NAV) */
	urtwn_write_2(sc, R92C_SPEC_SIFS, 0x100a);
	urtwn_write_2(sc, R92C_MAC_SPEC_SIFS, 0x100a);

	/* set SIFS CCK/OFDM */
	urtwn_write_2(sc, R92C_SIFS_CCK, 0x100a);
	urtwn_write_2(sc, R92C_SIFS_OFDM, 0x100a);

	/* TXOP */
	urtwn_write_4(sc, R92C_EDCA_BE_PARAM, 0x005ea42b);
	urtwn_write_4(sc, R92C_EDCA_BK_PARAM, 0x0000a44f);
	urtwn_write_4(sc, R92C_EDCA_VI_PARAM, 0x005ea324);
	urtwn_write_4(sc, R92C_EDCA_VO_PARAM, 0x002fa226);

	if (ISSET(sc->chip, URTWN_CHIP_21A)) {
		/* 80 MHz clock */
		urtwn_write_1(sc, R92C_USTIME_TSF, 0x50);
		urtwn_write_1(sc, R92C_USTIME_EDCA, 0x50);
	}
}

static void
urtwn_write_txpower(struct urtwn_softc *sc, int chain,
    uint16_t power[URTWN_RIDX_COUNT])
{
	uint32_t reg;

	URTWNHIST_FUNC();
	URTWNHIST_CALLARGS("chain=%jd", chain, 0, 0, 0);

	/* Write per-CCK rate Tx power. */
	if (chain == 0) {
		reg = urtwn_bb_read(sc, R92C_TXAGC_A_CCK1_MCS32);
		reg = RW(reg, R92C_TXAGC_A_CCK1,  power[0]);
		urtwn_bb_write(sc, R92C_TXAGC_A_CCK1_MCS32, reg);

		reg = urtwn_bb_read(sc, R92C_TXAGC_B_CCK11_A_CCK2_11);
		reg = RW(reg, R92C_TXAGC_A_CCK2,  power[1]);
		reg = RW(reg, R92C_TXAGC_A_CCK55, power[2]);
		reg = RW(reg, R92C_TXAGC_A_CCK11, power[3]);
		urtwn_bb_write(sc, R92C_TXAGC_B_CCK11_A_CCK2_11, reg);
	} else {
		reg = urtwn_bb_read(sc, R92C_TXAGC_B_CCK1_55_MCS32);
		reg = RW(reg, R92C_TXAGC_B_CCK1,  power[0]);
		reg = RW(reg, R92C_TXAGC_B_CCK2,  power[1]);
		reg = RW(reg, R92C_TXAGC_B_CCK55, power[2]);
		urtwn_bb_write(sc, R92C_TXAGC_B_CCK1_55_MCS32, reg);

		reg = urtwn_bb_read(sc, R92C_TXAGC_B_CCK11_A_CCK2_11);
		reg = RW(reg, R92C_TXAGC_B_CCK11, power[3]);
		urtwn_bb_write(sc, R92C_TXAGC_B_CCK11_A_CCK2_11, reg);
	}
	/* Write per-OFDM rate Tx power. */
	urtwn_bb_write(sc, R92C_TXAGC_RATE18_06(chain),
	    SM(R92C_TXAGC_RATE06, power[ 4]) |
	    SM(R92C_TXAGC_RATE09, power[ 5]) |
	    SM(R92C_TXAGC_RATE12, power[ 6]) |
	    SM(R92C_TXAGC_RATE18, power[ 7]));
	urtwn_bb_write(sc, R92C_TXAGC_RATE54_24(chain),
	    SM(R92C_TXAGC_RATE24, power[ 8]) |
	    SM(R92C_TXAGC_RATE36, power[ 9]) |
	    SM(R92C_TXAGC_RATE48, power[10]) |
	    SM(R92C_TXAGC_RATE54, power[11]));
	/* Write per-MCS Tx power. */
	urtwn_bb_write(sc, R92C_TXAGC_MCS03_MCS00(chain),
	    SM(R92C_TXAGC_MCS00,  power[12]) |
	    SM(R92C_TXAGC_MCS01,  power[13]) |
	    SM(R92C_TXAGC_MCS02,  power[14]) |
	    SM(R92C_TXAGC_MCS03,  power[15]));
	urtwn_bb_write(sc, R92C_TXAGC_MCS07_MCS04(chain),
	    SM(R92C_TXAGC_MCS04,  power[16]) |
	    SM(R92C_TXAGC_MCS05,  power[17]) |
	    SM(R92C_TXAGC_MCS06,  power[18]) |
	    SM(R92C_TXAGC_MCS07,  power[19]));
	urtwn_bb_write(sc, R92C_TXAGC_MCS11_MCS08(chain),
	    SM(R92C_TXAGC_MCS08,  power[20]) |
	    SM(R92C_TXAGC_MCS09,  power[21]) |
	    SM(R92C_TXAGC_MCS10,  power[22]) |
	    SM(R92C_TXAGC_MCS11,  power[23]));
	urtwn_bb_write(sc, R92C_TXAGC_MCS15_MCS12(chain),
	    SM(R92C_TXAGC_MCS12,  power[24]) |
	    SM(R92C_TXAGC_MCS13,  power[25]) |
	    SM(R92C_TXAGC_MCS14,  power[26]) |
	    SM(R92C_TXAGC_MCS15,  power[27]));
}

static void
urtwn_r21a_write_txpower(struct urtwn_softc *sc, struct ieee80211_channel *c,
	int chain, uint16_t power[RTWN_RIDX_COUNT])
{

	if (IEEE80211_IS_CHAN_2GHZ(c)) {
		/* Write per-CCK rate Tx power. */
		urtwn_bb_write(sc, R21A_TXAGC_CCK11_1(chain),
		    SM(R21A_TXAGC_CCK1,  power[RTWN_RIDX_CCK1]) |
		    SM(R21A_TXAGC_CCK2,  power[RTWN_RIDX_CCK2]) |
		    SM(R21A_TXAGC_CCK55, power[RTWN_RIDX_CCK55]) |
		    SM(R21A_TXAGC_CCK11, power[RTWN_RIDX_CCK11]));
	}

	/* Write per-OFDM rate Tx power. */
	urtwn_bb_write(sc, R21A_TXAGC_OFDM18_6(chain),
	    SM(R21A_TXAGC_OFDM06, power[RTWN_RIDX_OFDM6]) |
	    SM(R21A_TXAGC_OFDM09, power[RTWN_RIDX_OFDM9]) |
	    SM(R21A_TXAGC_OFDM12, power[RTWN_RIDX_OFDM12]) |
	    SM(R21A_TXAGC_OFDM18, power[RTWN_RIDX_OFDM18]));
	urtwn_bb_write(sc, R21A_TXAGC_OFDM54_24(chain),
	    SM(R21A_TXAGC_OFDM24, power[RTWN_RIDX_OFDM24]) |
	    SM(R21A_TXAGC_OFDM36, power[RTWN_RIDX_OFDM36]) |
	    SM(R21A_TXAGC_OFDM48, power[RTWN_RIDX_OFDM48]) |
	    SM(R21A_TXAGC_OFDM54, power[RTWN_RIDX_OFDM54]));
	/* Write per-MCS Tx power. */
	urtwn_bb_write(sc, R21A_TXAGC_MCS3_0(chain),
	    SM(R21A_TXAGC_MCS0, power[RTWN_RIDX_HT_MCS(0)]) |
	    SM(R21A_TXAGC_MCS1, power[RTWN_RIDX_HT_MCS(1)]) |
	    SM(R21A_TXAGC_MCS2, power[RTWN_RIDX_HT_MCS(2)]) |
	    SM(R21A_TXAGC_MCS3, power[RTWN_RIDX_HT_MCS(3)]));
	urtwn_bb_write(sc, R21A_TXAGC_MCS7_4(chain),
	    SM(R21A_TXAGC_MCS4, power[RTWN_RIDX_HT_MCS(4)]) |
	    SM(R21A_TXAGC_MCS5, power[RTWN_RIDX_HT_MCS(5)]) |
	    SM(R21A_TXAGC_MCS6, power[RTWN_RIDX_HT_MCS(6)]) |
	    SM(R21A_TXAGC_MCS7, power[RTWN_RIDX_HT_MCS(7)]));
	if (sc->ntxchains >= 2) {
		urtwn_bb_write(sc, R21A_TXAGC_MCS11_8(chain),
		    SM(R21A_TXAGC_MCS8,  power[RTWN_RIDX_HT_MCS(8)]) |
		    SM(R21A_TXAGC_MCS9,  power[RTWN_RIDX_HT_MCS(9)]) |
		    SM(R21A_TXAGC_MCS10, power[RTWN_RIDX_HT_MCS(10)]) |
		    SM(R21A_TXAGC_MCS11, power[RTWN_RIDX_HT_MCS(11)]));
		urtwn_bb_write(sc, R21A_TXAGC_MCS15_12(chain),
		    SM(R21A_TXAGC_MCS12, power[RTWN_RIDX_HT_MCS(12)]) |
		    SM(R21A_TXAGC_MCS13, power[RTWN_RIDX_HT_MCS(13)]) |
		    SM(R21A_TXAGC_MCS14, power[RTWN_RIDX_HT_MCS(14)]) |
		    SM(R21A_TXAGC_MCS15, power[RTWN_RIDX_HT_MCS(15)]));
	}

	/* TODO: VHT rates */
}

static void
urtwn_get_txpower(struct urtwn_softc *sc, size_t chain, u_int chan,
     u_int ht40m, uint16_t power[URTWN_RIDX_COUNT])
{
	struct r92c_rom *rom = &sc->rom;
	uint16_t cckpow, ofdmpow, htpow, diff, maxpow;
	const struct rtwn_txpwr *base;
	int ridx, group;

	URTWNHIST_FUNC();
	URTWNHIST_CALLARGS("chain=%jd, chan=%jd", chain, chan, 0, 0);

	/* Determine channel group. */
	if (chan <= 3) {
		group = 0;
	} else if (chan <= 9) {
		group = 1;
	} else {
		group = 2;
	}

	/* Get original Tx power based on board type and RF chain. */
	if (!(sc->chip & URTWN_CHIP_92C)) {
		if (sc->board_type == R92C_BOARD_TYPE_HIGHPA) {
			base = &rtl8188ru_txagc[chain];
		} else {
			base = &rtl8192cu_txagc[chain];
		}
	} else {
		base = &rtl8192cu_txagc[chain];
	}

	memset(power, 0, URTWN_RIDX_COUNT * sizeof(power[0]));
	if (sc->regulatory == 0) {
		for (ridx = 0; ridx <= 3; ridx++) {
			power[ridx] = base->pwr[0][ridx];
		}
	}
	for (ridx = 4; ridx < URTWN_RIDX_COUNT; ridx++) {
		if (sc->regulatory == 3) {
			power[ridx] = base->pwr[0][ridx];
			/* Apply vendor limits. */
			if (ht40m) {
				maxpow = rom->ht40_max_pwr[group];
			} else {
				maxpow = rom->ht20_max_pwr[group];
			}
			maxpow = (maxpow >> (chain * 4)) & 0xf;
			if (power[ridx] > maxpow) {
				power[ridx] = maxpow;
			}
		} else if (sc->regulatory == 1) {
			if (!ht40m) {
				power[ridx] = base->pwr[group][ridx];
			}
		} else if (sc->regulatory != 2) {
			power[ridx] = base->pwr[0][ridx];
		}
	}

	/* Compute per-CCK rate Tx power. */
	cckpow = rom->cck_tx_pwr[chain][group];
	for (ridx = 0; ridx <= 3; ridx++) {
		power[ridx] += cckpow;
		if (power[ridx] > R92C_MAX_TX_PWR) {
			power[ridx] = R92C_MAX_TX_PWR;
		}
	}

	htpow = rom->ht40_1s_tx_pwr[chain][group];
	if (sc->ntxchains > 1) {
		/* Apply reduction for 2 spatial streams. */
		diff = rom->ht40_2s_tx_pwr_diff[group];
		diff = (diff >> (chain * 4)) & 0xf;
		htpow = (htpow > diff) ? htpow - diff : 0;
	}

	/* Compute per-OFDM rate Tx power. */
	diff = rom->ofdm_tx_pwr_diff[group];
	diff = (diff >> (chain * 4)) & 0xf;
	ofdmpow = htpow + diff; /* HT->OFDM correction. */
	for (ridx = 4; ridx <= 11; ridx++) {
		power[ridx] += ofdmpow;
		if (power[ridx] > R92C_MAX_TX_PWR) {
			power[ridx] = R92C_MAX_TX_PWR;
		}
	}

	/* Compute per-MCS Tx power. */
	if (!ht40m) {
		diff = rom->ht20_tx_pwr_diff[group];
		diff = (diff >> (chain * 4)) & 0xf;
		htpow += diff;	/* HT40->HT20 correction. */
	}
	for (ridx = 12; ridx < URTWN_RIDX_COUNT; ridx++) {
		power[ridx] += htpow;
		if (power[ridx] > R92C_MAX_TX_PWR) {
			power[ridx] = R92C_MAX_TX_PWR;
		}
	}
#ifdef URTWN_DEBUG
	if (urtwn_debug & DBG_RF) {
		/* Dump per-rate Tx power values. */
		printf("%s: %s: Tx power for chain %zd:\n",
		    device_xname(sc->sc_uw.uw_dev), __func__, chain);
		for (ridx = 0; ridx < URTWN_RIDX_COUNT; ridx++) {
			printf("%s: %s: Rate %d = %u\n",
			    device_xname(sc->sc_uw.uw_dev), __func__, ridx,
			    power[ridx]);
		}
	}
#endif
}

void
urtwn_r88e_get_txpower(struct urtwn_softc *sc, size_t chain, u_int chan,
    u_int ht40m, uint16_t power[URTWN_RIDX_COUNT])
{
	uint16_t cckpow, ofdmpow, bw20pow, htpow;
	const struct rtwn_r88e_txpwr *base;
	int ridx, group;

	URTWNHIST_FUNC();
	URTWNHIST_CALLARGS("chain=%jd, chan=%jd", chain, chan, 0, 0);

	/* Determine channel group. */
	if (chan <= 2)
		group = 0;
	else if (chan <= 5)
		group = 1;
	else if (chan <= 8)
		group = 2;
	else if (chan <= 11)
		group = 3;
	else if (chan <= 13)
		group = 4;
	else
		group = 5;

	/* Get original Tx power based on board type and RF chain. */
	base = &rtl8188eu_txagc[chain];

	memset(power, 0, URTWN_RIDX_COUNT * sizeof(power[0]));
	if (sc->regulatory == 0) {
		for (ridx = 0; ridx <= 3; ridx++)
			power[ridx] = base->pwr[0][ridx];
	}
	for (ridx = 4; ridx < URTWN_RIDX_COUNT; ridx++) {
		if (sc->regulatory == 3)
			power[ridx] = base->pwr[0][ridx];
		else if (sc->regulatory == 1) {
			if (!ht40m)
				power[ridx] = base->pwr[group][ridx];
		} else if (sc->regulatory != 2)
			power[ridx] = base->pwr[0][ridx];
	}

	/* Compute per-CCK rate Tx power. */
	cckpow = sc->cck_tx_pwr[group];
	for (ridx = 0; ridx <= 3; ridx++) {
		power[ridx] += cckpow;
		if (power[ridx] > R92C_MAX_TX_PWR)
			power[ridx] = R92C_MAX_TX_PWR;
	}

	htpow = sc->ht40_tx_pwr[group];

	/* Compute per-OFDM rate Tx power. */
	ofdmpow = htpow + sc->ofdm_tx_pwr_diff;
	for (ridx = 4; ridx <= 11; ridx++) {
		power[ridx] += ofdmpow;
		if (power[ridx] > R92C_MAX_TX_PWR)
			power[ridx] = R92C_MAX_TX_PWR;
	}

	bw20pow = htpow + sc->bw20_tx_pwr_diff;
	for (ridx = 12; ridx <= 27; ridx++) {
		power[ridx] += bw20pow;
		if (power[ridx] > R92C_MAX_TX_PWR)
			power[ridx] = R92C_MAX_TX_PWR;
	}
}

static int
urtwn_r21a_get_power_group(struct urtwn_softc *sc, struct ieee80211_channel *c)
{
	int group;

	int chan = rtwn_chan2centieee(c);
	if (IEEE80211_IS_CHAN_2GHZ(c)) {
		if (chan <= 2)			group = 0;
		else if (chan <= 5)		group = 1;
		else if (chan <= 8)		group = 2;
		else if (chan <= 11)		group = 3;
		else if (chan <= 14)		group = 4;
		else {
			KASSERT(0);
			return (-1);
		}
	} else if (IEEE80211_IS_CHAN_5GHZ(c)) {
		if (chan < 36)
			return (-1);

		if (chan <= 42)			group = 0;
		else if (chan <= 48)		group = 1;
		else if (chan <= 58)		group = 2;
		else if (chan <= 64)		group = 3;
		else if (chan <= 106)		group = 4;
		else if (chan <= 114)		group = 5;
		else if (chan <= 122)		group = 6;
		else if (chan <= 130)		group = 7;
		else if (chan <= 138)		group = 8;
		else if (chan <= 144)		group = 9;
		else if (chan <= 155)		group = 10;
		else if (chan <= 161)		group = 11;
		else if (chan <= 171)		group = 12;
		else if (chan <= 177)		group = 13;
		else {
			KASSERT(0);
			return (-1);
		}
	} else {
		KASSERT(0); /* wrong channel band */
		return (-1);
	}

	return (group);
}

static void
urtwn_r21a_get_txpower(struct urtwn_softc *sc, struct ieee80211_channel *c, int chain,
    uint16_t power[URTWN_RIDX_COUNT])
{
	struct urtwn_r21a_data *r21a_data = sc->sc_chip_priv.data;
	int i, ridx, group, max_mcs;

	/* Determine channel group. */
	group = urtwn_r21a_get_power_group(sc, c);
	if (group == -1) {	/* shouldn't happen */
		device_printf(sc->sc_uw.uw_dev, "%s: incorrect channel\n", __func__);
		return;
	}

	max_mcs = RTWN_RIDX_HT_MCS(sc->ntxchains * 8 - 1);
	if (IEEE80211_IS_CHAN_2GHZ(c)) {
		for (ridx = RTWN_RIDX_CCK1; ridx <= RTWN_RIDX_CCK11; ridx++)
			power[ridx] = r21a_data->cck_tx_pwr[chain][group];
		for (ridx = RTWN_RIDX_OFDM6; ridx <= max_mcs; ridx++)
			power[ridx] = r21a_data->ht40_tx_pwr_2g[chain][group];

		for (ridx = RTWN_RIDX_OFDM6; ridx <= RTWN_RIDX_OFDM54; ridx++)
			power[ridx] += r21a_data->ofdm_tx_pwr_diff_2g[chain][0];

		for (i = 0; i < sc->ntxchains; i++) {
			uint8_t min_mcs;
			uint8_t pwr_diff;

			if (IEEE80211_IS_CHAN_HT40(c))
				pwr_diff = r21a_data->bw40_tx_pwr_diff_2g[chain][i];
			else
				pwr_diff = r21a_data->bw20_tx_pwr_diff_2g[chain][i];

			min_mcs = RTWN_RIDX_HT_MCS(i * 8);
			for (ridx = min_mcs; ridx <= max_mcs; ridx++)
				power[ridx] += pwr_diff;
		}
	} else {	/* 5GHz */
		for (ridx = RTWN_RIDX_OFDM6; ridx <= max_mcs; ridx++)
			power[ridx] = r21a_data->ht40_tx_pwr_5g[chain][group];

		for (ridx = RTWN_RIDX_OFDM6; ridx <= RTWN_RIDX_OFDM54; ridx++)
			power[ridx] += r21a_data->ofdm_tx_pwr_diff_5g[chain][0];

		for (i = 0; i < sc->ntxchains; i++) {
			uint8_t min_mcs;
			uint8_t pwr_diff;

			if (IEEE80211_IS_CHAN_HT40(c))
				pwr_diff = r21a_data->bw40_tx_pwr_diff_5g[chain][i];
			else
				pwr_diff = r21a_data->bw20_tx_pwr_diff_5g[chain][i];

			min_mcs = RTWN_RIDX_HT_MCS(i * 8);
			for (ridx = min_mcs; ridx <= max_mcs; ridx++)
				power[ridx] += pwr_diff;
		}
	}

	/* Apply max limit. */
	for (ridx = RTWN_RIDX_CCK1; ridx <= max_mcs; ridx++) {
		if (power[ridx] > R92C_MAX_TX_PWR)
			power[ridx] = R92C_MAX_TX_PWR;
	}
}

static void
urtwn_set_txpower(struct urtwn_softc *sc, u_int chan, u_int ht40m)
{
	uint16_t power[URTWN_RIDX_COUNT];
	size_t i;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	for (i = 0; i < sc->ntxchains; i++) {
		/* Compute per-rate Tx power values. */
		if (ISSET(sc->chip, URTWN_CHIP_88E) ||
		    ISSET(sc->chip, URTWN_CHIP_92EU))
			urtwn_r88e_get_txpower(sc, i, chan, ht40m, power);
		else
			urtwn_get_txpower(sc, i, chan, ht40m, power);
		/* Write per-rate Tx power values to hardware. */
		urtwn_write_txpower(sc, i, power);
	}
}

static void
urtwn_r21a_set_txpower(struct urtwn_softc *sc, struct ieee80211_channel *c)
{
	uint16_t power[RTWN_RIDX_COUNT];
	size_t i;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	for (i = 0; i < sc->ntxchains; i++) {
		memset(power, 0, sizeof(power));
		/* Compute per-rate Tx power values. */
		urtwn_r21a_get_txpower(sc, c, i, power);
		/* Write per-rate Tx power values to hardware. */
		urtwn_r21a_write_txpower(sc, c, i, power);
	}
}

static void
urtwn_set_chan(struct urtwn_softc *sc, struct ieee80211_channel *c)
{
	struct ieee80211com *ic = usbwifi_ic(&sc->sc_uw);
	u_int chan, ht40m;
	size_t i;

	chan = ieee80211_chan2ieee(ic, c);	/* XXX center freq! */
	ht40m = c->ic_flags & IEEE80211_CHAN_HT40;

	URTWNHIST_FUNC();
	URTWNHIST_CALLARGS("chan=%jd", chan, 0, 0, 0);

	usbwifi_isowned_ic(&sc->sc_uw);

	if (ht40m == IEEE80211_CHAN_HT40U) {
		chan += 2;
	} else if (ht40m == IEEE80211_CHAN_HT40D){
		chan -= 2;
	}

	/* Set Tx power for this new channel. */
	urtwn_set_txpower(sc, chan, ht40m);

	for (i = 0; i < sc->nrxchains; i++) {
		urtwn_rf_write(sc, i, R92C_RF_CHNLBW,
		    RW(sc->rf_chnlbw[i], R92C_RF_CHNLBW_CHNL, chan));
	}

	if (ht40m) {
		/* Is secondary channel below or above primary? */
		int prichlo = (ht40m == IEEE80211_CHAN_HT40U);
		uint32_t reg;

		urtwn_write_1(sc, R92C_BWOPMODE,
		    urtwn_read_1(sc, R92C_BWOPMODE) & ~R92C_BWOPMODE_20MHZ);

		reg = urtwn_read_1(sc, R92C_RRSR + 2);
		reg = (reg & ~0x6f) | (prichlo ? 1 : 2) << 5;
		urtwn_write_1(sc, R92C_RRSR + 2, (uint8_t)reg);

		urtwn_bb_write(sc, R92C_FPGA0_RFMOD,
		    urtwn_bb_read(sc, R92C_FPGA0_RFMOD) | R92C_RFMOD_40MHZ);
		urtwn_bb_write(sc, R92C_FPGA1_RFMOD,
		    urtwn_bb_read(sc, R92C_FPGA1_RFMOD) | R92C_RFMOD_40MHZ);

		/* Set CCK side band. */
		reg = urtwn_bb_read(sc, R92C_CCK0_SYSTEM);
		reg = (reg & ~0x00000010) | (prichlo ? 0 : 1) << 4;
		urtwn_bb_write(sc, R92C_CCK0_SYSTEM, reg);

		reg = urtwn_bb_read(sc, R92C_OFDM1_LSTF);
		reg = (reg & ~0x00000c00) | (prichlo ? 1 : 2) << 10;
		urtwn_bb_write(sc, R92C_OFDM1_LSTF, reg);

		urtwn_bb_write(sc, R92C_FPGA0_ANAPARAM2,
		    urtwn_bb_read(sc, R92C_FPGA0_ANAPARAM2) &
		    ~R92C_FPGA0_ANAPARAM2_CBW20);

		reg = urtwn_bb_read(sc, 0x818);
		reg = (reg & ~0x0c000000) | (prichlo ? 2 : 1) << 26;
		urtwn_bb_write(sc, 0x818, reg);

		/* Select 40MHz bandwidth. */
		urtwn_rf_write(sc, 0, R92C_RF_CHNLBW,
		    (sc->rf_chnlbw[0] & ~0xfff) | chan);
	} else {
		urtwn_write_1(sc, R92C_BWOPMODE,
		    urtwn_read_1(sc, R92C_BWOPMODE) | R92C_BWOPMODE_20MHZ);

		urtwn_bb_write(sc, R92C_FPGA0_RFMOD,
		    urtwn_bb_read(sc, R92C_FPGA0_RFMOD) & ~R92C_RFMOD_40MHZ);
		urtwn_bb_write(sc, R92C_FPGA1_RFMOD,
		    urtwn_bb_read(sc, R92C_FPGA1_RFMOD) & ~R92C_RFMOD_40MHZ);

		if (!ISSET(sc->chip, URTWN_CHIP_88E) &&
		    !ISSET(sc->chip, URTWN_CHIP_92EU)) {
			urtwn_bb_write(sc, R92C_FPGA0_ANAPARAM2,
			    urtwn_bb_read(sc, R92C_FPGA0_ANAPARAM2) |
			    R92C_FPGA0_ANAPARAM2_CBW20);
		}

		/* Select 20MHz bandwidth. */
		urtwn_rf_write(sc, 0, R92C_RF_CHNLBW,
		    (sc->rf_chnlbw[0] & ~0xfff) | chan |
		    (ISSET(sc->chip, URTWN_CHIP_88E) ||
		     ISSET(sc->chip, URTWN_CHIP_92EU) ?
		      R88E_RF_CHNLBW_BW20 : R92C_RF_CHNLBW_BW20));
	}
}

static void
urtwn_get_rates(struct urtwn_softc *sc, const struct ieee80211_rateset *rs,
    const struct ieee80211_htrateset *rs_ht, uint32_t *rates_p,
    int *maxrate_p, int basic_rates)
{
	uint32_t rates;
	uint8_t ridx;
	int i, maxrate;
	
	URTWNHIST_FUNC(); URTWNHIST_CALLED();
	/* Get rates mask. */
	rates = 0;
	maxrate = 0;

	/* This is for 11bg */
	for (i = 0; i < rs->rs_nrates; i++) {
		/* Convert 802.11 rate to HW rate index. */
		ridx = rate2ridx(IEEE80211_RV(rs->rs_rates[i]));
		if (ridx == RTWN_RIDX_UNKNOWN)	/* Unknown rate, skip. */
			continue;
		if (((rs->rs_rates[i] & IEEE80211_RATE_BASIC) != 0) ||
		    !basic_rates) {
			rates |= 1 << ridx;
			if (ridx > maxrate)
				maxrate = ridx;
		}
	}

	/* If we're doing 11n, enable 11n rates */
	if (rs_ht != NULL && !basic_rates) {
		for (i = 0; i < rs_ht->rs_nrates; i++) {
			if ((rs_ht->rs_rates[i] & 0x7f) > 0xf)
				continue;
			/* 11n rates start at index 12 */
			ridx = RTWN_RIDX_HT_MCS((rs_ht->rs_rates[i]) & 0xf);
			rates |= (1 << ridx);

			/* Guard against the rate table being oddly ordered */
			if (ridx > maxrate)
				maxrate = ridx;
		}
	}

	DPRINTFN(DBG_FN, "%s: rates 0x%08X, maxrate %d\n", __func__, rates, maxrate, 0);

	if (rates_p != NULL)
		*rates_p = rates;
	if (maxrate_p != NULL)
		*maxrate_p = maxrate;
}


static void
urtwn_r21a_bypass_ext_lna_2ghz(struct urtwn_softc *sc)
{
	urtwn_bb_setbits(sc, R21A_RFE_INV(0), 0x00100000, 0);
	urtwn_bb_setbits(sc, R21A_RFE_INV(0), 0x00400000, 0);
	urtwn_bb_setbits(sc, R21A_RFE_PINMUX(0), 0, 0x07);
	urtwn_bb_setbits(sc, R21A_RFE_PINMUX(0), 0, 0x0700);
}

static void
urtwn_r21a_set_band_2ghz(struct urtwn_softc *sc, uint32_t basicrates)
{
	struct urtwn_r21a_data *r21a_data = sc->sc_chip_priv.data;
	/* Enable CCK / OFDM. */
	urtwn_bb_setbits(sc, R21A_OFDMCCK_EN,
	    0, R21A_OFDMCCK_EN_CCK | R21A_OFDMCCK_EN_OFDM);

	/* Turn off RF PA and LNA. */
	urtwn_bb_setbits(sc, R21A_RFE_PINMUX(0),
	    R21A_RFE_PINMUX_LNA_MASK, 0x7000);
	urtwn_bb_setbits(sc, R21A_RFE_PINMUX(0),
	    R21A_RFE_PINMUX_PA_A_MASK, 0x70);

	if (R21A_ROM_IS_LNA_EXT(r21a_data->ext_lna_2g)) {
		/* Turn on 2.4 GHz external LNA. */
		urtwn_bb_setbits(sc, R21A_RFE_INV(0), 0, 0x00100000);
		urtwn_bb_setbits(sc, R21A_RFE_INV(0), 0x00400000, 0);
		urtwn_bb_setbits(sc, R21A_RFE_PINMUX(0), 0x05, 0x02);
		urtwn_bb_setbits(sc, R21A_RFE_PINMUX(0), 0x0500, 0x0200);
	} else {
		/* Bypass 2.4 GHz external LNA. */
		urtwn_r21a_bypass_ext_lna_2ghz(sc);
	}

	/* Select AGC table. */
	urtwn_bb_setbits(sc, R21A_TX_SCALE(0), 0x0f00, 0);

	urtwn_bb_setbits(sc, R21A_TX_PATH, 0xf0, 0x10);
	urtwn_bb_setbits(sc, R21A_CCK_RX_PATH, 0x0f000000, 0x01000000);

	/* Write basic rates. */
	urtwn_set_basicrates(sc, basicrates);

	urtwn_write_1(sc, R21A_CCK_CHECK, 0);
}

static void
urtwn_r21a_set_band_5ghz(struct urtwn_softc *sc, uint32_t basicrates)
{
	struct urtwn_r21a_data *r21a_data = sc->sc_chip_priv.data;
	int ntries;

	urtwn_bb_setbits(sc, R21A_RFE_PINMUX(0),
	    R21A_RFE_PINMUX_LNA_MASK, 0x5000);
	urtwn_bb_setbits(sc, R21A_RFE_PINMUX(0),
	    R21A_RFE_PINMUX_PA_A_MASK, 0x40);

	if (R21A_ROM_IS_LNA_EXT(r21a_data->ext_lna_2g)) {
		/* Bypass 2.4 GHz external LNA. */
		urtwn_r21a_bypass_ext_lna_2ghz(sc);
	}

	urtwn_write_1(sc, R21A_CCK_CHECK, R21A_CCK_CHECK_5GHZ);

	for (ntries = 0; ntries < 100; ntries++) {
		if ((urtwn_read_2(sc, R21A_TXPKT_EMPTY) & 0x30) == 0x30)
			break;

		DELAY(25);
	}
	if (ntries == 100) {
		device_printf(sc->sc_uw.uw_dev,
		    "%s: TXPKT_EMPTY check failed (%04X)\n",
		    __func__, urtwn_read_2(sc, R21A_TXPKT_EMPTY));
	}

	/* Enable OFDM. */
	urtwn_bb_setbits(sc, R21A_OFDMCCK_EN, R21A_OFDMCCK_EN_CCK,
	    R21A_OFDMCCK_EN_OFDM);

	/* Select AGC table. */
	urtwn_bb_setbits(sc, R21A_TX_SCALE(0), 0x0f00, 0x0100);

	urtwn_bb_setbits(sc, R21A_TX_PATH, 0xf0, 0);
	urtwn_bb_setbits(sc, R21A_CCK_RX_PATH, 0, 0x0f000000);

	/* Write basic rates. */
	urtwn_set_basicrates(sc, basicrates);
}

static void
urtwn_r21a_set_band(struct urtwn_softc *sc, struct ieee80211_channel *c,
	struct ieee80211com *ic)
{
	struct r21a_rom *rom = &sc->sc_chip_priv.data->r21a_rom;
	uint32_t basicrates;
	uint8_t swing;
	int i;

	/* Check if band was changed. */
	if (ic->ic_nrunning && IEEE80211_IS_CHAN_5GHZ(c) ^
	    !(urtwn_read_1(sc, R21A_CCK_CHECK) & R21A_CCK_CHECK_5GHZ))
		return;

	urtwn_get_rates(sc, ieee80211_get_suprates(ic, c), NULL, &basicrates,
	    NULL, 1);
	if (IEEE80211_IS_CHAN_2GHZ(c)) {
		urtwn_r21a_set_band_2ghz(sc, basicrates);
		swing = URTWN_GET_ROM_VAR(rom->tx_bbswing_2g, 0);
	} else if (IEEE80211_IS_CHAN_5GHZ(c)) {
		urtwn_r21a_set_band_5ghz(sc, basicrates);
		swing = URTWN_GET_ROM_VAR(rom->tx_bbswing_5g, 0);
	} else {
		KASSERT(0);
		return;
	}

	/* XXX PATH_B is set by vendor driver. */
	for (i = 0; i < 2; i++) {
		uint16_t val = 0;

		switch ((swing >> i * 2) & 0x3) {
		case 0:
			val = 0x200;	/* 0 dB	*/
			break;
		case 1:
			val = 0x16a;	/* -3 dB */
			break;
		case 2:
			val = 0x101;	/* -6 dB */
			break;
		case 3:
			val = 0xb6;	/* -9 dB */
			break;
		}
		urtwn_bb_setbits(sc, R21A_TX_SCALE(i), R21A_TX_SCALE_SWING_M,
		    val << R21A_TX_SCALE_SWING_S);
	}
}

static void
urtwn_r21a_set_chan(struct urtwn_softc *sc, struct ieee80211_channel *c)
{
	struct ieee80211com *ic = usbwifi_ic(&sc->sc_uw);
	u_int chan;
	uint32_t val;
	size_t i;


	URTWNHIST_FUNC();
	urtwn_r21a_set_band(sc, c, ic);
	chan = ieee80211_chan2ieee(ic, c);	/* XXX center freq! */
	URTWNHIST_CALLARGS("chan=%jd", chan, 0, 0, 0);

	
	if (36 <= chan && chan <= 48)
		val = 0x09280000;
	else if (50 <= chan && chan <= 64)
		val = 0x08a60000;
	else if (100 <= chan && chan <= 116)
		val = 0x08a40000;
	else if (118 <= chan)
		val = 0x08240000;
	else
		val = 0x12d40000;

	urtwn_bb_setbits(sc, R21A_FC_AREA, 0x1ffe0000, val);

	for (i = 0; i < sc->nrxchains; i++) {
		if (36 <= chan && chan <= 64)
			val = 0x10100;
		else if (100 <= chan && chan <= 140)
			val = 0x30100;
		else if (140 < chan)
			val = 0x50100;
		else
			val = 0x00000;

		urtwn_rf_setbits(sc, i, R92C_RF_CHNLBW, 0x70300, val);

		KASSERT(chan <= 0xff);
		urtwn_rf_setbits(sc, i, R92C_RF_CHNLBW, 0xff, chan);
	}

	if (IEEE80211_IS_CHAN_HT40(c)) {	/* 40 MHz */
		uint8_t ext_chan;

		if (IEEE80211_IS_CHAN_HT40U(c))
			ext_chan = R21A_DATA_SEC_PRIM_DOWN_20;
		else
			ext_chan = R21A_DATA_SEC_PRIM_UP_20;

		urtwn_setbits_2(sc, R92C_WMAC_TRXPTCL_CTL, 0x100, 0x80);
		urtwn_write_1(sc, R21A_DATA_SEC, ext_chan);

		urtwn_bb_setbits(sc, R21A_RFMOD, 0x003003c3, 0x00300201);
		urtwn_bb_setbits(sc, R21A_ADC_BUF_CLK, 0x40000000, 0);

		/* discard high 4 bits */
		val = urtwn_bb_read(sc, R21A_RFMOD);
		val = RW(val, R21A_RFMOD_EXT_CHAN, ext_chan);
		urtwn_bb_write(sc, R21A_RFMOD, val);

		val = urtwn_bb_read(sc, R21A_CCA_ON_SEC);
		val = RW(val, R21A_CCA_ON_SEC_EXT_CHAN, ext_chan);
		urtwn_bb_write(sc, R21A_CCA_ON_SEC, val);

		if (urtwn_read_1(sc, 0x837) & 0x04)
			val = 0x01800000;
		else if (sc->nrxchains == 2 && sc->ntxchains == 2)
			val = 0x01c00000;
		else
			val = 0x02000000;

		urtwn_bb_setbits(sc, R21A_L1_PEAK_TH, 0x03c00000, val);

		if (IEEE80211_IS_CHAN_HT40U(c))
			urtwn_bb_setbits(sc, R92C_CCK0_SYSTEM, 0x10, 0);
		else
			urtwn_bb_setbits(sc, R92C_CCK0_SYSTEM, 0, 0x10);

		val = 0x400;
	} else {	/* 20 MHz */
		urtwn_setbits_2(sc, R92C_WMAC_TRXPTCL_CTL, 0x180, 0);
		urtwn_write_1(sc, R21A_DATA_SEC, R21A_DATA_SEC_NO_EXT);

		urtwn_bb_setbits(sc, R21A_RFMOD, 0x003003c3, 0x00300200);
		urtwn_bb_setbits(sc, R21A_ADC_BUF_CLK, 0x40000000, 0);

		if (sc->nrxchains == 2 && sc->ntxchains == 2)
			val = 0x01c00000;
		else
			val = 0x02000000;

		urtwn_bb_setbits(sc, R21A_L1_PEAK_TH, 0x03c00000, val);

		val = 0xc00;
	}

	for (i = 0; i < sc->nrxchains; i++)
		urtwn_rf_setbits(sc, i, R92C_RF_CHNLBW, 0xc00, val);

	/* Set Tx power for this new channel. */
	urtwn_r21a_set_txpower(sc, c);
}

static void
urtwn_r21au_dfs_radar_disable(struct urtwn_softc *sc)
{
	urtwn_bb_setbits(sc, 0x924, 0x00008000, 0);
}

static int
urtwn_r21au_dfs_radar_is_enabled(struct urtwn_softc *sc)
{
	return !!(urtwn_bb_read(sc, 0x924) & 0x00008000);
}

static int
urtwn_r21au_dfs_radar_reset(struct urtwn_softc *sc)
{
	urtwn_bb_setbits(sc, 0x924, 0x00008000, 0);
	urtwn_bb_setbits(sc, 0x924, 0, 0x00008000);
	return (0);
}

static int
urtwn_r21au_dfs_radar_enable(struct urtwn_softc *sc)
{
	urtwn_bb_setbits(sc, 0x814, 0x3fffffff, 0x04cc4d10);
	urtwn_bb_setbits(sc, R21A_BW_INDICATION, 0xff, 0x06);
	urtwn_bb_write(sc, 0x918, 0x1c16ecdf);
	urtwn_bb_write(sc, 0x924, 0x0152a400);
	urtwn_bb_write(sc, 0x91c, 0x0fa21a20);
	urtwn_bb_write(sc, 0x920, 0xe0f57204);

	return (urtwn_r21au_dfs_radar_reset(sc));
}

static int
urtwn_r21au_dfs_radar_is_detected(struct urtwn_softc *sc)
{
	return !!(urtwn_bb_read(sc, 0xf98) & 0x00020000);
}

static void
urtwn_r21au_chan_check(void *arg, int npending)
{
	struct urtwn_softc *sc = arg;
	struct urtwn_r21a_data *r21a_data = sc->sc_chip_priv.data;
	struct ieee80211com *ic = usbwifi_ic(&sc->sc_uw);;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();
	DPRINTFN(DBG_RF, "%s: periodical radar detection task\n", 
		__func__, 0, 0, 0);

	if (!urtwn_r21au_dfs_radar_is_enabled(sc)) {
		if (r21a_data->flags & R21A_RADAR_ENABLED) {
			/* should not happen */
			device_printf(sc->sc_uw.uw_dev,
			    "%s: radar detection was turned off "
			    "unexpectedly, resetting...\n", __func__);

			/* XXX something more appropriate? */
			ieee80211_restart_all(ic);
		}
		return;
	}

	if (urtwn_r21au_dfs_radar_is_detected(sc)) {
		urtwn_r21au_dfs_radar_reset(sc);

		DPRINTFN(DBG_RF, "%s: got radar event\n",
		    __func__, 0, 0, 0);

		IEEE80211_LOCK(ic);

		ieee80211_dfs_notify_radar(ic, ic->ic_curchan);

		IEEE80211_UNLOCK(ic);
	}

	if (r21a_data->flags & R21A_RADAR_ENABLED) {
		taskqueue_enqueue_timeout(sc->sc_tq,
		    &r21a_data->chan_check, URTWN_R21AU_RADAR_CHECK_PERIOD);
	}
}

static void __noinline
urtwn_iq_calib(struct urtwn_softc *sc, bool inited)
{

	URTWNHIST_FUNC();
	URTWNHIST_CALLARGS("inited=%jd", inited, 0, 0, 0);

	uint32_t addaBackup[16], iqkBackup[4], piMode;

#ifdef notyet
	uint32_t odfm0_agccore_regs[3];
	uint32_t ant_regs[3];
	uint32_t rf_regs[8];
#endif
	uint32_t reg0, reg1, reg2;
	int i, attempt;

#ifdef notyet
	urtwn_write_1(sc, R92E_STBC_SETTING + 2, urtwn_read_1(sc,
	    R92E_STBC_SETTING + 2));
	urtwn_write_1(sc, R92C_ACLK_MON, 0);
	/* Save AGCCORE regs. */
	for (i = 0; i < sc->nrxchains; i++) {
		odfm0_agccore_regs[i] = urtwn_read_4(sc,
		    R92C_OFDM0_AGCCORE1(i));
	}
#endif
	/* Save BB regs. */
	reg0 = urtwn_bb_read(sc, R92C_OFDM0_TRXPATHENA);
	reg1 = urtwn_bb_read(sc, R92C_OFDM0_TRMUXPAR);
	reg2 = urtwn_bb_read(sc, R92C_FPGA0_RFIFACESW(1));

	/* Save adda regs to be restored when finished. */
	for (i = 0; i < __arraycount(addaReg); i++)
		addaBackup[i] = urtwn_bb_read(sc, addaReg[i]);
	/* Save mac regs. */
	iqkBackup[0] = urtwn_read_1(sc, R92C_TXPAUSE);
	iqkBackup[1] = urtwn_read_1(sc, R92C_BCN_CTRL);
	iqkBackup[2] = urtwn_read_1(sc, R92C_BCN_CTRL1);
	iqkBackup[3] = urtwn_read_4(sc, R92C_GPIO_MUXCFG);

#ifdef notyet
	ant_regs[0] = urtwn_read_4(sc, R92C_CONFIG_ANT_A);
	ant_regs[1] = urtwn_read_4(sc, R92C_CONFIG_ANT_B);

	rf_regs[0] = urtwn_read_4(sc, R92C_FPGA0_RFIFACESW(0));
	for (i = 0; i < sc->nrxchains; i++)
		rf_regs[i+1] = urtwn_read_4(sc, R92C_FPGA0_RFIFACEOE(i));
	reg4 = urtwn_read_4(sc, R92C_CCK0_AFESETTING);
#endif

	piMode = (urtwn_bb_read(sc, R92C_HSSI_PARAM1(0)) &
	    R92C_HSSI_PARAM1_PI);
	if (piMode == 0) {
		urtwn_bb_write(sc, R92C_HSSI_PARAM1(0),
		    urtwn_bb_read(sc, R92C_HSSI_PARAM1(0))|
		    R92C_HSSI_PARAM1_PI);
		urtwn_bb_write(sc, R92C_HSSI_PARAM1(1),
		    urtwn_bb_read(sc, R92C_HSSI_PARAM1(1))|
		    R92C_HSSI_PARAM1_PI);
	}

	attempt = 1;

next_attempt:

	/* Set mac regs for calibration. */
	for (i = 0; i < __arraycount(addaReg); i++) {
		urtwn_bb_write(sc, addaReg[i],
		    addaReg[__arraycount(addaReg) - 1]);
	}
	urtwn_write_2(sc, R92C_CCK0_AFESETTING, urtwn_read_2(sc,
	    R92C_CCK0_AFESETTING));
	urtwn_write_2(sc, R92C_OFDM0_TRXPATHENA, R92C_IQK_TRXPATHENA);
	urtwn_write_2(sc, R92C_OFDM0_TRMUXPAR, R92C_IQK_TRMUXPAR);
	urtwn_write_2(sc, R92C_FPGA0_RFIFACESW(1), R92C_IQK_RFIFACESW1);
	urtwn_write_4(sc, R92C_LSSI_PARAM(0), R92C_IQK_LSSI_PARAM);

	if (sc->ntxchains > 1)
		urtwn_bb_write(sc, R92C_LSSI_PARAM(1), R92C_IQK_LSSI_PARAM);

	urtwn_write_1(sc, R92C_TXPAUSE, (~R92C_TXPAUSE_BCN) & R92C_TXPAUSE_ALL);
	urtwn_write_1(sc, R92C_BCN_CTRL, (iqkBackup[1] &
	    ~R92C_BCN_CTRL_EN_BCN));
	urtwn_write_1(sc, R92C_BCN_CTRL1, (iqkBackup[2] &
	    ~R92C_BCN_CTRL_EN_BCN));

	urtwn_write_1(sc, R92C_GPIO_MUXCFG, (iqkBackup[3] &
	    ~R92C_GPIO_MUXCFG_ENBT));

	urtwn_bb_write(sc, R92C_CONFIG_ANT_A, R92C_IQK_CONFIG_ANT);

	if (sc->ntxchains > 1)
		urtwn_bb_write(sc, R92C_CONFIG_ANT_B, R92C_IQK_CONFIG_ANT);
	urtwn_bb_write(sc, R92C_FPGA0_IQK, R92C_FPGA0_IQK_SETTING);
	urtwn_bb_write(sc, R92C_TX_IQK, R92C_TX_IQK_SETTING);
	urtwn_bb_write(sc, R92C_RX_IQK, R92C_RX_IQK_SETTING);

	/* Restore BB regs. */
	urtwn_bb_write(sc, R92C_OFDM0_TRXPATHENA, reg0);
	urtwn_bb_write(sc, R92C_FPGA0_RFIFACESW(1), reg2);
	urtwn_bb_write(sc, R92C_OFDM0_TRMUXPAR, reg1);

	urtwn_bb_write(sc, R92C_FPGA0_IQK, 0x0);
	urtwn_bb_write(sc, R92C_LSSI_PARAM(0), R92C_IQK_LSSI_RESTORE);
	if (sc->nrxchains > 1)
		urtwn_bb_write(sc, R92C_LSSI_PARAM(1), R92C_IQK_LSSI_RESTORE);

	if (attempt-- > 0)
		goto next_attempt;

	/* Restore mode. */
	if (piMode == 0) {
		urtwn_bb_write(sc, R92C_HSSI_PARAM1(0),
		    urtwn_bb_read(sc, R92C_HSSI_PARAM1(0)) &
		    ~R92C_HSSI_PARAM1_PI);
		urtwn_bb_write(sc, R92C_HSSI_PARAM1(1),
		    urtwn_bb_read(sc, R92C_HSSI_PARAM1(1)) &
		    ~R92C_HSSI_PARAM1_PI);
	}

#ifdef notyet
	for (i = 0; i < sc->nrxchains; i++) {
		urtwn_write_4(sc, R92C_OFDM0_AGCCORE1(i),
		    odfm0_agccore_regs[i]);
	}
#endif

	/* Restore adda regs. */
	for (i = 0; i < __arraycount(addaReg); i++)
		urtwn_bb_write(sc, addaReg[i], addaBackup[i]);
	/* Restore mac regs. */
	urtwn_write_1(sc, R92C_TXPAUSE, iqkBackup[0]);
	urtwn_write_1(sc, R92C_BCN_CTRL, iqkBackup[1]);
	urtwn_write_1(sc, R92C_USTIME_TSF, iqkBackup[2]);
	urtwn_write_4(sc, R92C_GPIO_MUXCFG, iqkBackup[3]);

#ifdef notyet
	urtwn_write_4(sc, R92C_CONFIG_ANT_A, ant_regs[0]);
	urtwn_write_4(sc, R92C_CONFIG_ANT_B, ant_regs[1]);

	urtwn_write_4(sc, R92C_FPGA0_RFIFACESW(0), rf_regs[0]);
	for (i = 0; i < sc->nrxchains; i++)
		urtwn_write_4(sc, R92C_FPGA0_RFIFACEOE(i), rf_regs[i+1]);
	urtwn_write_4(sc, R92C_CCK0_AFESETTING, reg4);
#endif
}

static int
urtwn_r21a_iq_calib_fw_supported(struct urtwn_softc *sc)
{
	if (sc->fwver == 0x16)
		return (1);

	return (0);
}

static void
urtwn_r21a_iq_calib_fw(struct urtwn_softc *sc)
{
	struct urtwn_r21a_data *r21a_data = sc->sc_chip_priv.data;
	struct ieee80211_channel *c = usbwifi_ic(&sc->sc_uw)->ic_curchan;
	struct r21a_fw_cmd_iq_calib cmd;
	URTWNHIST_FUNC(); URTWNHIST_CALLED();
	DPRINTFN(DBG_FN, "Starting IQ calibration (FW)\n", 0, 0, 0, 0);

	cmd.chan = rtwn_chan2centieee(c);
	if (IEEE80211_IS_CHAN_5GHZ(c))
		cmd.band_bw = RTWN_CMD_IQ_BAND_5GHZ;
	else
		cmd.band_bw = RTWN_CMD_IQ_BAND_2GHZ;

	/* TODO: 80/160 MHz. */
	if (IEEE80211_IS_CHAN_HT40(c))
		cmd.band_bw |= RTWN_CMD_IQ_CHAN_WIDTH_40;
	else
		cmd.band_bw |= RTWN_CMD_IQ_CHAN_WIDTH_20;

	cmd.ext_5g_pa_lna = RTWN_CMD_IQ_EXT_PA_5G(r21a_data->ext_pa_5g);
	cmd.ext_5g_pa_lna |= RTWN_CMD_IQ_EXT_LNA_5G(r21a_data->ext_lna_5g);

	if (urtwn_r88e_fw_cmd(sc, R21A_CMD_IQ_CALIBRATE, &cmd, sizeof(cmd)) != 0) {
		aprint_error_dev(sc->sc_uw.uw_dev,
		    "error while sending IQ calibration command to FW!\n");
		return;
	}
}

static void
urtwn_r21a_iq_calib_sw(struct urtwn_softc *sc)
{
#define R21A_MAX_NRXCHAINS	2
	uint32_t saved_bb_vals[nitems(r21a_iq_bb_regs)];
	uint32_t saved_afe_vals[nitems(r21a_iq_afe_regs)];
	uint32_t saved_rf_vals[nitems(r21a_iq_rf_regs) * 2];
	int i, j;
	/* Save registers. */
	urtwn_bb_setbits(sc, R21A_TXAGC_TABLE_SELECT, 0x80000000, 0);
	for (i = 0; i < nitems(r21a_iq_bb_regs); i++)
		saved_bb_vals[i] = urtwn_bb_read(sc, r21a_iq_bb_regs[i]);

	for (i = 0; i < nitems(r21a_iq_afe_regs); i++)
		saved_afe_vals[i] = urtwn_bb_read(sc, r21a_iq_afe_regs[i]);

	urtwn_bb_setbits(sc, R21A_TXAGC_TABLE_SELECT, 0x80000000, 0);
	for (j = 0; j < sc->nrxchains; j++)
		for (i = 0; i < nitems(r21a_iq_rf_regs); i++)
			saved_rf_vals[j * nitems(r21a_iq_rf_regs) + i] = urtwn_rf_read(sc, j, r21a_iq_rf_regs[i]);

#ifdef notyet
	/* Select page C. */
	urtwn_bb_setbits(sc, R21A_TXAGC_TABLE_SELECT, 0x80000000, 0);
	urtwn_write_1(sc, R92C_TXPAUSE,
	    R92C_TX_QUEUE_AC | R92C_TX_QUEUE_MGT | R92C_TX_QUEUE_HIGH);
	/* BCN_CTRL & BCN_CTRL1 */
	urtwn_setbits_1(sc, R92C_BCN_CTRL(0), R92C_BCN_CTRL_EN_BCN, 0);
	urtwn_setbits_1(sc, R92C_BCN_CTRL(1), R92C_BCN_CTRL_EN_BCN, 0);
	/* Rx ant off */
	urtwn_write_1(sc, R21A_OFDMCCK_EN, 0);
	/* CCA off */
	urtwn_bb_setbits(sc, R21A_CCA_ON_SEC, 0x03, 0x0c);
	/* CCK RX Path off */
	urtwn_write_1(sc, R21A_CCK_RX_PATH + 3, 0x0f);
#endif

	urtwn_bb_setbits(sc, R21A_TXAGC_TABLE_SELECT, 0x80000000, 0);
	for (j = 0; j < sc->nrxchains; j++)
		for (i = 0; i < nitems(r21a_iq_rf_regs); i++)
			urtwn_rf_write(sc, j, r21a_iq_rf_regs[i], saved_rf_vals[j * nitems(r21a_iq_rf_regs) + i]);

	for (i = 0; i < nitems(r21a_iq_afe_regs); i++)
		urtwn_bb_write(sc, r21a_iq_afe_regs[i], saved_afe_vals[i]);

	/* Select page C1. */
	urtwn_bb_setbits(sc, R21A_TXAGC_TABLE_SELECT, 0, 0x80000000);

	urtwn_bb_write(sc, R21A_SLEEP_NAV(0), 0);
	urtwn_bb_write(sc, R21A_PMPD(0), 0);
	urtwn_bb_write(sc, 0xc88, 0);
	urtwn_bb_write(sc, 0xc8c, 0x3c000000);
	urtwn_bb_write(sc, 0xc90, 0x80);
	urtwn_bb_write(sc, 0xc94, 0);
	urtwn_bb_write(sc, 0xcc4, 0x20040000);
	urtwn_bb_write(sc, 0xcc8, 0x20000000);
	urtwn_bb_write(sc, 0xcb8, 0);

	urtwn_bb_setbits(sc, R21A_TXAGC_TABLE_SELECT, 0x80000000, 0);
	for (i = 0; i < nitems(r21a_iq_bb_regs); i++)
		urtwn_bb_write(sc, r21a_iq_bb_regs[i], saved_bb_vals[i]);

#undef R21A_MAX_NRXCHAINS
}

static void
urtwn_r21a_iq_calib(struct urtwn_softc *sc, bool inited)
{
	URTWNHIST_FUNC();
	URTWNHIST_CALLARGS("inited=%jd", inited, 0, 0, 0);
	if ((sc->sc_uw.uw_flags & URTWN_FLAG_FWREADY) &&
	    urtwn_r21a_iq_calib_fw_supported(sc))
		urtwn_r21a_iq_calib_fw(sc);
	else
		urtwn_r21a_iq_calib_sw(sc);
}

static void
urtwn_lc_calib(struct urtwn_softc *sc)
{
	uint32_t rf_ac[2];
	uint8_t txmode;
	size_t i;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	usbwifi_isowned_ic(&sc->sc_uw);

	txmode = urtwn_read_1(sc, R92C_OFDM1_LSTF + 3);
	if ((txmode & 0x70) != 0) {
		/* Disable all continuous Tx. */
		urtwn_write_1(sc, R92C_OFDM1_LSTF + 3, txmode & ~0x70);

		/* Set RF mode to standby mode. */
		for (i = 0; i < sc->nrxchains; i++) {
			rf_ac[i] = urtwn_rf_read(sc, i, R92C_RF_AC);
			urtwn_rf_write(sc, i, R92C_RF_AC,
			    RW(rf_ac[i], R92C_RF_AC_MODE,
				R92C_RF_AC_MODE_STANDBY));
		}
	} else {
		/* Block all Tx queues. */
		urtwn_write_1(sc, R92C_TXPAUSE, 0xff);
	}
	/* Start calibration. */
	urtwn_rf_write(sc, 0, R92C_RF_CHNLBW,
	    urtwn_rf_read(sc, 0, R92C_RF_CHNLBW) | R92C_RF_CHNLBW_LCSTART);

	/* Give calibration the time to complete. */
	urtwn_delay_ms(sc, 100);

	/* Restore configuration. */
	if ((txmode & 0x70) != 0) {
		/* Restore Tx mode. */
		urtwn_write_1(sc, R92C_OFDM1_LSTF + 3, txmode);
		/* Restore RF mode. */
		for (i = 0; i < sc->nrxchains; i++) {
			urtwn_rf_write(sc, i, R92C_RF_AC, rf_ac[i]);
		}
	} else {
		/* Unblock all Tx queues. */
		urtwn_write_1(sc, R92C_TXPAUSE, 0x00);
	}
}

static void
urtwn_temp_calib(struct urtwn_softc *sc)
{
	int temp, t_meter_reg;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	usbwifi_isowned_ic(&sc->sc_uw);

	if (!ISSET(sc->chip, URTWN_CHIP_92EU))
		t_meter_reg = R92C_RF_T_METER;
	else
		t_meter_reg = R92E_RF_T_METER;

	if (sc->thcal_state == 0) {
		/* Start measuring temperature. */
		DPRINTFN(DBG_RF, "start measuring temperature", 0, 0, 0, 0);
		urtwn_rf_write(sc, 0, t_meter_reg, 0x60);
		sc->thcal_state = 1;
		return;
	}
	sc->thcal_state = 0;

	/* Read measured temperature. */
	temp = urtwn_rf_read(sc, 0, R92C_RF_T_METER) & 0x1f;
	DPRINTFN(DBG_RF, "temperature=%jd", temp, 0, 0, 0);
	if (temp == 0)		/* Read failed, skip. */
		return;

	/*
	 * Redo LC calibration if temperature changed significantly since
	 * last calibration.
	 */
	if (sc->thcal_lctemp == 0) {
		/* First LC calibration is performed in urtwn_init(). */
		sc->thcal_lctemp = temp;
	} else if (abs(temp - sc->thcal_lctemp) > 1) {
		DPRINTFN(DBG_RF,
		    "LC calib triggered by temp: %jd -> %jd", sc->thcal_lctemp,
		    temp, 0, 0);
		urtwn_lc_calib(sc);
		/* Record temperature of last LC calibration. */
		sc->thcal_lctemp = temp;
	}
}

static void
urtwn_set_media_status(struct urtwn_softc *sc, int macid)
{
	sc->sc_set_media_status(sc, macid);
}


static void
urtwn_r21a_set_media_status(struct urtwn_softc *sc, int macid)
{
	struct r21a_fw_cmd_msrrpt status;
	int error;

	if (macid & RTWN_MACID_VALID)
		status.msrb0 = R21A_MSRRPT_B0_ASSOC;
	else
		status.msrb0 = R21A_MSRRPT_B0_DISASSOC;

	status.macid = 0;
	status.macid_end = 0;

	error = urtwn_r88e_fw_cmd(sc, R21A_CMD_MSR_RPT, &status, sizeof(status));
	if (error != 0)
		device_printf(sc->sc_uw.uw_dev, "cannot change media status!\n");
}


static int
urtwn_init(struct usbwifi *uw)
{
	struct urtwn_softc *sc = usbwifi_softc(uw);
	struct ieee80211com *ic = usbwifi_ic(&sc->sc_uw);
	uint32_t reg;
	int error;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	usbwifi_isowned_ic(&sc->sc_uw);

	mutex_enter(&sc->sc_task_mtx);
	/* Init host async commands ring. */
	sc->cmdq.cur = sc->cmdq.next = sc->cmdq.queued = 0;
	mutex_exit(&sc->sc_task_mtx);

	/* Init firmware commands ring. */
	sc->fwcur = 0;

	/* Power on adapter. */
	error = urtwn_power_on(sc);
	if (error != 0)
		goto fail;

	/* Initialize DMA. */
	error = urtwn_dma_init(sc);
	if (error != 0)
		goto fail;

	/* Drop incorrect TX (USB). */
	urtwn_setbits_1_shift(sc, R92C_TXDMA_OFFSET_CHK, 0, 0x200, 1);

	/* Set info size in Rx descriptors (in 64-bit words). */
	urtwn_write_1(sc, R92C_RX_DRVINFO_SZ, 4);

	/* Init interrupts. */
	if (ISSET(sc->chip, URTWN_CHIP_88E) ||
	     ISSET(sc->chip, URTWN_CHIP_92EU)) {
		urtwn_write_4(sc, R88E_HISR, 0xffffffff);
		urtwn_write_4(sc, R88E_HIMR, R88E_HIMR_CPWM | R88E_HIMR_CPWM2 |
		    R88E_HIMR_TBDER | R88E_HIMR_PSTIMEOUT);
		urtwn_write_4(sc, R88E_HIMRE, R88E_HIMRE_RXFOVW |
		    R88E_HIMRE_TXFOVW | R88E_HIMRE_RXERR | R88E_HIMRE_TXERR);
		if (ISSET(sc->chip, URTWN_CHIP_88E)) {
			urtwn_write_1(sc, R92C_USB_SPECIAL_OPTION,
			    urtwn_read_1(sc, R92C_USB_SPECIAL_OPTION) |
			      R92C_USB_SPECIAL_OPTION_INT_BULK_SEL);
		}
		if (ISSET(sc->chip, URTWN_CHIP_92EU))
			urtwn_write_1(sc, R92C_USB_HRPWM, 0);
	} else if(ISSET(sc->chip, URTWN_CHIP_21A)) {
		urtwn_write_4(sc, R88E_HIMR, 0);
		urtwn_write_4(sc, R88E_HIMRE, 0);
	} else {
		urtwn_write_4(sc, R92C_HISR, 0xffffffff);
		urtwn_write_4(sc, R92C_HIMR, 0xffffffff);
	}
	
	/* Set initial network type. */

	// reg = urtwn_read_4(sc, R92C_CR);
	// switch (ic->ic_opmode) {
	// case IEEE80211_M_STA:
	// default:
	// 	reg = RW(reg, R92C_CR_NETTYPE, R92C_CR_NETTYPE_INFRA);
	// 	break;

	// case IEEE80211_M_IBSS:
	// 	reg = RW(reg, R92C_CR_NETTYPE, R92C_CR_NETTYPE_ADHOC);
	// 	break;
	// }
	// urtwn_write_4(sc, R92C_CR, reg);
	urtwn_set_nettype0_msr(sc, R92C_MSR_NOLINK);
	
	/* Set MAC address. */
	urtwn_write_region(sc, R92C_MACID, ic->ic_macaddr, IEEE80211_ADDR_LEN);

	if (ISSET(sc->chip, URTWN_CHIP_21A)){
		urtwn_setbits_4(sc, R92C_RRSR, R92C_RRSR_RATE_BITMAP_M,
	    	R92C_RRSR_RATE_CCK_ONLY_1M);
	}
	else {
		/* Set response rate */
		reg = urtwn_read_4(sc, R92C_RRSR);
		reg = RW(reg, R92C_RRSR_RATE_BITMAP, R92C_RRSR_RATE_CCK_ONLY_1M);
		urtwn_write_4(sc, R92C_RRSR, reg);
		/* SIFS (used in NAV) */
		urtwn_write_2(sc, R92C_SPEC_SIFS,
    		SM(R92C_SPEC_SIFS_CCK, 0x10) | SM(R92C_SPEC_SIFS_OFDM, 0x10));

	}
	urtwn_rxfilter_init(sc);	// reset R92C_RCR

	/* Set short/long retry limits. */
	urtwn_write_2(sc, R92C_RL,
	    SM(R92C_RL_SRL, 0x30) | SM(R92C_RL_LRL, 0x30));

	/* Initialize EDCA parameters. */
	urtwn_edca_init(sc);

	/* Setup rate fallback. */
	if (!ISSET(sc->chip, URTWN_CHIP_88E) &&
	    !ISSET(sc->chip, URTWN_CHIP_92EU) &&
	    !ISSET(sc->chip, URTWN_CHIP_21A)) {
		urtwn_write_4(sc, R92C_DARFRC + 0, 0x00000000);
		urtwn_write_4(sc, R92C_DARFRC + 4, 0x10080404);
		urtwn_write_4(sc, R92C_RARFRC + 0, 0x04030201);
		urtwn_write_4(sc, R92C_RARFRC + 4, 0x08070605);
	}

	urtwn_setbits_1(sc, R92C_FWHW_TXQ_CTRL, 0,
	    R92C_FWHW_TXQ_CTRL_AMPDU_RTY_NEW);
	/* Set ACK timeout. */
	urtwn_write_1(sc, R92C_ACKTO, ISSET(sc->chip, URTWN_CHIP_21A) ? 0x80 : 0x40);

	/* Setup USB aggregation. */
	/* Tx */
	reg = urtwn_read_4(sc, R92C_TDECTRL);
	reg = RW(reg, R92C_TDECTRL_BLK_DESC_NUM, 6);
	urtwn_write_4(sc, R92C_TDECTRL, reg);
	if ISSET(sc->chip, URTWN_CHIP_21A) {
		urtwn_write_1(sc, R21A_DWBCN1_CTRL, 12);
	}

	/* Rx */
	if (ISSET(sc->chip, URTWN_CHIP_88E) ||
	    ISSET(sc->chip, URTWN_CHIP_92EU)){
		urtwn_write_1(sc, R92C_USB_SPECIAL_OPTION,
	    urtwn_read_1(sc, R92C_USB_SPECIAL_OPTION) &
	      ~R92C_USB_SPECIAL_OPTION_AGG_EN);
		urtwn_write_1(sc, R92C_RXDMA_AGG_PG_TH, 48);
		urtwn_write_1(sc, R92C_RXDMA_AGG_PG_TH + 1, 4);
	}
	else if ISSET(sc->chip, URTWN_CHIP_21A) 
		urtwn_write_2(sc, R92C_RXDMA_AGG_PG_TH, 4097);
	else{
		urtwn_write_1(sc, R92C_USB_SPECIAL_OPTION,
	    urtwn_read_1(sc, R92C_USB_SPECIAL_OPTION) &
	      ~R92C_USB_SPECIAL_OPTION_AGG_EN);
		urtwn_write_1(sc, R92C_RXDMA_AGG_PG_TH, 48);
		urtwn_write_1(sc, R92C_USB_DMA_AGG_TO, 4);
	}
	urtwn_setbits_1(sc, R92C_TRXDMA_CTRL, 0, R92C_TRXDMA_CTRL_RXDMA_AGG_EN);

	/* Initialize beacon parameters. */
	urtwn_write_1(sc, R92C_BCN_CTRL, 0x10);
	urtwn_write_1(sc, R92C_BCN_CTRL1, 0x10);
	urtwn_write_2(sc, R92C_TBTT_PROHIBIT, 0x6404);
	urtwn_write_1(sc, R92C_DRVERLYINT, R92C_DRVERLYINT_INIT_TIME);
	urtwn_write_1(sc, R92C_BCNDMATIM, R92C_BCNDMATIM_INIT_TIME);
	urtwn_write_2(sc, R92C_BCNTCFG, 0x660f);

	if ISSET(sc->chip, URTWN_CHIP_21A)
		urtwn_r21au_init_ampdu(sc);
	else if (!ISSET(sc->chip, URTWN_CHIP_88E) &&
	    !ISSET(sc->chip, URTWN_CHIP_92EU)) {
		/* Setup AMPDU aggregation. */
		urtwn_write_4(sc, R92C_AGGLEN_LMT, 0x99997631); /* MCS7~0 */
		urtwn_write_1(sc, R92C_AGGR_BREAK_TIME, 0x16);
		urtwn_write_2(sc, 0x4ca, 0x0708);

		urtwn_write_1(sc, R92C_BCN_MAX_ERR, 0xff);
		urtwn_write_1(sc, R92C_BCN_CTRL, R92C_BCN_CTRL_DIS_TSF_UDT0);
	}

	/* Load 8051 microcode. */
	error = urtwn_load_firmware(sc);
	if (error != 0)
		goto fail;
	SET(sc->sc_uw.uw_flags, URTWN_FLAG_FWREADY);

	/* Initialize MAC/BB/RF blocks. */
	/*
	 * XXX: urtwn_mac_init() sets R92C_RCR[0:15] = R92C_RCR_APM |
	 * R92C_RCR_AM | R92C_RCR_AB | R92C_RCR_AICV | R92C_RCR_AMF.
	 * XXX: This setting should be removed from rtl8192cu_mac[].
	 */
	urtwn_mac_init(sc);		// sets R92C_RCR[0:15]

	if (ISSET(sc->chip, URTWN_CHIP_21A)) {
		urtwn_r21a_bb_init(sc);
		urtwn_r21a_rf_init(sc);
	} else{
		urtwn_bb_init(sc);
		urtwn_rf_init(sc);
	}

	/* Set default channel. */
	if(ISSET(sc->chip, URTWN_CHIP_21A))
		urtwn_r21a_set_chan(sc, ic->ic_curchan);
	else
		urtwn_set_chan(sc, ic->ic_curchan);
	
	if (ISSET(sc->chip, URTWN_CHIP_88E) ||
	    ISSET(sc->chip, URTWN_CHIP_92EU)||
	    ISSET(sc->chip, URTWN_CHIP_21A)) {
		urtwn_setbits_1(sc, R92C_CR, 0, R92C_CR_MACTXEN | R92C_CR_MACRXEN);
	}

	/* Turn CCK and OFDM blocks on. */
	if (!ISSET(sc->chip, URTWN_CHIP_21A)){
		reg = urtwn_bb_read(sc, R92C_FPGA0_RFMOD);
		reg |= R92C_RFMOD_CCK_EN;
		urtwn_bb_write(sc, R92C_FPGA0_RFMOD, reg);
		reg = urtwn_bb_read(sc, R92C_FPGA0_RFMOD);
		reg |= R92C_RFMOD_OFDM_EN;
		urtwn_bb_write(sc, R92C_FPGA0_RFMOD, reg);
	}

	/* Clear per-station keys table. */
	if (ISSET(sc->chip, URTWN_CHIP_21A))
		urtwn_write_4(sc, R92C_CAMCMD, R92C_CAMCMD_POLLING | R92C_CAMCMD_CLR);
	else
		urtwn_cam_init(sc);

	urtwn_write_2(sc, R92C_SECCFG,
	    R92C_SECCFG_TXENC_ENA | R92C_SECCFG_RXDEC_ENA |
	    R92C_SECCFG_MC_SRCH_DIS);



	/* Perform LO and IQ calibrations. */
	if (ISSET(sc->chip, URTWN_CHIP_21A))
		urtwn_r21a_iq_calib(sc, sc->iqk_inited);
	else
		urtwn_iq_calib(sc, sc->iqk_inited);
	sc->iqk_inited = true;

	/* Perform LC calibration. */
	if (!ISSET(sc->chip, URTWN_CHIP_21A)) /* XXX Not used in 8821a */
		urtwn_lc_calib(sc);

	if (!ISSET(sc->chip, URTWN_CHIP_88E) &&
	    !ISSET(sc->chip, URTWN_CHIP_92EU) && 
	    !ISSET(sc->chip, URTWN_CHIP_21A)) {
		/* Fix USB interference issue. */
		urtwn_write_1(sc, 0xfe40, 0xe0);
		urtwn_write_1(sc, 0xfe41, 0x8d);
		urtwn_write_1(sc, 0xfe42, 0x80);
		urtwn_write_4(sc, 0x20c, 0xfd0320);

		urtwn_pa_bias_init(sc);
	}

	/* Initialize antenna selection. */
	if (ISSET(sc->chip, URTWN_CHIP_21A)){
		urtwn_write_1(sc, R92C_LEDCFG2, 0x82);
		urtwn_bb_setbits(sc, R92C_FPGA0_RFPARAM(0), 0, 0x2000);
	} else if (!(sc->chip & (URTWN_CHIP_92C | URTWN_CHIP_92C_1T2R)) ||
	    !(sc->chip & URTWN_CHIP_92EU)) {
		/* 1T1R */
		urtwn_bb_write(sc, R92C_FPGA0_RFPARAM(0),
		    urtwn_bb_read(sc, R92C_FPGA0_RFPARAM(0)) | __BIT(13));
	}

	/* Enable hardware sequence numbering. */
	urtwn_write_1(sc, R92C_HWSEQ_CTRL, 0xff);

	/* Disable BAR. */
	urtwn_write_4(sc, R92C_BAR_MODE_CTRL, 0x0201ffff);

	/* NAV limit. */
	urtwn_write_1(sc, R92C_NAV_UPPER, 0);

	/* Initialize GPIO setting. */
	urtwn_write_1(sc, R92C_GPIO_MUXCFG,
	    urtwn_read_1(sc, R92C_GPIO_MUXCFG) & ~R92C_GPIO_MUXCFG_ENBT);

	/* Initialize MRR. */
	urtwn_mrr_init(sc);

	/* Fix for lower temperature. */
	if (!ISSET(sc->chip, URTWN_CHIP_88E) &&
	    !ISSET(sc->chip, URTWN_CHIP_92EU) && 
	    !ISSET(sc->chip, URTWN_CHIP_21A))
		urtwn_write_1(sc, 0x15, 0xe9);

	if (ISSET(sc->chip, URTWN_CHIP_21A)) {
		/* Setup RTS BW (equal to data BW). */
		urtwn_setbits_1(sc, R92C_QUEUE_CTRL, 0x08, 0);

		urtwn_write_1(sc, R21A_EARLY_MODE_CONTROL + 3, 0x01);

		/* Reset USB mode switch setting. */
		urtwn_write_1(sc, R21A_SDIO_CTRL, 0);
		urtwn_write_1(sc, R92C_ACLK_MON, 0);

		urtwn_write_1(sc, R92C_USB_HRPWM, 0);
	}
	power_control(sc, false);
#if 0
	// XXXX can't do that here while core is locked?
	urtwn_wait_async(sc);
#endif

	return 0;

 fail:
	urtwn_stop(&sc->sc_uw);
	return error;
}

static void __noinline
urtwn_stop(struct usbwifi *uw)
{
	struct urtwn_softc *sc = usbwifi_softc(uw);
	uint32_t reg;
	const bool disable = true;

	URTWNHIST_FUNC(); URTWNHIST_CALLED();

	sc->tx_timer = 0;

	callout_stop(&sc->sc_calib_to);

	if (ISSET(sc->chip, URTWN_CHIP_88E) ||
	    ISSET(sc->chip, URTWN_CHIP_92EU)) {
		DPRINTFN(DBG_INIT, "not powering down, 88E: %jd 92EU: %jd",
		    ISSET(sc->chip, URTWN_CHIP_88E),
		    ISSET(sc->chip, URTWN_CHIP_92EU), 0, 0);
		return;
	}
	if (ISSET(sc->chip, URTWN_CHIP_21A))
		return urtwn_r21a_power_off(sc);
	/*
	 * RF Off Sequence
	 */
	/* Pause MAC TX queue */
	urtwn_write_1(sc, R92C_TXPAUSE, 0xFF);

	/* Disable RF */
	urtwn_rf_write(sc, 0, 0, 0);

	urtwn_write_1(sc, R92C_APSD_CTRL, R92C_APSD_CTRL_OFF);

	/* Reset BB state machine */
	urtwn_write_1(sc, R92C_SYS_FUNC_EN,
	    R92C_SYS_FUNC_EN_USBD |
	    R92C_SYS_FUNC_EN_USBA |
	    R92C_SYS_FUNC_EN_BB_GLB_RST);
	urtwn_write_1(sc, R92C_SYS_FUNC_EN,
	    R92C_SYS_FUNC_EN_USBD | R92C_SYS_FUNC_EN_USBA);

	/*
	 * Reset digital sequence
	 */
	if (urtwn_read_1(sc, R92C_MCUFWDL) & R92C_MCUFWDL_RDY) {
		/* Reset MCU ready status */
		urtwn_write_1(sc, R92C_MCUFWDL, 0);
		/* If firmware in ram code, do reset */
		if (ISSET(sc->sc_uw.uw_flags, URTWN_FLAG_FWREADY)) {
			if (ISSET(sc->chip, URTWN_CHIP_88E) ||
			    ISSET(sc->chip, URTWN_CHIP_92EU))
				urtwn_r88e_fw_reset(sc);
			else
				urtwn_fw_reset(sc);
			CLR(sc->sc_uw.uw_flags, URTWN_FLAG_FWREADY);
		}
	}

	/* Reset MAC and Enable 8051 */
	urtwn_write_1(sc, R92C_SYS_FUNC_EN + 1, 0x54);

	/* Reset MCU ready status */
	urtwn_write_1(sc, R92C_MCUFWDL, 0);

	if (disable) {
		/* Disable MAC clock */
		urtwn_write_2(sc, R92C_SYS_CLKR, 0x70A3);
		/* Disable AFE PLL */
		urtwn_write_1(sc, R92C_AFE_PLL_CTRL, 0x80);
		/* Gated AFE DIG_CLOCK */
		urtwn_write_2(sc, R92C_AFE_XTAL_CTRL, 0x880F);
		/* Isolated digital to PON */
		urtwn_write_1(sc, R92C_SYS_ISO_CTRL, 0xF9);
	}

	/*
	 * Pull GPIO PIN to balance level and LED control
	 */
	/* 1. Disable GPIO[7:0] */
	urtwn_write_2(sc, R92C_GPIO_PIN_CTRL + 2, 0x0000);

	reg = urtwn_read_4(sc, R92C_GPIO_PIN_CTRL) & ~0x0000ff00;
	reg |= ((reg << 8) & 0x0000ff00) | 0x00ff0000;
	urtwn_write_4(sc, R92C_GPIO_PIN_CTRL, reg);

	/* Disable GPIO[10:8] */
	urtwn_write_1(sc, R92C_GPIO_MUXCFG + 3, 0x00);

	reg = urtwn_read_2(sc, R92C_GPIO_MUXCFG + 2) & ~0x00f0;
	reg |= (((reg & 0x000f) << 4) | 0x0780);
	urtwn_write_2(sc, R92C_GPIO_MUXCFG + 2, reg);

	/* Disable LED0 & 1 */
	urtwn_write_2(sc, R92C_LEDCFG0, 0x8080);

	/*
	 * Reset digital sequence
	 */
	if (disable) {
		/* Disable ELDR clock */
		urtwn_write_2(sc, R92C_SYS_CLKR, 0x70A3);
		/* Isolated ELDR to PON */
		urtwn_write_1(sc, R92C_SYS_ISO_CTRL + 1, 0x82);
	}

	/*
	 * Disable analog sequence
	 */
	if (disable) {
		/* Disable A15 power */
		urtwn_write_1(sc, R92C_LDOA15_CTRL, 0x04);
		/* Disable digital core power */
		urtwn_write_1(sc, R92C_LDOV12D_CTRL,
		    urtwn_read_1(sc, R92C_LDOV12D_CTRL) &
		      ~R92C_LDOV12D_CTRL_LDV12_EN);
	}

	/* Enter PFM mode */
	urtwn_write_1(sc, R92C_SPS0_CTRL, 0x23);

	/* Set USB suspend */
	urtwn_write_2(sc, R92C_APS_FSMCO,
	    R92C_APS_FSMCO_APDM_HOST |
	    R92C_APS_FSMCO_AFSM_HSUS |
	    R92C_APS_FSMCO_PFM_ALDN);

	urtwn_write_1(sc, R92C_RSV_CTRL, 0x0E);
}

static void
urtwn_delay_ms(struct urtwn_softc *sc, int ms)
{

	if (sc->sc_uw.uw_udev == NULL)
		DELAY(ms * 1000);
	else
		usbd_delay_ms(sc->sc_uw.uw_udev, ms);
}

MODULE(MODULE_CLASS_DRIVER, if_urtwn, NULL);

#ifdef _MODULE
#include "ioconf.c"
#endif

static int
if_urtwn_modcmd(modcmd_t cmd, void *aux)
{
	int error = 0;

	switch (cmd) {
	case MODULE_CMD_INIT:
#ifdef _MODULE
		error = config_init_component(cfdriver_ioconf_urtwn,
		    cfattach_ioconf_urtwn, cfdata_ioconf_urtwn);
#endif
		return error;
	case MODULE_CMD_FINI:
#ifdef _MODULE
		error = config_fini_component(cfdriver_ioconf_urtwn,
		    cfattach_ioconf_urtwn, cfdata_ioconf_urtwn);
#endif
		return error;
	default:
		return ENOTTY;
	}
}
