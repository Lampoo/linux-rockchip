/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _HALMAC_API_8822B_SDIO_H_
#define _HALMAC_API_8822B_SDIO_H_

#include "../../halmac_2_platform.h"
#include "../../halmac_type.h"

HALMAC_RET_STATUS
halmac_mac_power_switch_8822b_sdio(
	IN PHALMAC_ADAPTER pHalmac_adapter,
	IN HALMAC_MAC_POWER halmac_power
);

#if 0
HALMAC_RET_STATUS
halmac_tx_allowed_sdio_8822b(
	IN PHALMAC_ADAPTER pHalmac_adapter,
	IN u8 *pHalmac_buf,
	IN u32 halmac_size
);
#endif

HALMAC_RET_STATUS
halmac_phy_cfg_8822b_sdio(
	IN PHALMAC_ADAPTER pHalmac_adapter,
	IN HALMAC_INTF_PHY_PLATFORM platform
);

#endif/* _HALMAC_API_8822B_SDIO_H_ */
