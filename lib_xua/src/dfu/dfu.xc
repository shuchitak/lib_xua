// Copyright 2011-2026 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#include "xua.h"
#if (XUA_DFU_EN== 1)
#include <xs1.h>
#include <platform.h>

#if XUA_USB_EN
#include "xud_device.h"
#include "dfu_types.h"
#include "flash_interface.h"
#include "dfu_interface.h"


#endif /* XUA_USB_EN */

#endif
