/*
 *  arch/arm/plat-aspeed/include/plat/ast-scu.h
 *
 *  Copyright (C) 2012-2020  ASPEED Technology Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#if defined(CONFIG_IRMP) || defined(CONFIG_REMOTEFX) || defined(CONFIG_PCEXT)
#include "ast-bmc-scu.h"
#elif defined(CONFIG_RT360_CAM)
#include "ast-cam-scu.h"
#else
#err "no define for ast-scu.h"
#endif
