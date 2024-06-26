/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef STM32MEM_H
#define STM32MEM_H

#ifdef WIN32
#   include <lusb0_usb.h>
#else
#   include <usb.h>
#endif

int stm32_mem_erase(usb_dev_handle *dev, uint16_t iface, uint32_t addr);
int stm32_mem_write(usb_dev_handle *dev, uint16_t iface, void *data, int size, uint32_t addr);
int stm32_mem_manifest(usb_dev_handle *dev, uint16_t iface);

#endif /* STM32MEM_H */
