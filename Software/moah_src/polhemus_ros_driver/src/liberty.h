/*

 Communication library for a Polhemus Liberty v2 (tm) Motion tracker
 Copyright (C) 2008 Jonathan Kleinehellefort <kleinehe@cs.tum.edu>
     Intelligent Autonomous Systems Lab,
     Lehrstuhl fuer Informatik 9, Technische Universitaet Muenchen

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA

*/


#ifndef LIBERTY_H
#define LIBERTY_H

#include <usb.h>

typedef struct buffert_t {
    char buf[8192];
    int fill;
} buffer_t;

void init_buffer(buffer_t *b);

/* set up usb interface and configuration, send initial magic and reset */
int liberty_init(usb_dev_handle *handle);
/* send a command */
int liberty_send(usb_dev_handle *handle, char *cmd);
/* receive a packet of size bytes */
int liberty_receive(usb_dev_handle *handle, buffer_t *b, void *buf, int size);
/* disable previous `c' commands and empty input buffer */
void liberty_reset(usb_dev_handle *handle);

/* read until the device doesn't send anything else */
void liberty_clear_input(usb_dev_handle *handle);
void liberty_ignore_input(usb_dev_handle *handle, int count);

int liberty_read(usb_dev_handle *handle, void *buf, int size, int timeout);

#endif
