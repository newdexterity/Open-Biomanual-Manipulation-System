
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

//#define DEBUG 1

#include <string.h>
#include <usb.h>

#include "liberty.h"

#ifdef DEBUG
#include <stdio.h>
#define warn(as...) { fprintf(stderr, "%s:%d: ", __FILE__, __LINE__); \
    fprintf(stderr, as); }
#else
#define warn(as...)
#endif

#define IN 0x88
#define OUT 0x4
#define INTERFACE 0
#define CONFIGURATION 1
#define TIMEOUT 500

void init_buffer(buffer_t *b)
{
    b->fill = 0;
}


static int liberty_write(usb_dev_handle *handle, char *buf, int size, int timeout)
{
    return usb_bulk_write(handle, OUT, buf, size, timeout);
}

int liberty_init(usb_dev_handle *handle)
{
    if (usb_set_configuration(handle, CONFIGURATION) != 0)
        warn("could not set usb configuration to %d\n", CONFIGURATION);

    if (usb_claim_interface(handle, INTERFACE) != 0) {
        warn("could not claim usb interface %d\n", INTERFACE);
        return 0;
    }

    /*
    static char magic[] = { '*', '*', 0xff, 0x16, 0, 0, 0, 0 };
    if (liberty_write(handle, magic, sizeof(magic), 0) != sizeof(magic)) {
        warn("usb bulk write failed\n");
        return 0;
    }*/
    liberty_reset(handle);
    return 1;
}

int liberty_send(usb_dev_handle *handle, char *cmd)
{
    int size = strlen(cmd);
    if (liberty_write(handle, cmd, size, TIMEOUT) != size) {
        warn("sending cmd `%s' to device failed\n");
        return 0;
    }
    return 1;
}

int liberty_read(usb_dev_handle *handle, void *buf, int size,
                        int timeout)
{
    return usb_bulk_read(handle, IN, (char*)buf, size, timeout);
}

void liberty_clear_input(usb_dev_handle *handle)
{
    static char buf[1024];

    while( liberty_read(handle, buf, sizeof(buf), TIMEOUT) > 0); 
}

void liberty_ignore_input(usb_dev_handle *handle, int count)
{
    static char buf[2048];
    int i;
    for (i = 0; i < count; ++i)
        liberty_read(handle, buf, sizeof(buf), TIMEOUT);
}

int liberty_receive(usb_dev_handle *handle, buffer_t *b, void *buf, int size)
{
    //printf("liberty_receive: %u\n",size);
    while (1) {

        while (b->fill < size) {
            int n_read = liberty_read(handle, b->buf + b->fill,
                                  sizeof(b->buf) - b->fill, TIMEOUT);
            warn("read %d\n", n_read);
            if (n_read < 0) {
                warn("error while reading from device (%d)", n_read);
                return 0;
            }
            b->fill += n_read;
            // Clear input if reading too much
            if (n_read > size) {
                liberty_ignore_input(handle, 3);
            }
        }

        #ifdef DEBUG
        fprintf(stderr, "- %c%c\n", b->buf[0], b->buf[1]);
        #endif

        if (b->buf[0] == 'L' && b->buf[1] == 'Y') {
            memcpy(buf, b->buf, size);
            memmove(b->buf, b->buf + size, b->fill - size);
            b->fill -= size;
            return 1;
        } else {
            warn("got corrupted data\n");
            b->fill = 0;
        }
    }
}

/** this resets previous `c' commands and puts the device in binary mode
 *
 *  beware: the device can be misconfigured in other ways too, though this will
 *  usually work
 */
void liberty_reset(usb_dev_handle *handle)
{
    // reset c, this may produce "invalid command" answers
    liberty_send(handle, (char *)"\rp\r");
    // remove everything from input
    liberty_clear_input(handle);
}
