/*

  Communication library for a Polhemus Liberty v2 (tm) Motion tracker
  Copyright (C) 2008 Jonathan Kleinehellefort and Alexis Maldonado
  Intelligent Autonomous Systems Lab,
  Lehrstuhl fuer Informatik 9, Technische Universitaet Muenchen
  <kleinehe@cs.tum.edu>  <maldonad@cs.tum.edu>

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

/*
  This file is a modified version of polhemus_ros_node.cpp by Kleinehellefort and Maldonado.
  Modified by C. E. Mower, 2017.
*/

#include <usb.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <signal.h>

#include <sys/time.h>
#include <time.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include "liberty.h"
#include "protocol.h"

/* Vendor 0x0f44 -> Polhemus */
#define VENDOR 0xf44

/* Product 0xff20 -> Liberty v2 Motion Tracker (with USB 2.0 using an EzUSB fx2
   chip)  after loading the firmware (it has ProductId 0xff21 before) */
#define PRODUCT 0xff20

/* make control character out of ordinary character */
#define control(c) ((c) & 0x1f)

static int count_bits(uint16_t v) {
  int c;
  for (c = 0; v; c++)
    {
      v &= v - 1; // clear the least significant bit set
    }
  return c;
}

/* main loop running? */
static int go_on;

static void signal_handler(int s) {
  switch (s) {
  case SIGINT:
    go_on = 0;
    break;
  }
}

static void print_hex(FILE *stream, const char *buf, size_t size) {
  const char *c;
  for (c = buf; c != buf + size; ++c)
    fprintf(stream, "%02x:%c ", (unsigned char) *c, isprint(*c) ? *c : '.');
  fprintf(stream, "\n");
}

static void print_ascii(FILE *stream, const char *buf, size_t size) {
  const char *c;
  for (c = buf; c != buf + size; ++c)
    if (isprint(*c)) {
      fprintf(stream, "%c", *c);
    }
  fprintf(stream, "\n");
}

static struct usb_device *find_device_by_id(uint16_t vendor, uint16_t product) {
  struct usb_bus *bus;

  usb_find_busses();
  usb_find_devices();

  for (bus = usb_get_busses(); bus; bus = bus->next) {
    struct usb_device *dev;
    for (dev = bus->devices; dev; dev = dev->next) {
      if (dev->descriptor.idVendor == vendor && dev->descriptor.idProduct == product)
	return dev;
    }
  }
  return NULL;
}

static int request_num_of_stations(usb_dev_handle *handle, buffer_t *b) {
  static char cmd[] = { control('u'), '0', '\r', '\0' };
  active_station_state_response_t resp;
  liberty_send(handle, cmd);
  liberty_receive(handle, b, &resp, sizeof(resp));

  if (resp.head.init_cmd == 21) {
    return count_bits(resp.detected & resp.active);
  }
  else {
    return 0;
  }
}

/* sets the zenith of the hemisphere in direction of vector (x, y, z) */
static void set_hemisphere(usb_dev_handle *handle, int x, int y, int z) {
  char cmd[32];
  snprintf(cmd, sizeof(cmd), "h*,%d,%d,%d\r", x, y, z);
  liberty_send(handle, cmd);
}

int main(int argc, char** argv) {

  int i, nstations;
  double x_hs, y_hs, z_hs;
  struct usb_device *dev;
  usb_dev_handle *handle;
  buffer_t buf;
  struct timeval tv;

  // Setup polhemus
  usb_init();

  dev = find_device_by_id(VENDOR, PRODUCT);
  if (!dev) {
    fprintf(stderr, "Could not find the Polhemus Liberty device.\n");
    abort();
  }

  handle = usb_open(dev);
  if (!handle) {
    fprintf(stderr, "Could not get a handle to the Polhemus Liberty device.\n");
    abort();
  }

  if (!liberty_init(handle)) {
    fprintf(stderr, "Could not initialize the Polhemus Liberty device.\n");
    usb_close(handle);
    return 1;
  }

  init_buffer(&buf);
  liberty_send(handle, (char *)"f1\r"); // activate binary mode

  nstations = request_num_of_stations(handle, &buf);
  fprintf(stderr, "Found %d stations.\n\n", nstations);
  if (nstations == 0) {    
    usb_close(handle);
    return 1;
  }

  // Setup ros
  ros::init(argc, argv, "polhemus_tf_broadcaster");
  ros::NodeHandle nh("/polhemus_tf_broadcaster");


  /* define which information to get per sensor (called a station
     by polhemus)

     o* applies to all stations

     if this is changed, the station_t struct or (***) below has to be edited accordingly 
  */
  liberty_send(handle, (char *)"O*,8,9,11,3,7\r"); // station_t: quaternions

  /* set output hemisphere -- this will produce a response which we're
     ignoring 
  */
  nh.getParam("/x_hs", x_hs);
  nh.getParam("/y_hs", y_hs);
  nh.getParam("/z_hs", z_hs);
  set_hemisphere(handle, x_hs, y_hs, z_hs);

  /* switch output to centimeters */
  //liberty_send(handle, "u1\r");
  liberty_clear_input(handle); //right now, we just ignore the answer

  // (***)
  station_t *stations = (station_t*) (malloc(sizeof(station_t) * nstations));
  //std::vector<station_t> stations(sizeof(station_t)*nstations);
  if (!stations)
    abort();

  /* set up signal handler to catch the interrupt signal */
  signal(SIGINT, signal_handler);

  go_on = 1;

  /* enable continuous mode (get data points continously) */
  liberty_send(handle, (char *)"c\r");

  gettimeofday(&tv, NULL);
  printf("Begin time: %d.%06d\n", (unsigned int) (tv.tv_sec), (unsigned int) (tv.tv_usec));

  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  ros::Rate r(250);

  // Start main loop
  while(ros::ok()) {
    if (go_on == 0)
      break;

    // Update polhemus
    // (***)
    if (!liberty_receive(handle, &buf, stations, sizeof(station_t) * nstations)) {
      fprintf(stderr, "Receive failed.\n");
      return 2;
    }

    /* Note: timestamp is the time in ms after the first read to the
       system after turning it on
       at 240Hz, the time between data sample is 1/240 = .00416_
       seconds.
       The framecount is a more exact way of finding out the time:
       timestamp = framecount*1000/240 (rounded down to an int)* 
    */
    
    // Header info - acquired at same time = same timestamp
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "polhemus_base";

    for (i=0; i < nstations; i++) {
      // Header info
      transformStamped.child_frame_id = "polhemus_station_" + std::to_string(i+1);
	
      // Set translation (conversion: inches -> meters)
      transformStamped.transform.translation.x = 0.0254*stations[i].x;
      transformStamped.transform.translation.y = 0.0254*stations[i].y;
      transformStamped.transform.translation.z = 0.0254*stations[i].z;

      // Set rotation
      transformStamped.transform.rotation.w = stations[i].quaternion[0];
      transformStamped.transform.rotation.x = stations[i].quaternion[1];
      transformStamped.transform.rotation.y = stations[i].quaternion[2];
      transformStamped.transform.rotation.z = stations[i].quaternion[3];

      // Broadcast frame
      br.sendTransform(transformStamped);
    }

    r.sleep();
  }

  // Shutdown
  liberty_send(handle, (char *)"p"); // stop continuous mode
  usb_close(handle);
  delete[] stations;
  
  return 0;
}
