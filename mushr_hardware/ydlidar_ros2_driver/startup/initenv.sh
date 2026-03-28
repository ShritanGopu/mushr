#!/bin/bash
set -e

# This lidar is addressed through the kernel-managed /dev/serial/by-id path,
# so no custom udev alias is required here.
echo "No custom YDLIDAR udev rule required. Use /dev/serial/by-id/... in the driver params."
