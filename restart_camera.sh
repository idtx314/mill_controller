#!/bin/bash

# # let's unplug and re-plug the camera drivers
# sudo rmmod uvcvideo
# sudo rmmod videobuf2_vmalloc
# sudo rmmod videobuf2_v4l2
# sudo rmmod videobuf2_core
# sudo rmmod videodev


# sudo modprobe uvcvideo
# sudo modprobe videobuf2_vmalloc
# sudo modprobe videobuf2_v4l2
# sudo modprobe videobuf2_core
# sudo modprobe videodev

# restart the USB devices
# echo -n "0000:00:14.0" | sudo tee /sys/bus/pci/drivers/xhci_hcd/unbind
echo -n "1-7:1.0" | sudo tee /sys/bus/usb/drivers/uvcvideo/unbind
echo "sleeping"
sleep 1
# let's try reloading udev
sudo service udev reload
echo -n "1-7:1.0" | sudo tee /sys/bus/usb/drivers/uvcvideo/bind
sleep 1
