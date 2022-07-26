# jetsonDynamixel

The core concept is to utilize the full-duplex UART of Jetson TX2 to communicate in half-duplex with Dynamixel servos through 74LS241, as illustrated in [this beautiful post](https://www.instructables.com/id/How-to-drive-Dynamixel-AX-12A-servos-with-a-Raspbe/).

This library is done starting from [this library for Raspberry](https://github.com/thiagohersan/memememe/tree/master/Python/ax12) and [this python GPIO handler](https://github.com/vitiral/gpio).

## Device hardware
I've used a Jetson TX2 with an [Orbitty Carrier](http://connecttech.com/product/orbitty-carrier-for-nvidia-jetson-tx2-tx1/), where the UART0 is handled by the kernel with the namespace `/dev/ttyTHS2` (`ttyS0` is already used by the system for... boh, something, so it's easier to use this one!). Maybe you've to change something if you use other platforms.

## Prerequisites
First of all install [GPIO handler](https://github.com/vitiral/gpio), cloning that repo and executing `python setup.py build && sudo python setup.py install`. Remeber that anything you do with GPIOs need superuser privileges.

## Further improvements
It's not so unusual that some reply from the motors went lost, and I suppose it's related to I/O timings; probably a little improvement could be done implementing [this optimizations in GPIO library](https://github.com/vitiral/gpio/issues/11).