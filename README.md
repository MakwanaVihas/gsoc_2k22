
# GSoC '22

Project title :- "Introduce a new Wi-Fi driver"

NetBSD kernel supports a range of different wireless devices. 
But it lacks a driver for the devices 
based on RTL8821AU chipset 
(e.g., TP-Link Archer T2U Nano, product id 0x011e) and devices based on MediaTek MT7601U chipset. 
Driver for that chipset is available in FreeBSD, named “rtwn”.
I ported that driver to NetBSD and converted it to the new wifi stack.
Also introduced a new driver, "mtw", for MediaTek based devices.
It is ported from OpenBSD's mtw driver.


## 🔗 Links
[New wifi stack](https://wiki.netbsd.org/Converting_drivers_to_the_new_wifi_stack/)\
[OpenBSD's mtw driver](https://man.openbsd.org/mtw.4)\
[FreeBSD's rtwn driver](https://www.freebsd.org/cgi/man.cgi?query=rtwn&sektion=4&manpath=FreeBSD+13.1-RELEASE+and+Ports)