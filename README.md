# OpenVNAVI

![](https://github.com/davidanton/OpenVNAVI/blob/master/doc/img/logo.png)

**Bachelor Thesis at the Media Computing Group, Computer Science Department @ RWTH Aachen University (Germany)**
**Author:** [David Antón Sánchez](https://hci.rwth-aachen.de/sanchez)
**Project Website:** https://hci.rwth-aachen.de/openvnavi

---
#Wiki
- [Home](https://github.com/davidanton/OpenVNAVI/wiki)
- [Setting Up the Raspberry Pi 2](https://github.com/davidanton/OpenVNAVI/wiki/Setting-Up-the-Raspberry-Pi-2)
---

###System Description

OpenVNAVI is a vest equipped with a depth sensor and array of vibration motor units that allow people with visual impairment to avoid obstacles in the environment.

The ASUS Xtion PRO LIVE depth sensor, positioned onto the user’s chest scans the environment as the user moves. From the video feed of the depth sensor a frame is captured and then processed by the Raspberry Pi 2. Each frame is downsampled from 640x480 to 16x8 and each pixel is then mapped to a vibration motor unit forming an array positioned onto the user’s belly.

The grayscale value of each pixel on the lower resolution frame is assigned to a PWM voltage value generated by the Raspberry Pi 2 via PWM drivers that will drive each vibration motor obtaining a vibration amplitude value as a function of the proximity of an object.

With this method the vibration motor unit array is able to represent a vibratory image onto the user’s belly to help create a mental representation of the obstacles in the scene.

---

###Pictures

![](https://github.com/davidanton/OpenVNAVI/blob/master/doc/img/overview.png)

![](https://github.com/davidanton/OpenVNAVI/blob/master/doc/img/array1.JPG)

![](https://github.com/davidanton/OpenVNAVI/blob/master/doc/img/vest_front.png)

![](https://github.com/davidanton/OpenVNAVI/blob/master/doc/img/specimen.JPG)

![](https://github.com/davidanton/OpenVNAVI/blob/master/doc/img/driver_unit.png)

![](https://github.com/davidanton/OpenVNAVI/blob/master/doc/img/pcb.jpg)

![](https://github.com/davidanton/OpenVNAVI/blob/master/doc/img/enclosure01.JPG)
