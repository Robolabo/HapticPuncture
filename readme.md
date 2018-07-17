

HapticPuncture - The Open Source Haptic Puncture Simulator
----------------------------------------------------------


Introduction
============

HapticPuncture Simulator is an innovative simulation system for epidural anesthesia training. Among its many advantages it is important to say that it is virtual-reality (VR) based, which means that can be used indefinitely. Moreover, it is a low-cost, open-source, customizable prototype that opens doors to new procedure simulators in multiple specialties of medicine.

This guide allows to any user to install the simulator downloaded from \url{www.github.com/robolabo}, and to start using the proposed simulator in \textbf{Ubuntu 14.04}. Furthermore, instructions about how to create new applications are detailed. 

HapticPuncture Simulator is composed by two main modules: an haptic device and a virtual environment. In order to use this system, some requirements exist.


Online Resources
================

* The release is available online in GitHub. 

  https://github.com/Robolabo


* Instructions on how to get started can be found in the 
  documentation folder.

  doc/getting-started.pdf


Materials
=========

The materials used in for the HapticPuncture Simuator 
prototype are listed below. A more detailed list can 
be found in the documentation folder.

  doc/getting-started.pdf

* A computer with Ubuntu 14.04

* An Arduino Due board

* A motor's board X-NUCLEO-IHM04A1

* A voltage generator

* A DC motor MINIMOTOR 2842-012C

* An encoder HEDS 5540 A

* 3D printed structures

* Two Linear Shaft Rail Bars

* Two bearings

* A transmission tendon

* A capstan

* A pulley


Software Requirements
=====================

HapticPuncture requires the following software resources:

* CHAI3D platform.

* Arduino IDE, with two libraries installed: Encoder.h and DueTimer.h.

* The code provided in this folder.

Libraries needed for CHAI3D:
* GLUT package
$sudo apt-get install freeglut3 freeglut3-dev
$sudo apt-get install binutils-gold
* usb-1.0
$sudo apt-get install libusb-1.0-0-dev
* libasound2-dev
$sudo apt-get install libasound2-dev

* Additionally, we need a compiler (gcc):
$sudo apt-get install build-essentials
* pthread 
* rt 
* dl 
* GL 
* GLU 
* usb-1.0

Start the system
================

The C++ code to run the virtual environment needs to be compiled. Firstly, copy the 07-HapticPuncture directory from <HapticPunture directory>/Chai3D_code to <CHAI3d_directory>/GEL/modules/GEL/examples/GLFW/
Then, run in the command line:
$ cd <HapticPunture directory>/Chai3D_code/07-HapticPuncture
$ make

Next, to run HapticPuncture the following code scripts need to be run:
* `Arduino_code/HapticPuncture_arduino.ino` in arduino
* `Chai3D_code/07-HapticPuncture/obj/release/lin-x86_64-ccHapticPuncture/HapticPuncture` by typing in the terminal from lin-x86_64-ccHapticPuncture directory: $ ./HapticPuncture



