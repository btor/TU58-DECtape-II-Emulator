# TU58-DECtape-II-Emulator
Digital Equipment TU58 DECtape II emulation with a microcontroller and a SD-card

The emulator was developed using open soure hardware and software components. 
The microcontroller is an Arduino Serial (with a AVR ATmega328, http://arduino.cc/en/Main/Hardware). 
Arduino is a low-cost microcontroller board available from different companies.
The development system is available for free from http://www.arduino.cc 
		
The firmware is an extended version of the TU58 DOS emulator written by William T. Kranz 
which is still available from http://www.willsworks.net/pdp11/rt11arc.exe

In 2006 I ported this emulator to Widows & DOS with his permission.
The missing Modified Radial Serial Protocol (MRSP) support was added in 2011. 
The Windows/DOS emulator is available form http://www.belatorok.com/computing/pdp11/tu58/tu58.zip 
Since no MRSP specification was available, the MRSP-specific code in the firmware is a result of 
implemetation of a loop in write command found in the source code of Don North's TU58 emulator 
http://www.slowdeath.com/AK6DN/PDP-11/TU58/tu58em/index.html and reverse engineering of the 
XXDP ZTUUF0 TU58 test program 
http://bitsavers.informatik.uni-stuttgart.de/pdf/dec/pdp11/xxdp/fiche_200dpi/0150_CZTUUF0_TU58.pdf

Since the timing of the MRSP protocol at least with the ZTUUF0 test program is critical, 
the Windows emulator does not work realiably. Possible problems are timing delays due to the 
FIFOs of the modern serial interfaces, time delays with USB-to-serial converters 
and/or delays if multiple programs or tasks are running on the PC at the same time.

A logical solution to overcome of the above problems was the development of a standalone device. 
The firmware of the standalone emulator is available form: http://www.belatorok.com/computing/pdp11/tu58/arduino/tu58.zip 
		
Usage:
The emulator use disk image files stored on the SD-card. 100 disk image files are 
selectable with the keyboard of the emulator. The file names have the following format: TU58-%%.dsk

%% is a two digit decimal number, range: 00 - 99
			
To enter the setup menu press the "Menu" key for a second. Selectable parameters:
			
	Baudrate (9600-115200)
	Delay (0-255 ms)
	Protocol (RSP only or RSP/MRSP)
	D0 Filename (disk image for unit 0)
	D1 Filename (disk image for unit 1)
	Write parameters to EEPROM
	Exit setup
			
		Press
		"Menu" to select the next parameter
		Up or down arrows to change the value of the parameter
		Enter to exit setup menu.
		
Using large disk mage files with the TU58 emulator
According to Will Kranz, disk image files larger than the 256k of the TU58 cartrige can be used with the emulator.
When used with the dd.sys and ddx.sys device drivers, do not use the squeeze and init commands, since they do not 
work properly with disk images > 256kBytes (>512 blocks). He foud out however, a method to modify block size of 
the dd driver and made a dw.sys driver for rl (10 mByte) disk images. 
The block size is strored at hexadecimal offset 0x2c and 0x2d.

The original dd.sys and ddx.sys drivers have 512 (hex 200) blocks. (offset 0x2c=00, 0x2d=02)
To modify the driver you can use a hex editor to chage the value on 0x2c and 0x2d.
I created device drivers for 10mByte rl compatible (dl:) and 32mByte mscp compatible (du:) disk images.
The name of the 10 mByte rl: compatible device drivers are dw.sys and dwx.sys, and for the 32 mByte sized 
mscp compatible disk images dv.sys and dvx.sys. The dv.sys, dvx.sys, dw.sys and dwx.sys files can be downloaded on 
a TU58 disk image file: <a target="new" href="TU58-dv-and-dw-drivers.dsk">TU58-dv-and-dw-drivers.dsk
To use the drivers mount the TU58-dv-and-dw-drivers.dsk file using the TU58 emulator, cop the drivers to 
the system disk, e.g., use the "copy /sys dd0:d*.sys dl:" command, and reboot the system.
If you boot rt11xm you before using the driver you have to load the driver with the load command, e.g., 
load dd:". The driver can be unloaded with the unload command. Do not load two or three TU58 drivers at the 
same time (dd, dv or dw)!
		
Empty 10 and 32 mByte disk image files are available: 10m-rl.zip and 32m.zip
        
Testing the emulator
The Radial Serial Protocol (RSP) specitic code was tested with RT11 V4.0 and 5.3 running on 
a PDP11/23, PDP11/53 and on different simulated PDP11s on Ersatz 11 (E11).
The Modified Radial Serial Protocol (MRSP) was tested with the XXDP ZTUUF0 test program.<br><br>
		
To test MRSP capabilities download a XXDP TU58 tape image from: 
http://www.willsworks.net/pdp11/xxdpv2d.tap,
Copy xxdp2d.tap to the base directory of a SD card, rename the file to, e.g., TU58-90.dsk
Copy xxdp2d.tap to the base directory of a SD card, rename the file to, e.g., TU58-91.dsk
Select TU58-90.dsk using the buttons as disk image0, and TU58-91.dsk as disk image1. 
We will use disk image0 to boot XXDP, disk image1 will be used as a test drive, any data on this tape will 
be overwritten!
		
A step-by-step guide is available in xxdp.txt
