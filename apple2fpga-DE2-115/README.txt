This README file is a modification of the original README file (now called "README_ORIGINAL.txt").
 

This is a reconstruction of an 1980s-era Apple ][+ implemented in VHDL for FPGAs.

Stephen A. Edwards, sedwards@cs.columbia.edu
http://www1.cs.columbia.edu/~sedwards

------------------------------
The current implementation is a modification of the original project of S. Edwards with the following modifications:
a) Pinout modified by Gabriel Caffarena for the Altera DE2-115 board (the original design was intended for the DE2)
b) The SPI controller is improved to support modern SD-cards. The author of the new controller is Stephan Stepim. Version 1.2. for the DE2 is now available on Edwards's website. 
c) The VGA controller requires some revising. The DE2 has 10 bits per RGB channel, while the DE2-115 has 8 bits. I simply truncated the values from 10 to 8 bits without considering much the consequences. I believe that this is producing some color artifacts. Let's leave it for a new version.

The design takes advantage the board's off-chip SRAM, VGA DAC, SD card, audio CODEC, and PS/2 keyboard interface. 

It was designed to be fairly easy to port: the apple2.vhd file should be implementation-agnostic: it only assumes the external availability of 48K of RAM and a keyboard.

It contains a simple read-only Disk II emulator that expects
"nibblized" disk images written raw onto an SD or MMC card (i.e., it does not use a FAT or any other type of filesystem).

The VGA controller (not part of an original Apple) doubles each line and interprets the Apple's NTSC-compatible color signals to produce a color
640 X 480 VGA display with non-standard dot timing.
------------------------------
To compile under Altera's Quartus software, open the apple2fpga.qpf
project file and compile.
------------------------------
VHDL files, in order of elaboration:

timing_generator.vhd Timing signal generation, video counters
character_rom.vhd    The beautiful 5 X 8 uppercase-only text font
video_generator.vhd  Text, lores, and hires mode shift registers
main_roms.vhd        D000-FFFF ROMs: Applesoft and the Monitor
cpu6502.vhd          The 6502 CPU core
apple2.vhd           Top-level of the Apple: mostly address decode
disk_ii_rom.vhd      C600-C6FF ROM: Disk II bootstrap ROM
disk_ii.vhd          Read-only Disk II emulator
vga_controller.vhd   NTSC-to-VGA color interpolation, line doubler
PS2_Ctrl.vhd         Low-level PS/2 keyboard interface
keyboard.vhd         PS/2 keyboard-to-Apple interface
spi_controller.vhd   SD/MMC card controller: reads raw tracks (S. Stepim) 
i2c_controller.vhd   Simple I2C bus driver; initializes the codec
wm8731_audio.vhd     Controller for the Wolfson WM8731 audio codec
DE2_TOP.vhd          Top-level entity for the Altera DE2 board
CLK28MPLL.vhd	     Altera-specific configuration for 28 MHz PLL

Other files:

dsk2nib.c            Converts a 140K .dsk image file to the raw 228K
                     .nib format used by the Disk II emulator

makenibs	     A shell (e.g., bash) script that assembles
		     collections of .dsk files into a file suitable
		     for directly writing onto an SD card		     

rom2vhdl             Script to convert raw ROM files into
		     synthesizable VHDL code.  Used to produce main_roms.vhd

apple2fpga.qpf       Project file for Altera's Quartus
DE2_TOP.qsf          Mostly pin assignments for Altera's Quartus
DE2_TOP.sof	     A compiled bitstream for the DE2 board: the
		     result of compiling all the VHDL files in
		     Quartus; suitable for programming if you have a
		     DE2 board.

dos33master.nib      Bootable disk image: Apple DOS 3.3 system master

bios.a65	     6502 assembly source for a "fake" BIOS
bios.rom	     Binary data for the "fake" BIOS

Makefile             Rules for creating the .zip, .vhd files, etc.
------------------------------
Disk images

The system expects a sequence of "nibblized" (227K) disk images on the
SD card starting at block 0.  Switches on the DE2 board selects which
image appears to be in the drive; the image number is displayed in hex
on two of the seven-segment displays.

Most Apple II disk images are in 140K .dsk files, which stores only
the disk's logical data, i.e., is not encoded.  dsk2nib.c is a small C
program that expands .dsk files to .nib files.

The "makenibs" script is used to find all the .dsk files in a tree of
directories, assemble them into an image suitable for downloading to
the SD card, and print an image number/file name cross-listing.

To write .nib images to an SD/MMC card under Linux, I use

dd if=dos33master.nib of=/dev/sdd

Of course, your card may appear as something other than /dev/sdd.
------------------------------
ROMs

This archive does NOT include a copy of the Apple ][+'s ROMs, which
are copyright Apple Computer.  Instead, it includes a very trivial
BIOS that beeps, displays a text screen, then cycles through some
lores and hires graphics patterns when keys are pressed.  This should
be enough to verify the graphics, sound, and keyboard are working (but
not the disk emulator).  Source for this BIOS is in the bios.a65 file,
which was assembled using the xa65 cross-assembler.

The system requires two ROM images: a 12K image of the system roms
(memory from 0xD000 - 0xFFFF) and a 256-byte image of the Disk II
controller bootstrap ROM (memory from 0xc600 - 0xc6ff if the card is
in the usual slot 6).

Once you obtain them, run the "rom2vhdl" script to convert the binary
files into .vhd files that hold the data.  The Makefile contains rules
for doing this.
------------------------------
Credits:

Peter Wendrich supplied the 6502 core:

-- cpu65xx_fast.vhdl, part of FPGA-64, is made available strictly for personal
-- educational purposes. Distributed with apple2fgpa with permission.
--
-- Copyright 2005-2008 by Peter Wendrich (pwsoft@syntiac.com).
-- All rights reserved.
-- http://www.syntiac.com/fpga64.html

The low-level PS/2 keyboard controller is from ALSE:

-- PS2_Ctrl.vhd
-- ------------------------------------------------
--   Simplified PS/2 Controller  (kbd, mouse...)
-- ------------------------------------------------
-- Only the Receive function is implemented !
-- (c) ALSE. http://www.alse-fr.com

The Apple ][ keyboard emulation was adapted from Alex Freed's FPGApple:
http://mirrow.com/FPGApple/


