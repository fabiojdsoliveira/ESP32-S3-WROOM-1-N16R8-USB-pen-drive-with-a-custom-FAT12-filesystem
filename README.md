# ESP32-S3-WROOM-1-N16R8 : USB-pen-drive-with-a-custom-FAT12-filesystem — built from scratch

The ESP32-S3 has a native USB port that can act as a USB Mass Storage device — in other words, it can appear on your PC as a pen drive. The idea was simple: store files on the ESP's internal flash, plug it into a PC, edit those files, unplug, and have the ESP read the updated files and act on them.

The problem is that Espressif's official FAT library uses a wear-levelling layer that remaps flash sectors internally. When Windows writes files via USB, it writes directly to physical sectors — bypassing that layer entirely. The result: the ESP can no longer read what Windows wrote, and vice versa. The two sides see different data on the same flash chip.

To solve this, I implemented a complete FAT12 filesystem parser from scratch — no external libraries. Both the USB MSC callbacks and the ESP's own file read/write functions access the same raw flash sectors directly, so PC and firmware always see exactly the same files.

-------------------------------------------------------------------

USB MSC
Partition appears as a pen drive on Windows

FAT12 parser
Boot sector, FAT table, cluster chains — all built from scratch

LFN support
Long filenames with accented characters, read and write

File API
Read, write, append — with PSRAM support for large files

----------------------------

The final demo uses this as a real-world foundation: a config.json file that the user can edit on the PC to configure the device, and a data.json log file in JSON Lines format that the firmware appends to every time the USB drive is ejected — tracking a counter and a timestamp across sessions.

One honest caveat worth mentioning: this implementation writes directly to flash with no wear-levelling, which means flash blocks could degrade with very frequent writes. For occasional use — config files, log dumps, music — it's perfectly fine. For high-frequency data logging, a buffered approach or an SD card would be more appropriate.

Youtube: https://www.youtube.com/watch?v=j9MnhmSkKlU

