.PHONY: sync

sync:
	rsync -av  --progress --checksum --exclude .git MicroPython/ /Volumes/MicroPython/

repl:
 	screen $(ls /dev/tty.* | grep usbmodem) 115200