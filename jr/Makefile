.PHONY: sync
.PHONY: repl

sync:
	rsync -av  --progress --checksum --exclude .git MicroPython/ /Volumes/MicroPython/

repl:
	@DEVICE=$(shell ls /dev/tty.* | grep usbmodem); \
	if [ -z "$$DEVICE" ]; then \
		echo "No USB modem device found"; \
		exit 1; \
	else \
		echo "USB modem device found: $$DEVICE"; \
		screen $$DEVICE 115200; \
	fi

