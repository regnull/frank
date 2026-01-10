.PHONY: sync
.PHONY: repl
.PHONY: clear_logs
.PHONY: log
.PHONY: eject

sync:
	rsync -av  --progress --checksum --exclude .git MicroPython/ /Volumes/MicroPython/

eject:
	diskutil eject "/Volumes/MicroPython"

repl:
	@DEVICE=$(shell ls /dev/tty.* | grep usbmodem); \
	if [ -z "$$DEVICE" ]; then \
		echo "No USB modem device found"; \
		exit 1; \
	else \
		echo "USB modem device found: $$DEVICE"; \
		screen $$DEVICE 115200; \
	fi

clear_logs:
	rm -f /Volumes/MicroPython/log/*.log

log:
	@last_log=$(shell ls /Volumes/MicroPython/log/*.log | sort -V | tail -n 1); \
	cat $$last_log
