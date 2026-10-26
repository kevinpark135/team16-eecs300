.PHONY: all firmware clean

all:
	$(MAKE) -C thermal

firmware:
	python3 build_firmware.py

clean:
	rm -rf build
