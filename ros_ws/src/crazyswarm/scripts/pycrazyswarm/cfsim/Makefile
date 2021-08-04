firmdir = ../../../../../../crazyflie-firmware
modinc = $(firmdir)/src/modules/interface
modsrc = $(firmdir)/src/modules/src

swig: setup.py cffirmware_wrap.c $(modsrc)/*.c
ifeq ($(CSW_PYTHON),)
	$(error Environment variable CSW_PYTHON must be set to "python2" or "python3")
endif
	$(CSW_PYTHON) setup.py build_ext --inplace

cffirmware_wrap.c: cffirmware.i $(modinc)/*.h
	swig -python -I$(modinc) cffirmware.i

clean:
	rm -f cffirmware.py _cffirmware.so *.pyc cffirmware_wrap.c
	rm -rf build
