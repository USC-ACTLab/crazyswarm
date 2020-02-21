firmdir = ../../../../../../crazyflie-firmware
modinc = $(firmdir)/src/modules/interface
modsrc = $(firmdir)/src/modules/src

swig: setup.py cffirmware_wrap.c $(modsrc)/*.c
	python2 setup.py build_ext --inplace

cffirmware_wrap.c: $(modinc)/*.h
	swig -python -I$(modinc) cffirmware.i

clean:
	rm -f cffirmware.py _cffirmware.so *.pyc cffirmware_wrap.c
