firmdir = ../../../../../../crazyflie-firmware
modinc = $(firmdir)/src/modules/interface
modsrc = $(firmdir)/src/modules/src

swig:
	swig -python -I$(modinc) cffirmware.i
	gcc -std=c99 -shared -fPIC \
		-I$(modinc) -I/usr/include/python2.7 \
		$(modsrc)/planner.c $(modsrc)/pptraj.c $(modsrc)/avoidtarget.c $(modsrc)/fig8traj.c \
		cffirmware_wrap.c \
		-lm -lpython2.7 \
		-o _cffirmware.so
	rm cffirmware_wrap.c

clean:
	rm -f cffirmware.py cffirmware.pyc _cffirmware.so __init__.pyc
