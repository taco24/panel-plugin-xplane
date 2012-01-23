
LIBPATH=
WINDLLMAIN=
COMPILERFLAGS=-Wall
CC=gcc
LIBRARIES=
INCLUDE=-I./SDK/CHeaders/XPLM
HIDFILE=


HOSTOS=$(shell uname | tr A-Z a-z)
ifeq ($(HOSTOS),linux)
 LNFLAGS=-shared -rdynamic -nodefaultlibs -L.
 CFLAGS=$(COMPILERFLAGS) -DAPL=0 -DIBM=0 -DLIN=1 -DXPLM200=1
 LIBRARIES+=-lpthread
 HIDFILE=linux/hid.c
else
  HOSTOS=windows
  LNFLAGS=-m32 -Wl,-O1 -shared -L.
  CFLAGS=$(COMPILERFLAGS) -DAPL=0 -DIBM=1 -DLIN=0 -DXPLM200=1
  LIBPATH+=-L".\SDK\Libraries\Win" -L".\lib"
  LIBRARIES+=-lsetupapi -lXPLM -lpthreadGC1
  INCLUDE+=-I./include
  HIDFILE=windows/hid.c
endif


all:
	$(CC) -c $(INCLUDE) $(CFLAGS) $(HIDFILE)
	$(CC) -c $(INCLUDE) $(CFLAGS) utils.c
	$(CC) -c $(INCLUDE) $(CFLAGS) log.c
	$(CC) -c $(INCLUDE) $(CFLAGS) time.c
	$(CC) -c $(INCLUDE) $(CFLAGS) properties.c
	$(CC) -c $(INCLUDE) $(CFLAGS) settings.c
	$(CC) -c $(INCLUDE) $(CFLAGS) rp_driver.c
	$(CC) -c $(INCLUDE) $(CFLAGS) rp_controller.c
	$(CC) -c $(INCLUDE) $(CFLAGS) panel_plugin.c

	$(CC) -o panel_plugin.xpl hid.o utils.o log.o properties.o settings.o rp_driver.o rp_controller.o time.o panel_plugin.o  $(WINDLLMAIN) $(LNFLAGS) $(LIBPATH) $(LIBRARIES)

clean:
	$(RM) *.o *.xpl
