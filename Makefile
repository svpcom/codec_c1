# object building.
ifeq ($(ARM), 1)
CC=arm-none-linux-gnueabi-gcc
AR=arm-none-linux-gnueabi-ar
else
CC=gcc
AR=ar
endif

TARGET= libvpcodec.a
 
CODEOBJECT = AML_HWEncoder.o enc_api.o rate_control_m8_fast.o m8venclib_fast.o libvpcodec.o

#libvpcodec.so: $(CODEOBJECT)
#	$(CC) $(CFLAGS) $(CODEOBJECT) -o $(TARGET)

libvpcodec.a: $(CODEOBJECT)
	$(AR) cr libvpcodec.a $(CODEOBJECT)

libvpcodec.o: libvpcodec.cpp
	$(CC) $(CFLAGS)  -c $<

AML_HWEncoder.o: AML_HWEncoder.cpp include/AML_HWEncoder.h
	$(CC) $(CFLAGS)  -c $<

enc_api.o: enc_api.cpp enc_api.h
	$(CC) $(CFLAGS)  -c $<

#fill_buffer.o: enc/common/fill_buffer.cpp
#	$(CC) $(CFLAGS)  -c $<
#
rate_control_m8_fast.o: enc/m8_enc_fast/rate_control_m8_fast.cpp enc/m8_enc_fast/rate_control_m8_fast.h
	$(CC) $(CFLAGS)  -c $<

m8venclib_fast.o: enc/m8_enc_fast/m8venclib_fast.cpp enc/m8_enc_fast/m8venclib_fast.h
	$(CC) $(CFLAGS)  -c $<

#dump.o: enc/m8_enc/dump.cpp enc/m8_enc/dump.h
#	$(CC) $(CFLAGS)  -c $<
#
#m8venclib.o: enc/m8_enc/m8venclib.cpp enc/m8_enc/m8venclib.h
#	$(CC) $(CFLAGS)  -c $<
#
#rate_control_m8.o: enc/m8_enc/rate_control_m8.cpp enc/m8_enc/rate_control_m8.h
#	$(CC) $(CFLAGS)  -c $<
#
#noise_reduction.o: enc/m8_enc/noise_reduction.cpp enc/m8_enc/noise_reduction.h
#	$(CC) $(CFLAGS)  -c $<
#
#gxvenclib_fast.o: enc/gx_enc_fast/gxvenclib_fast.cpp enc/gx_enc_fast/gxvenclib_fast.h
#	$(CC) $(CFLAGS)  -c $<
#
#rate_control_gx_fast.o: enc/gx_enc_fast/rate_control_gx_fast.cpp enc/gx_enc_fast/rate_control_gx_fast.h
#	$(CC) $(CFLAGS)  -c $<
#
#parser.o: enc/gx_enc_fast/parser.cpp enc/gx_enc_fast/parser.h
#	$(CC) $(CFLAGS)  -c $<
#
#pred.o: enc/intra_search/pred.cpp enc/intra_search/pred.h
#	$(CC) $(CFLAGS)  -c $<

#pred_neon_asm.o: enc/intra_search/pred_neon_asm.s
#	$(CC) $(CFLAGS)  -c $<


LDFLAGS += -lm -lrt
ifeq ($(ARM), 1)
CFLAGS+=-DARM
else
CFLAGS+=-DARM -std=c99
endif

ifeq ($(REAP), 1)
CFLAGS+=-DREAP_MODE
endif

CFLAGS+= -O2 -Wall -fPIC -mfpu=neon -ftree-vectorize -ffast-math -Iinclude -shared

clean:
	-rm -f *.o
	-rm -f $(TARGET)

	
