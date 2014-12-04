CC = g++
ifeq ($(shell sw_vers 2>/dev/null | grep Mac | awk '{ print $$2}'),Mac)
	CFLAGS = -Wall -g -DGL_GLEXT_PROTOTYPES -I./include/ -I/usr/X11/include -DOSX -Ieigen
	LDFLAGS = -framework GLUT -framework OpenGL \
    	-L"/System/Library/Frameworks/OpenGL.framework/Libraries" \
    	-lGL -lGLU -lm -lstdc++ -O3
else
	CFLAGS = -Wall -g -DGL_GLEXT_PROTOTYPES -Iglut-3.7.6-bin -Ieigen -O3
	LDFLAGS = -lglut -lGLU
endif

RM = /bin/rm -f
all: main
main: InverseKinematics.o
	$(CC) $(CFLAGS) -o ik InverseKinematics.o $(LDFLAGS)
InverseKinematics.o: InverseKinematics.cpp
	$(CC) $(CFLAGS) -c InverseKinematics.cpp -o InverseKinematics.o
clean:
	$(RM) *.o ik



