INCLUDE_DIRS =
LIB_DIRS =
CC = g++

CDEFS=
#CFLAGS= -O0 $(INCLUDE_DIRS) $(CDEFS)
#CFLAGS= -O0 -msse3 -malign-double $(INCLUDE_DIRS) $(CDEFS)
#CFLAGS= -O2 -msse3 -malign-double $(INCLUDE_DIRS) $(CDEFS)
#CFLAGS= -O3 $(INCLUDE_DIRS) $(CDEFS)
CFLAGS= -O3 -msse3 $(INCLUDE_DIRS) $(CDEFS)
#CFLAGS= -O3 -mssse3 $(INCLUDE_DIRS) $(CDEFS)
LIBS=-lpthread

PRODUCT=test_code

HFILES=
CPPFILES= test_code.cpp

SRCS= ${HFILES} ${CFILES}
OBJS= ${CPPFILES:.cpp=.o}

all:	${PRODUCT}

clean:
	-rm -f *.o *.NEW *~
	-rm -f ${PRODUCT} ${DERIVED} ${GARBAGE}

${PRODUCT}:	${OBJS}
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $(OBJS) $(LIBS)

depend:

.c.o:
	$(CC) $(CFLAGS) -c $<
