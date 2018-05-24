OUTPUT_NAME = project

# Source files
SRCS      = main.c \
            data.c \
            memory.c \
            project1.c \
						circbuf.c \
						log.c\
						profiler.c \
						test_circbuf.c\
            SPI.c\
            nRF24L01.c

LIBS_SRCS = data.c \
            memory.c \
						circbuf.c \
						log.c\
						profiler.c\
            SPI.c\
            nRF24L01.c

TEST_SRCS = test_main.c \
            test_data.c \
            test_memory.c \
						test_circbuf.c
