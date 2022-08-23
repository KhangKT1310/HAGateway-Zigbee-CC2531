
VPATH += ./include
CFLAGS  += -I./include

VPATH += ./Source
CFLAGS  += -I./Source


OBJ += $(OBJ_DIR)/main.o 
OBJ += $(OBJ_DIR)/serial.o
OBJ += $(OBJ_DIR)/app_serial.o
OBJ += $(OBJ_DIR)/ti_znp.o 

