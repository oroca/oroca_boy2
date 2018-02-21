TARGET = orocaboy2_gameloader
CC     = gcc
LD     = gcc
CFLAGS =
LIBS   =
LDFLAGS = -lc  -lm
USERCFLAGS = -O2

INC_DIR   = ../../
BUILD_DIR = ./
SRC_PATH  = ../../
SRC_LIST  = main.c \
            util.c \
            util_linux.c \
            uart_linux.c \
            cmd.c \
            loader.c \
            download.c \

ALLSRCBASE = $(notdir $(basename $(SRC_LIST)))
OBJ_LIST   = $(addsuffix .o, $(ALLSRCBASE))

vpath %c $(SRC_PATH)
vpath %h $(SRC_PATH)

.SUFFIXES : .c .o

all : build

build: $(OBJ_LIST)
	gcc $(CFLAGS) $(USERCFLAGS) $(OBJ_LIST) --output $(TARGET) $(LDFLAGS) $(LIBS)

# Compile: create object files from C source files.
define COMPILE_C_TEMPLATE
$(notdir $(basename $(1))).o : $(1)
	@echo $(MSG_COMPILING) $$< to $$@
	gcc -c $$(CFLAGS) $(USERCFLAGS) $$< -o $$@ $$(LIBS)
endef
$(foreach src, $(SRC_LIST), $(eval $(call COMPILE_C_TEMPLATE, $(src))))

clean: clean_list

clean_list :
	@echo $(MSG_CLEANING)
	rm -f $(TARGET)
	rm -f $(TARGET).sym
	rm -f $(TARGET).lss
	rm -f $(OBJ_LIST)
	rm -f $(LSTFILES)
	rm -f $(DEPFILES)
