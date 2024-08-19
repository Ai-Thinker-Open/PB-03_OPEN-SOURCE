BIN = ./output/meshlight.bin
IHEX = ./output/meshlight.ihex
ELF = ./output/meshlight.elf
ASM = ./output/meshlight.asm

CONFIG_PHY6222_PHY_MESH = 1

ROOT := ../../../..
PROJ_ROOT := ..

VPATH = $(PROJ_ROOT)
VPATH += $(PROJ_ROOT)/source/
VPATH += $(PROJ_ROOT)/source/bleMesh/

include $(ROOT)/components/gcc/components.cflags
include $(ROOT)/components/gcc/components.mk

DEF += -DMAX_NUM_LL_CONN=1
DEF += -DGATT_MAX_NUM_CONN=MAX_NUM_LL_CONN+1
#DEF += -DCFG_MESH_FAST_PRO=1
DEF += -DHOST_CONFIG=0x0c

FLASH_LD = $(PROJ_ROOT)/gcc/meshlight.ld

#OTA src
#DEF += -DPHY_SLB_OTA_ENABLE=1
#INC   += -I$(ROOT)/components/profiles/ppsp
VPATH += $(ROOT)/components/libraries/crc16
CSRCS += crc16.c
#CSRCS += ppsp_serv.c
#CSRCS += list_slst.c
#CSRCS += core_queu.c

#project src
SRC_RAW = main.c
SRC_RAW += appl_sample_example_phylight.c
SRC_RAW += OSAL_bleMesh.c
SRC_RAW += bleMesh.c
SRC_RAW += bleMesh_Main.c


STARTUP_RAW = phy6222_start.s

SRC_RAW += $(CSRCS)

INC += -I$(PROJ_ROOT)/source/
INC += -I$(PROJ_ROOT)/source/bleMesh/



CFLAGS += -Os
CFLAGS += -g3
CFLAGS += --specs=nosys.specs
CFLAGS += -mcpu=cortex-m0
#CFLAGS += -mfloat-abi=soft
CFLAGS += -mthumb
CFLAGS += -ffunction-sections
#CFLAGS += -mfpu=fpv4-sp-d16
CFLAGS += -Wall
#CFLAGS += --specs=rdimon.specs

LIBS += -lethermind_mesh_core
LIBS += -lethermind_mesh_models
LIBS += -lethermind_utils

#LFLAGS += -mcpu=cortex-m0
#LFLAGS += -mfloat-abi=soft
#LFLAGS += -mthumb
#LFLAGS += -mfpu=fpv4-sp-d16
LFLAGS += -Wl,--gc-sections
LFLAGS += -Wl,-Map=build.map
LFLAGS += $(ROOT)/misc/bb_rom_sym_m0.gdbsym
LFLAGS += $(LIBS_PATH)
LFLAGS += $(LIBS)

OBJ = $(SRC_RAW:%.c=%.o)
OBJ :=  $(notdir $(OBJ))

STARTUP_OBJ = $(STARTUP_RAW:%.s=%.o)
STARTUP_OBJ :=  $(notdir $(STARTUP_OBJ))



all:$(OBJ) $(STARTUP_OBJ)
	@echo $(STARTUP_OBJ)
	@echo $(OBJ)
	@echo $(BIN)

	mkdir -p output

	arm-none-eabi-gcc -o $(ELF) $(CFLAGS) $(OBJ) $(STARTUP_OBJ) $(LFLAGS) -T$(FLASH_LD) 
	#arm-none-eabi-ld -o $(ELF) $(LFLAGS) $(OBJ) $(STARTUP_OBJ) -T$(FLASH_LD) 
	arm-none-eabi-objcopy -O binary -S $(ELF) $(BIN)
	arm-none-eabi-objcopy -O ihex $(ELF) $(IHEX)
	arm-none-eabi-objdump -s -S $(ELF) > $(ASM)
	
%.o:%.s
	arm-none-eabi-gcc -c $(CFLAGS) $< -o $@
	@echo

%.o:%.c
	arm-none-eabi-gcc -c $(CFLAGS) $(DEF) $(INC) $< -o $@
	@echo

.PHONY: clean
clean :
	@-rm  *.elf *.hex *.bin
	-rm $(OBJ)
	-rm $(STARTUP_OBJ)
