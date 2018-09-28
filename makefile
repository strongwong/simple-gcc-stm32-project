# #工程的名称及最后生成文件的名字
# TARGET = LED_project
# 
# #设定临时性环境变量
# export CC             = arm-none-eabi-gcc           
# export AS             = arm-none-eabi-as
# export LD             = arm-none-eabi-ld
# export OBJCOPY        = arm-none-eabi-objcopy
# 
# #读取当前工作目录
# TOP=$(shell pwd)
# 
# #设定包含文件目录
# INC_FLAGS= -I $(TOP)/CORE                  \
#            -I $(TOP)/HARDWARE    \
#            -I $(TOP)/STM32F10x_FWLib/inc             \
#            -I $(TOP)/SYSTEM        \
#            -I $(TOP)/USER
# 
# CFLAGS =  -W -Wall -g -mcpu=cortex-m3 -mthumb -D STM32F10X_HD -D USE_STDPERIPH_DRIVER $(INC_FLAGS) -O0 -std=gnu11
# C_SRC=$(shell find ./ -name '*.c')  
# C_OBJ=$(C_SRC:%.c=%.o)          
# 
# .PHONY: all clean update      
# 
# all:$(C_OBJ)
# 	$(CC) $(C_OBJ) -T stm32_f103ze_gcc.ld -o $(TARGET).elf   -mthumb -mcpu=cortex-m3 -Wl,--start-group -lc -lm -Wl,--end-group -specs=nano.specs -specs=nosys.specs -static -Wl,-cref,-u,Reset_Handler -Wl,-Map=Project.map -Wl,--gc-sections -Wl,--defsym=malloc_getpagesize_P=0x80 
# 	$(OBJCOPY) $(TARGET).elf  $(TARGET).bin -Obinary 
# 	$(OBJCOPY) $(TARGET).elf  $(TARGET).hex -Oihex
# 
# $(C_OBJ):%.o:%.c
# 	$(CC) -c $(CFLAGS) -o $@ $<
# clean:
# 	rm -f $(shell find ./ -name '*.o')
# 	rm -f $(shell find ./ -name '*.d')
# 	rm -f $(shell find ./ -name '*.map')
# 	rm -f $(shell find ./ -name '*.elf')
# 	rm -f $(shell find ./ -name '*.bin')
# 	rm -f $(shell find ./ -name '*.hex')
# 
# update:
# 	openocd -f /usr/share/openocd/scripts/interface/stlink-v2.cfg  -f /usr/share/openocd/scripts/target/stm32f1x_stlink.cfg -c init -c halt -c "flash write_image erase $(TOP)/LED_project.hex" -c reset -c shutdown
#  

TARGET := LED_project
COMPILERNAME := gcc
CONFIG := bin

SHELL:=/bin/bash

TOOLCHAIN ?= arm-none-eabi
CPU = cortex-m3

LINKER_FILE := stm32_f103ze_gcc.ld
STARTUP_FILE := USER/CoIDE_startup.c

CC = $(TOOLCHAIN)-gcc
AS = $(TOOLCHAIN)-as
LD = $(TOOLCHAIN)-ld
CP = $(TOOLCHAIN)-objcopy
OD = $(TOOLCHAIN)-objdump
RM = $(shell which rm 2>/dev/null)

#TOP=$(shell pwd)

#ifneq ($(strip $(value K)),)
#all clean:
#	$(info Tools $(TOOLCHAIN)-$(COMPILERNAME) not installed.)
#	$(RM) -rf bin
#else

DEFINES = -DSTM32F10X_HD
DEFINES += -DUSE_STDPERIPH_DRIVER

INCLUDES = -I CORE
INCLUDES += -I HARDWARE
INCLUDES += -I STM32F10x_FWLib/inc
INCLUDES += -I SYSTEM
INCLUDES += -I USER

VPATH = CORE
VPATH += HARDWARE
VPATH += STM32F10x_FWLib/src
VPATH += SYSTEM
VPATH += USER

#SRC = $(shell find ./ -name '*.c')

SRC = main.c
SRC += led.c
SRC += delay.c
SRC += sys.c
SRC += CoIDE_startup.c
SRC += system_stm32f10x.c
SRC += misc.c
SRC += stm32f10x_adc.c
SRC += stm32f10x_bkp.c
SRC += stm32f10x_can.c
SRC += stm32f10x_cec.c
SRC += stm32f10x_crc.c
SRC += stm32f10x_dac.c
SRC += stm32f10x_dbgmcu.c
SRC += stm32f10x_dma.c
SRC += stm32f10x_exti.c
SRC += stm32f10x_flash.c
SRC += stm32f10x_fsmc.c
SRC += stm32f10x_gpio.c
SRC += stm32f10x_i2c.c
SRC += stm32f10x_iwdg.c
SRC += stm32f10x_pwr.c
SRC += stm32f10x_rcc.c
SRC += stm32f10x_rtc.c
SRC += stm32f10x_sdio.c
SRC += stm32f10x_spi.c
SRC += stm32f10x_tim.c
SRC += stm32f10x_usart.c
SRC += stm32f10x_wwdg.c
SRC += stm32f10x_it.c

CSRC = $(filter %.c,$(SRC))
ASRC = $(filter %.s,$(SRC))

OBJS = $(CSRC:%.c=$(CONFIG)/%.o)
OBJS+= $(ASRC:%.s=$(CONFIG)/%.o)

DEPS = $(CSRC:%.c=$(CONFIG)/%.d)
DEPS+= $(ASRC:%.s=$(CONFIG)/%.d)

#INCS = /SYSTEM/sys.h
#INCS += $(shell find ./ -name '*.h')

CFLAGS = -mthumb -mcpu=$(CPU) 
CFLAGS += -std=c99 -W -Wall -g
CFLAGS += -O0 -MMD -MP
CFLAGS += $(DEFINES)
CFLAGS += $(INCLUDES)
CFLAGS += -Dgcc

ST_F103FLAGS = -mthumb -mcpu=$(CPU) 
ST_F103FLAGS += -nostartfiles -static
ST_F103FLAGS += -Wl,--gc-sections,--entry,Reset_Handler,-Map,$(CONFIG)/$(TARGET).map
ST_F103FLAGS += -Wl,--start-group -lm -lc -lgcc -Wl,--end-group

# Additional user specified CFLAGS
CFLAGS+=$(EXTRA_CFLAGS)
CPFLAGS = -Obinary
ODFLAGS = -S

.PHONY: all clean directories

#### Rules ####
all: directories $(CONFIG)/$(TARGET).bin

directories: $(CONFIG)

$(CONFIG):
	@mkdir -p $@

$(CONFIG)/%.o: %.c $(CONFIG)/%.d 
	@echo " Compiling $(COMPILERNAME) $<" ;\
	$(CC) -c $(CFLAGS) $< -o $@

$(CONFIG)/%.o: %.s $(CONFIG)/%.d 
	@echo " Assembling $(COMPILERNAME) $<" ;\
	$(CC) -c $(CFLAGS) $< -o $@


$(CONFIG)/$(TARGET).elf: $(OBJS)
	@echo " Linking $(COMPILERNAME) $@" ;\
	#$(CC) -Wl,-T, $(LINKER_FILE) -o $@ $(OBJS)  $(ST_F103FLAGS)
	$(CC) $(OBJS) -T stm32_f103ze_gcc.ld -o $@   -mthumb -mcpu=cortex-m3 -Wl,--start-group -lc -lm -Wl,--end-group -specs=nano.specs -specs=nosys.specs -static -Wl,-cref,-u,Reset_Handler -Wl,-Map=Project.map -Wl,--gc-sections -Wl,--defsym=malloc_getpagesize_P=0x80 

$(CONFIG)/$(TARGET).bin: $(CONFIG)/$(TARGET).elf
	@echo " Copying $(COMPILERNAME) $@..." ;\
	$(CP) $(CPFLAGS) $< $@ ; \
	$(OD) $(ODFLAGS) $< > $(CONFIG)/$(TARGET).lst
#	$(CP) $(TARGET).elf  $(TARGET).bin -Obinary
#	$(CP) $(TARGET).elf  $(TARGET).hex -Oihex  


clean:
	@echo "Cleaning..." ;\
	$(RM) -f $(OBJS) $(DEPS) \
		$(CONFIG)/$(TARGET).bin $(CONFIG)/$(TARGET).elf \
		$(CONFIG)/$(TARGET).lst $(CONFIG)/$(TARGET).map

$(CONFIG)/%.d: ;

# Automatically include any generated dependencies
-include $(DEPS)
#endif


