
ccflags-y += -I$(srctree)/drivers/spi/mediatek/mt6765/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/ili9882n/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/ili9882n/firmware/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/mt6765/include/

obj-$(CONFIG_TOUCHSCREEN_ILI9882N) += ilitek_v3.o \
	ilitek_v3_spi.o \
	ilitek_v3_mtk.o \
	ilitek_v3_ic.o \
	ilitek_v3_touch.o \
	ilitek_v3_mp.o \
	ilitek_v3_hostdl.o \
	ilitek_v3_node.o

