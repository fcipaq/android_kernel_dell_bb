obj-$(CONFIG_GFX_RGX_DEVFREQ) += dfrgx.o

ccflags-y += \
 -DTHERMAL_DEBUG \
 -I$(srctree)/$(src) \
 -I$(srctree)/drivers/devfreq \
 -I$(srctree)/drivers/staging/imgtec/intel/display/tng/drv \
 -I$(srctree)/drivers/staging/imgtec/intel/graphics/rgx/services/3rdparty/intel_devfreq

dfrgx-y += \
 dev_freq_attrib.o \
 dev_freq_graphics_pm.o \
 df_rgx.o \
 df_rgx_burst.o \
 df_rgx_utils.o
