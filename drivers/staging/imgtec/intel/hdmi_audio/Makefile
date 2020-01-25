obj-$(CONFIG_INTEL_HDMI_AUDIO) += hdmi_audio.o

ccflags-y += \
 -I$(srctree)/$(src) \
 -I$(srctree)/drivers/staging/imgtec/intel/display/tng/drv \
 -I$(srctree)/drivers/staging/imgtec/intel/otm_hdmi/os/android/include \
 -I$(srctree)/drivers/staging/imgtec/intel/otm_hdmi/pil/include

hdmi_audio-y += \
 intel_mid_hdmi_audio.o \
 intel_mid_hdmi_audio_debug.o \
 intel_mid_hdmi_audio_if.o
