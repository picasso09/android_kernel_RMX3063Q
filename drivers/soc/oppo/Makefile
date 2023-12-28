#ifdef VENDOR_EDIT
# Bin.Li@EXP.BSP.bootloader.bootflow, 2017/05/15, Add for oppo device info and oppo project
obj-$(CONFIG_OPPO_COMMON_SOFT)  +=  oppo_project/
obj-$(CONFIG_OPPO_DEVICE_IFNO)  +=  oppo_devinfo/
obj-$(CONFIG_OPPO_HEALTHINFO)     +=  oppo_healthinfo/
obj-$(CONFIG_OPPO_ALS_CALI)   +=  oppo_als_cali_value/
obj-$(CONFIG_OPPO_HYPNUS)       +=  oppo_hypnus/
obj-y += oppo_custom/

#Liang.Zhang@TECH.Storage.Stability.OPPO_SHUTDOWN_DETECT, 2019/04/28, Add for shutdown detect
obj-$(CONFIG_OPPO_SHUTDOWN_DETECT)  +=  oppo_shutdown_detect/
#endif /* VENDOR_EDIT */
#ifdef VENDOR_EDIT
#Weican.Zhao@PSW.NW.EM, 2019/02/12, Add for OPPO Critical log
#obj-$(CONFIG_OPPO_CRITICALLOG)  +=  oppo_criticallog/
obj-y     +=  oppo_criticallog/

#Kun.Hu@TECH.BSP.Stability.PHOENIX_PROJECT 2019/06/15, Add for PHOENIX_PROJECT
obj-$(CONFIG_OPPO_PHOENIX)     +=  oppo_phoenix/
obj-$(CONFIG_OPPO_FORCE_DUMP)     +=  oppo_dump/
#endif /* VENDOR_EDIT */
#endif /* VENDOR_EDIT */

#ifdef VENDOR_EDIT
#Hailong.Liu@BSP.MM 2019/03/18, Add for svelte
obj-$(CONFIG_OPPO_SVELTE)     +=  oppo_svelte/
#endif
