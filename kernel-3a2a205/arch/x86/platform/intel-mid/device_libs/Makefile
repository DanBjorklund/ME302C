CFLAGS_platform_clv_regulator.o			:= -Werror

# IPC Devices
obj-y									+= platform_mrfld_audio.o
obj-y									+= platform_ipc.o
obj-y									+= platform_scu_mip.o
obj-y									+= platform_msic.o
obj-y									+= platform_msic_audio.o
obj-y									+= platform_msic_gpio.o
obj-y									+= platform_msic_ocd.o
obj-y									+= platform_tc35876x.o
obj-y									+= platform_ctp_audio.o
obj-y									+= platform_sst_audio.o
obj-y									+= platform_clv_regulator.o
obj-y									+= pci/
obj-$(subst m,y,$(CONFIG_MSIC_GPADC))					+= platform_msic_adc.o
obj-$(subst m,y,$(CONFIG_BATTERY_INTEL_MDF))				+= platform_msic_battery.o
obj-$(subst m,y,$(CONFIG_INTEL_MID_POWER_BUTTON))			+= platform_msic_power_btn.o
obj-$(subst m,y,$(CONFIG_GPIO_INTEL_PMIC))				+= platform_pmic_gpio.o
obj-$(subst m,y,$(CONFIG_MID_PWM))                                      += platform_mid_pwm.o
obj-$(subst m,y,$(CONFIG_INTEL_MFLD_THERMAL))				+= platform_msic_thermal.o
obj-$(subst m,y,$(CONFIG_SENSORS_THERMAL_MRFLD))			+= platform_mrfl_thermal.o
obj-$(subst m,y,$(CONFIG_SENSORS_MID_VDD))				+= platform_msic_vdd.o
obj-$(subst m,y,$(CONFIG_SENSORS_MRFL_OCD))				+= platform_mrfl_ocd.o
obj-$(subst m,y,$(CONFIG_PMIC_CCSM))                                    += platform_mrfl_pmic.o
obj-$(subst m,y,$(CONFIG_I2C_PMIC))                                    += platform_mrfl_pmic_i2c.o
obj-$(subst m,y,$(CONFIG_INTEL_SCU_FLIS))				+= platform_scu_flis.o
obj-$(subst m,y,$(CONFIG_IIO_BASINCOVE_GPADC))				+= platform_bcove_adc.o
# I2C Devices
obj-$(subst m,y,$(CONFIG_CRYSTAL_COVE))					+= platform_crystal_cove.o
obj-$(subst m,y,$(CONFIG_SENSORS_EMC1403))				+= platform_emc1403.o
obj-$(subst m,y,$(CONFIG_SENSORS_LIS3LV02D))				+= platform_lis331.o
obj-$(subst m,y,$(CONFIG_GPIO_PCA953X))					+= platform_max7315.o
obj-$(subst m,y,$(CONFIG_GPIO_PCA953X))					+= platform_tca6416.o
obj-$(subst m,y,$(CONFIG_SENSORS_MPU6500)) 				+= platform_gyro.o
obj-$(subst m,y,$(CONFIG_SENSORS_AKM8963)) 				+= platform_compass.o
obj-$(subst m,y,$(CONFIG_SENSORS_AL3320A)) 				+= platform_lightsensor.o
ifdef CONFIG_BATTERY_MAX17042
obj-$(subst m,y,$(CONFIG_BATTERY_MAX17042))				+= platform_max17042.o
else
obj-$(subst m,y,$(CONFIG_BATTERY_MAX17050))				+= platform_max17042.o
endif
obj-$(subst m,y,$(CONFIG_BYT_ULPMC_BATTERY))				+= platform_ulpmc_battery.o
obj-$(subst m,y,$(CONFIG_TOUCHSCREEN_ATMEL_MXT224))			+= platform_mxt224.o
obj-$(subst m,y,$(CONFIG_A1026))					+= platform_a1026.o
obj-$(subst m,y,$(CONFIG_SENSORS_APDS990X))				+= platform_apds990x.o
obj-$(subst m,y,$(CONFIG_CHARGER_BQ24192))				+= platform_bq24192.o
obj-$(subst m,y,$(CONFIG_SENSORS_L3G4200D_POLL))			+= platform_l3g4200d.o
obj-$(subst m,y,$(CONFIG_SENSORS_LSM303_MAG))				+= platform_lsm303.o
obj-$(subst m,y,$(CONFIG_PN544_NFC))					+= platform_pn544.o
obj-$(subst m,y,$(CONFIG_SENSORS_MS5607))				+= platform_ms5607.o
ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_RMI4_I2C
obj-$(subst m,y,$(CONFIG_TOUCHSCREEN_SYNAPTICS_RMI4_I2C))		+= platform_s3202.o
else
obj-$(subst m,y,$(CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4))		+= platform_s3202.o
obj-$(subst m,y,$(CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4))		+= platform_s3400.o
endif
obj-$(subst m,y,$(CONFIG_BQ24261_CHARGER))                            += platform_bq24261.o
# Camera
obj-$(subst m,y,$(CONFIG_VIDEO_ATOMISP))				+= platform_camera.o
ifdef CONFIG_VIDEO_ATOMISP
obj-$(subst m,y,$(CONFIG_VIDEO_MT9E013))				+= platform_mt9e013.o
obj-$(subst m,y,$(CONFIG_VIDEO_MT9D113))				+= platform_mt9d113.o
obj-$(subst m,y,$(CONFIG_VIDEO_MT9M114))				+= platform_mt9m114.o
obj-$(subst m,y,$(CONFIG_VIDEO_MT9V113))				+= platform_mt9v113.o
obj-$(subst m,y,$(CONFIG_VIDEO_OV8830))					+= platform_ov8830.o
obj-$(subst m,y,$(CONFIG_VIDEO_IMX175))					+= platform_imx175.o
obj-$(subst m,y,$(CONFIG_VIDEO_IMXVB))					+= platform_imx135.o
obj-$(subst m,y,$(CONFIG_VIDEO_OV9724))					+= platform_ov9724.o
obj-$(subst m,y,$(CONFIG_VIDEO_LM3554))					+= platform_lm3554.o
obj-$(subst m,y,$(CONFIG_VIDEO_LM3559))					+= platform_lm3559.o
# Peter++
obj-$(subst m,y,$(CONFIG_VIDEO_OV5693))					+= platform_ov5693.o
endif
# SPI Devices
obj-$(subst m,y,$(CONFIG_SERIAL_MRST_MAX3110))				+= platform_max3111.o
# Patrick
obj-$(subst m,y,$(CONFIG_VIDEO_SPCA700XA))				+= platform_spca700xa.o
# HSI Devices
obj-$(subst m,y,$(CONFIG_HSI_ARASAN))					+= platform_hsi_modem.o
obj-$(subst m,y,$(CONFIG_HSI_ARASAN))					+= platform_ffl_modem.o
obj-$(subst m,y,$(CONFIG_HSI_ARASAN))					+= platform_edlp_modem.o
obj-$(subst m,y,$(CONFIG_HSI_ARASAN))					+= platform_logical_modem.o
# Modem control Device
obj-$(subst m,y,$(CONFIG_MDM_CTRL))					+= platform_modem_ctrl.o
# UART Devices
obj-$(subst m,y,$(CONFIG_SERIAL_MFD_HSU))				+= platform_hsu.o
# MISC Devices
obj-$(subst m,y,$(CONFIG_KEYBOARD_GPIO))				+= platform_gpio_keys.o
obj-$(subst m,y,$(CONFIG_ANDROID_SWITCH_MID))				+= platform_switch_mid.o
obj-$(subst m,y,$(CONFIG_ANDROID_SWITCH_MID_USER_NOTIFY))		+= platform_switch_mid_user_notify.o
obj-$(subst m,y,$(CONFIG_I2C_GPIO))					+= platform_i2c_gpio.o
# WIFI
obj-$(subst m,y,$(CONFIG_WL12XX_PLATFORM_DATA))				+= platform_wl12xx.o
obj-$(subst m,y,$(CONFIG_BCM43XX_PLATFORM_DATA))			+= platform_bcm43xx.o
# Bluetooth
obj-$(subst m,y,$(CONFIG_TI_ST))					+= platform_btwilink.o
obj-$(subst m,y,$(CONFIG_BCM_BT_LPM))					+= platform_btlpm.o
# Pin Control
obj-$(subst m,y,$(CONFIG_PINCTRL_INTEL_MID))				+= platform_mid_pinctrl.o
obj-$(subst m,y,$(CONFIG_INTEL_MID_OSNIB_ILB))				+= platform_osnib_ilb.o
# Battery chris: (((
obj-$(subst m,y,$(CONFIG_ME302C_BATTERY_SMB347))		+= platform_me302c_smb347.o
obj-$(subst m,y,$(CONFIG_ME302C_BATTERY_BQ27520))		+= platform_bq27520.o
# )))
# Joe add for ELAN Touch ++
obj-$(subst m,y,$(CONFIG_TOUCHSCREEN_ELAN_EKTH3374))	+= platform_ektf3k.o
# Joe add for ELAN Touch --

# Joe add for EC driver ++
obj-$(subst m,y,$(CONFIG_HID_ASUS_PAD_EC))	+= platform_asus_ec.o
# Joe add for EC driver --

# Joe add for Elan touch pad driver ++
obj-$(subst m,y,$(CONFIG_MOUSE_ELAN_TOUCHPAD))	+= platform_elan_touchpad.o
# Joe add for Elan touch pad driver --

# Jui-Chuan add for Atmel ++
obj-$(subst m,y,$(CONFIG_TOUCHSCREEN_ATMEL_MXT1664S))   +=  platform_mxt1664s.o
# Jui-Chuan add for Atmel --

# add by leo for Goodix touch ++
obj-$(subst m,y,$(CONFIG_TOUCHSCREEN_GOODIX_GT927))      += platform_gt927.o
# add by leo for Goodix touch --

