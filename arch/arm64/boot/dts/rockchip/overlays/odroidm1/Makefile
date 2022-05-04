# Overlays for the Odroid platform

dtbo-$(CONFIG_ARCH_ROCKCHIP_ODROIDM1) += \
					display_vu8m.dtbo \
					gpio_shortcut.dtbo \
					i2c0.dtbo \
					i2c1.dtbo \
					pwm1.dtbo \
					pwm2.dtbo \
					pwm9.dtbo \
					spi0.dtbo \
					uart0-with-ctsrts.dtbo \
					uart0.dtbo \
					uart1.dtbo

dtbo-$(CONFIG_ARCH_ROCKCHIP_ODROIDM1) += \
					ov5647.dtbo \
					ov5647_irfilter.dtbo \
					imx219.dtbo \
					imx477.dtbo

targets += $(dtbo-y)
always := $(dtbo-y)
