# Overlays for the Odroid platform

dtbo-$(CONFIG_ARCH_ROCKCHIP_ODROIDM1) += \
					spi0.dtbo \
					i2c0.dtbo \
					i2c1.dtbo \
					uart0.dtbo \
					uart0-with-ctsrts.dtbo \
					uart1.dtbo \
					pwm1-0.dtbo \
					pwm1-1.dtbo \
					pwm2.dtbo \
					pwm9.dtbo \
					pwm10.dtbo \
					pwm11-ir.dtbo

dtbo-$(CONFIG_ARCH_ROCKCHIP_ODROIDM1) += \
					ov5647.dtbo

targets += $(dtbo-y)
always := $(dtbo-y)
