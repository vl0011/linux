# Overlays for the Odroid platform

dtbo-$(CONFIG_ARCH_ROCKCHIP_ODROIDM1) += \
					can0.dtbo \
					display_vu8m.dtbo \
					gpio_shortcut.dtbo \
					i2c0.dtbo \
					i2c1.dtbo \
					mcp2515.dtbo \
					mcp2515_12mhz.dtbo \
					mcp2515_16mhz.dtbo \
					pwm1.dtbo \
					pwm2.dtbo \
					pwm9.dtbo \
					spi0.dtbo \
					uart0-with-ctsrts.dtbo \
					uart0.dtbo \
					uart1.dtbo

dtbo-$(CONFIG_ARCH_ROCKCHIP_ODROIDM1) += \
					oled_rtc.dtbo \
					ov5647.dtbo \
					ov5647_irfilter.dtbo \
					pcf8563.dtbo \
					rtc.dtbo \
					ssd1306.dtbo

targets += $(dtbo-y)
always := $(dtbo-y)
