# Overlays for the Odroid platform

dtbo-$(CONFIG_ARCH_ROCKCHIP_ODROIDM1) += \
					display_vu8s.dtbo \
					gpio_shortcut.dtbo \
					i2c0.dtbo \
					i2c1.dtbo \
					mcp2515.dtbo \
					mcp2515_12mhz.dtbo \
					mcp2515_16mhz.dtbo \
					pwm1.dtbo \
					pwm2.dtbo \
					pwm3.dtbo \
					spi0.dtbo \
					uart0-with-ctsrts.dtbo \
					uart0.dtbo \
					uart1.dtbo

dtbo-$(CONFIG_ARCH_ROCKCHIP_ODROIDM1) += \
					oled_rtc.dtbo \
					pcf8563.dtbo \
					rtc.dtbo \
					ssd1306.dtbo

targets += $(dtbo-y)
always := $(dtbo-y)
