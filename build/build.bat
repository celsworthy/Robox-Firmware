:: revision and filename default to git tag, but are overridden by parameter if specified
@echo off
if "%1"=="" (for /f "delims=" %%a in ('git describe --tags') do set FILENAME=%%a) else (set FILENAME=%1)
echo building %FILENAME%

arm-none-eabi-gcc -Os -DOPTIMIZATION -DFIRMWARE_REVISION=\"%FILENAME%\" ^
-mcpu=cortex-m3 -mthumb -Wall -mlong-calls -ffunction-sections -g -nostartfiles -Wl,--gc-sections ^
-Dat91sam3u2 -Dflash -DTRACE_LEVEL= ^
-I../at91lib/boards/4pi ^
-I../at91lib/peripherals ^
-I../at91lib/usb/device ^
-I../at91lib ^
-I../external_libs ^
-I../at91lib/memories/sd_mmc ^
-I../at91lib/peripherals/hsmci ^
-Trobox.lds ^
../src/main.c ^
../src/syscalls.c ^
../src/board_test.c ^
../src/command.c ^
../src/eeprom.c ^
../src/file_system.c ^
../src/gcode.c ^
../src/heaters.c ^
../src/motion.c ^
../src/motion_hal.c ^
../src/motion_hal_r1.c ^
../src/motion_hal_r2.c ^
../src/parameters.c ^
../src/position.c ^
../src/reprogram.c ^
../src/samadc.c ^
../src/user_interface.c ^
../src/util.c ^
../at91lib/boards/4pi/board_cstartup_gnu.c ^
../at91lib/boards/4pi/board_lowlevel.c ^
../at91lib/boards/4pi/exceptions.c ^
../at91lib/peripherals/hsmci/hsmci.c ^
../at91lib/memories/sd_mmc/sd_mmc.c ^
../at91lib/peripherals/adc/adc12.c ^
../at91lib/peripherals/irq/nvic.c ^
../at91lib/peripherals/pio/pio.c ^
../at91lib/peripherals/systick/systick.c ^
../at91lib/peripherals/tc/tc.c ^
../at91lib/usb/common/cdc/CDCLineCoding.c ^
../at91lib/usb/common/cdc/CDCSetControlLineStateRequest.c ^
../at91lib/usb/common/core/USBConfigurationDescriptor.c ^
../at91lib/usb/common/core/USBEndpointDescriptor.c ^
../at91lib/usb/common/core/USBFeatureRequest.c ^
../at91lib/usb/common/core/USBGenericDescriptor.c ^
../at91lib/usb/common/core/USBGenericRequest.c ^
../at91lib/usb/common/core/USBGetDescriptorRequest.c ^
../at91lib/usb/common/core/USBInterfaceRequest.c ^
../at91lib/usb/common/core/USBSetAddressRequest.c ^
../at91lib/usb/common/core/USBSetConfigurationRequest.c ^
../at91lib/usb/device/cdc-serial/CDCDSerialDriver.c ^
../at91lib/usb/device/cdc-serial/CDCDSerialDriverDescriptors.c ^
../at91lib/usb/device/core/USBD_UDPHS.c ^
../at91lib/usb/device/core/USBDCallbacks_Initialized.c ^
../at91lib/usb/device/core/USBDCallbacks_Reset.c ^
../at91lib/usb/device/core/USBDDriver.c ^
../at91lib/usb/device/core/USBDDriverCb_CfgChanged.c ^
../at91lib/usb/device/core/USBDDriverCb_IfSettingChanged.c ^
../at91lib/utility/stdio.c ^
../at91lib/utility/string.c ^
../external_libs/cmsis/core_cm3.c ^
-o %FILENAME%.elf -lm
arm-none-eabi-objcopy -O binary %FILENAME%.elf %FILENAME%.bin
arm-none-eabi-objdump -dS %FILENAME%.elf > %FILENAME%.disassembly
arm-none-eabi-nm -n %FILENAME%.elf > %FILENAME%.info
anticheckbin %FILENAME%.bin 000000bc
arm-none-eabi-size %FILENAME%.elf
