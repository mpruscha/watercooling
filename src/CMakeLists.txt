Project(watercooling_free_rtos)

include(ExternalProject)
cmake_minimum_required(VERSION 2.6)

SET(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS)
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")


SET(CMAKE_SYSTEM_NAME Generic)

SET(CMAKE_C_COMPILER avr-gcc)
SET(CMAKE_CXX_COMPILER avr-g++)


find_package (avr-libc)

SET(CMAKE_C_FLAGS  "-mmcu=atmega32u4 -std=gnu99 -gstabs -Wall -Wstrict-prototypes -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Os  -DF_CPU=16000000")
SET(CMAKE_CXX_FLAGS "-mmcu=atmega32u4 -std=gnu99 -gstabs -Wall -Wstrict-prototypes -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Os -DF_CPU=16000000")

add_definitions(-D__AVR_ATmega32U4__)
add_definitions(-DUSE_LUFA_CONFIG_HEADER)
add_definitions(-IConfig)
add_definitions(-DF_USB=16000000)
add_definitions(-DBOARD=BOARD_MICRO)
SET(MCU atmega32u4)
SET(ARCH AVR8)
SET(BOARD BOARD_MICRO)
SET(F_CPU 16000000)
SET(F_USB $(F_CPU))
SET(OPTIMIZATION s)
SET(TARGET main)
SET(LUFA_PATH /home/martin/Downloads/lufa-LUFA-140928/LUFA)


include_directories(../src/)
include_directories(/usr/lib/avr/include/)
include_directories(../include/)
include_directories(/home/martin/Downloads/lufa-LUFA-140928/)
include_directories(ds18b20/)

include_directories(/home/martin/Apps/FreeRTOS/Source/)
include_directories(/home/martin/Apps/FreeRTOS/Source/include/)
include_directories(/home/martin/Apps/FreeRTOS/Demo/Common/include/)
include_directories(/home/martin/Apps/FreeRTOS/Source/portable/GCC/ATMega323/)



add_executable(watercooling_free_rtos main.c temp_sensors.c lufa_usb.c /home/martin/Apps/FreeRTOS/Source/portable/GCC/ATMega323/port.c /home/martin/Apps/FreeRTOS/Source/croutine.c /home/martin/Apps/FreeRTOS/Source/event_groups.c /home/martin/Apps/FreeRTOS/Source/list.c /home/martin/Apps/FreeRTOS/Source/queue.c /home/martin/Apps/FreeRTOS/Source/tasks.c /home/martin/Apps/FreeRTOS/Source/timers.c /home/martin/Apps/FreeRTOS/Source/portable/MemMang/heap_1.c Descriptors.c ds18b20/onewire.c ds18b20/ds18x20.c ds18b20/crc8.c)
target_link_libraries(watercooling_free_rtos ${avr-libc_LIBRARIES} /home/martin/Downloads/lufa-LUFA-140928/LUFA/Drivers/USB/build/libusb.a)

#set (cu_cmd "cu")
#set (cu_arg -l /dev/ttyACM0 -s 1200)

#add_custom_command(
#download
#COMMAND ${cu_cmd} ${cu_arg}  
#message(STATUS "git ver[${git_result}]: ${git_ver}")
#)

add_custom_target(download 
    #COMMAND stty -F /dev/ttyACM0 1200
    #COMMAND echo 1 > /dev/ttyACM0
    #COMMAND tail -f /dev/ttyACM0
    #COMMAND close /dev/ttyACM0
    #COMMAND ${cu_cmd} ${cu_arg} 
    #COMMAND cu -l /dev/ttyACM0 -s 1200
	COMMAND avr-objcopy -O ihex watercooling_free_rtos watercooling_free_rtos.hex
	COMMAND avrdude -v -p atmega32u4 -c avr109 -P /dev/ttyACM0 -b 57600 -D -U flash:w:watercooling_free_rtos.hex:i
	)