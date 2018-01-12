# RFID_STM32L052K6
This is a RFID program.hardware includes MCU which is STM32L052K6,RF Transceiver chip which is CC1101,Accelerometer which is MMA7361L that is ±1.5g, ±6g Three Axis Low-g Micromachined Accelerometer.
                                                                                      ————yufei
function include：
1.CC1101 transfer and receive function
2.CC1101 WOR function
3.MMA7361L accelerometer ADC data collect
4.according 《物联网硬件平台-RFID通讯数据结构.pdf》 complete RFID data transfer and receive function
5.eeprom configure CC1101 address and sync code 
6.CC1101 initial address and sync code from eeprom
7.UART program eeprom，protocol is [0x41 0x42 0x43 0x44 0xxx address sync[1] sync[0]]
