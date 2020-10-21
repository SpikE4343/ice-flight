EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L RF_Module:ESP32-PICO-D4 U2
U 1 1 5EFD0728
P 6250 3250
F 0 "U2" H 6250 1561 50  0000 C CNN
F 1 "ESP32-PICO-D4" H 6250 1470 50  0000 C CNN
F 2 "Package_DFN_QFN:QFN-48-1EP_7x7mm_P0.5mm_EP5.3x5.3mm" H 6250 1550 50  0001 C CNN
F 3 "https://www.espressif.com/sites/default/files/documentation/esp32-pico-d4_datasheet_en.pdf" H 6500 2250 50  0001 C CNN
	1    6250 3250
	1    0    0    -1  
$EndComp
$Comp
L Sensor_Motion:MPU-9250 U1
U 1 1 5EFD0D33
P 2600 2550
F 0 "U1" H 2600 1561 50  0000 C CNN
F 1 "MPU-9250" H 2600 1470 50  0000 C CNN
F 2 "Sensor_Motion:InvenSense_QFN-24_3x3mm_P0.4mm" H 2600 1550 50  0001 C CNN
F 3 "https://store.invensense.com/datasheets/invensense/MPU9250REV1.0.pdf" H 2600 2400 50  0001 C CNN
	1    2600 2550
	1    0    0    -1  
$EndComp
$EndSCHEMATC
