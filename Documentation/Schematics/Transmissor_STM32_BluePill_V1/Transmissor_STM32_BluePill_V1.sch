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
L Transmissor_STM32_BluePill_V1-rescue:YAAJ_BluePill_Part_Like_SWD_Breakout-YAAJ_BluePill_Part_Like_SWD_Breakout U?
U 1 1 5FDF5A20
P 5800 2500
F 0 "U?" H 5800 3665 50  0000 C CNN
F 1 "YAAJ_BluePill_Part_Like_SWD_Breakout" H 5800 3574 50  0000 C CNN
F 2 "" H 6600 1500 50  0001 C CNN
F 3 "" H 6600 1500 50  0001 C CNN
	1    5800 2500
	1    0    0    -1  
$EndComp
$Comp
L Transmissor_STM32_BluePill_V1-rescue:HC-SR04-HC-SR04 U?
U 1 1 5FE0331E
P 2800 6700
F 0 "U?" H 3230 6696 50  0000 L CNN
F 1 "HC-SR04" H 3230 6605 50  0000 L CNN
F 2 "XCVR_HC-SR04" H 2800 6700 50  0001 L BNN
F 3 "" H 2800 6700 50  0001 L BNN
F 4 "Osepp" H 2800 6700 50  0001 L BNN "MANUFACTURER"
	1    2800 6700
	1    0    0    -1  
$EndComp
$Comp
L Transmissor_STM32_BluePill_V1-rescue:MCP1700-3302E_TO-dk_PMIC-Voltage-Regulators-Linear U?
U 1 1 5FE05EBD
P 3150 5300
F 0 "U?" H 3150 5587 60  0000 C CNN
F 1 "RT9080" H 3150 5481 60  0000 C CNN
F 2 "digikey-footprints:TO-92-3" H 3350 5500 60  0001 L CNN
F 3 "http://www.microchip.com/mymicrochip/filehandler.aspx?ddocname=en011779" H 3350 5600 60  0001 L CNN
F 4 "MCP1700-3302E/TO-ND" H 3350 5700 60  0001 L CNN "Digi-Key_PN"
F 5 "MCP1700-3302E/TO" H 3350 5800 60  0001 L CNN "MPN"
F 6 "Integrated Circuits (ICs)" H 3350 5900 60  0001 L CNN "Category"
F 7 "PMIC - Voltage Regulators - Linear" H 3350 6000 60  0001 L CNN "Family"
F 8 "http://www.microchip.com/mymicrochip/filehandler.aspx?ddocname=en011779" H 3350 6100 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/microchip-technology/MCP1700-3302E-TO/MCP1700-3302E-TO-ND/652680" H 3350 6200 60  0001 L CNN "DK_Detail_Page"
F 10 "IC REG LINEAR 3.3V 250MA TO92-3" H 3350 6300 60  0001 L CNN "Description"
F 11 "Microchip Technology" H 3350 6400 60  0001 L CNN "Manufacturer"
F 12 "Active" H 3350 6500 60  0001 L CNN "Status"
	1    3150 5300
	1    0    0    -1  
$EndComp
$Comp
L Transmissor_STM32_BluePill_V1-rescue:AP2112K-3.3-Regulator_Linear U?
U 1 1 5FE08CC3
P 2150 5400
F 0 "U?" H 2150 5742 50  0000 C CNN
F 1 "Mini360-Alterado" H 2150 5651 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 2150 5725 50  0001 C CNN
F 3 "https://www.diodes.com/assets/Datasheets/AP2112.pdf" H 2150 5500 50  0001 C CNN
	1    2150 5400
	1    0    0    -1  
$EndComp
$Comp
L Transmissor_STM32_BluePill_V1-rescue:LD1117V33-dk_PMIC-Voltage-Regulators-Linear U?
U 1 1 5FE11EC5
P 1600 3050
F 0 "U?" H 1600 3337 60  0000 C CNN
F 1 "TS9011" H 1600 3231 60  0000 C CNN
F 2 "digikey-footprints:TO-220-3" H 1800 3250 60  0001 L CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/99/3b/7d/91/91/51/4b/be/CD00000544.pdf/files/CD00000544.pdf/jcr:content/translations/en.CD00000544.pdf" H 1800 3350 60  0001 L CNN
F 4 "497-1491-5-ND" H 1800 3450 60  0001 L CNN "Digi-Key_PN"
F 5 "LD1117V33" H 1800 3550 60  0001 L CNN "MPN"
F 6 "Integrated Circuits (ICs)" H 1800 3650 60  0001 L CNN "Category"
F 7 "PMIC - Voltage Regulators - Linear" H 1800 3750 60  0001 L CNN "Family"
F 8 "http://www.st.com/content/ccc/resource/technical/document/datasheet/99/3b/7d/91/91/51/4b/be/CD00000544.pdf/files/CD00000544.pdf/jcr:content/translations/en.CD00000544.pdf" H 1800 3850 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/stmicroelectronics/LD1117V33/497-1491-5-ND/586012" H 1800 3950 60  0001 L CNN "DK_Detail_Page"
F 10 "IC REG LINEAR 3.3V 800MA TO220AB" H 1800 4050 60  0001 L CNN "Description"
F 11 "STMicroelectronics" H 1800 4150 60  0001 L CNN "Manufacturer"
F 12 "Active" H 1800 4250 60  0001 L CNN "Status"
	1    1600 3050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5FE17183
P 5000 5050
F 0 "#PWR?" H 5000 4800 50  0001 C CNN
F 1 "GND" V 5005 4922 50  0000 R CNN
F 2 "" H 5000 5050 50  0001 C CNN
F 3 "" H 5000 5050 50  0001 C CNN
	1    5000 5050
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5FE197B4
P 5800 5650
F 0 "#PWR?" H 5800 5400 50  0001 C CNN
F 1 "GND" V 5805 5522 50  0000 R CNN
F 2 "" H 5800 5650 50  0001 C CNN
F 3 "" H 5800 5650 50  0001 C CNN
	1    5800 5650
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5FE1B1F7
P 5000 5750
F 0 "#PWR?" H 5000 5500 50  0001 C CNN
F 1 "GND" V 5005 5622 50  0000 R CNN
F 2 "" H 5000 5750 50  0001 C CNN
F 3 "" H 5000 5750 50  0001 C CNN
	1    5000 5750
	0    1    1    0   
$EndComp
Text GLabel 5000 5150 0    50   BiDi ~ 0
LORA_MISO
Text GLabel 6700 2500 2    50   BiDi ~ 0
LORA_MISO
Text GLabel 5000 5250 0    50   BiDi ~ 0
LORA_MOSI
Text GLabel 6700 2400 2    50   BiDi ~ 0
LORA_MOSI
Text GLabel 6700 2600 2    50   BiDi ~ 0
LORA_SCK
Text GLabel 5000 5350 0    50   BiDi ~ 0
LORA_SCK
Text GLabel 5000 5450 0    50   BiDi ~ 0
LORA_NSS
Text GLabel 5000 5550 0    50   BiDi ~ 0
LORA_RESET
Text GLabel 5800 5250 2    50   BiDi ~ 0
LORA_DIO0
NoConn ~ 5800 5150
NoConn ~ 5800 5450
NoConn ~ 5800 5550
NoConn ~ 5000 5650
Text GLabel 6700 2700 2    50   BiDi ~ 0
LORA_NSS
Text GLabel 6700 3100 2    50   BiDi ~ 0
LORA_RESET
Text GLabel 6700 3000 2    50   BiDi ~ 0
LORA_DIO0
$Comp
L power:+3V3 #PWR?
U 1 1 5FE20586
P 6700 1800
F 0 "#PWR?" H 6700 1650 50  0001 C CNN
F 1 "+3V3" V 6715 1928 50  0000 L CNN
F 2 "" H 6700 1800 50  0001 C CNN
F 3 "" H 6700 1800 50  0001 C CNN
	1    6700 1800
	0    1    1    0   
$EndComp
$Comp
L power:+3V3 #PWR?
U 1 1 5FE22266
P 1900 3050
F 0 "#PWR?" H 1900 2900 50  0001 C CNN
F 1 "+3V3" V 1915 3178 50  0000 L CNN
F 2 "" H 1900 3050 50  0001 C CNN
F 3 "" H 1900 3050 50  0001 C CNN
	1    1900 3050
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5FE25722
P 2150 5700
F 0 "#PWR?" H 2150 5450 50  0001 C CNN
F 1 "GND" H 2155 5527 50  0000 C CNN
F 2 "" H 2150 5700 50  0001 C CNN
F 3 "" H 2150 5700 50  0001 C CNN
	1    2150 5700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5FE26161
P 3150 5600
F 0 "#PWR?" H 3150 5350 50  0001 C CNN
F 1 "GND" H 3155 5427 50  0000 C CNN
F 2 "" H 3150 5600 50  0001 C CNN
F 3 "" H 3150 5600 50  0001 C CNN
	1    3150 5600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5FE271D8
P 1600 3350
F 0 "#PWR?" H 1600 3100 50  0001 C CNN
F 1 "GND" H 1605 3177 50  0000 C CNN
F 2 "" H 1600 3350 50  0001 C CNN
F 3 "" H 1600 3350 50  0001 C CNN
	1    1600 3350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5FE281F7
P 6700 1600
F 0 "#PWR?" H 6700 1350 50  0001 C CNN
F 1 "GND" V 6705 1472 50  0000 R CNN
F 2 "" H 6700 1600 50  0001 C CNN
F 3 "" H 6700 1600 50  0001 C CNN
	1    6700 1600
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5FE293F1
P 6700 1700
F 0 "#PWR?" H 6700 1450 50  0001 C CNN
F 1 "GND" V 6705 1572 50  0000 R CNN
F 2 "" H 6700 1700 50  0001 C CNN
F 3 "" H 6700 1700 50  0001 C CNN
	1    6700 1700
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5FE2985B
P 4900 3400
F 0 "#PWR?" H 4900 3150 50  0001 C CNN
F 1 "GND" V 4905 3272 50  0000 R CNN
F 2 "" H 4900 3400 50  0001 C CNN
F 3 "" H 4900 3400 50  0001 C CNN
	1    4900 3400
	0    1    1    0   
$EndComp
$Comp
L power:+3V3 #PWR?
U 1 1 5FE2A7B2
P 4900 3500
F 0 "#PWR?" H 4900 3350 50  0001 C CNN
F 1 "+3V3" V 4915 3628 50  0000 L CNN
F 2 "" H 4900 3500 50  0001 C CNN
F 3 "" H 4900 3500 50  0001 C CNN
	1    4900 3500
	0    -1   -1   0   
$EndComp
NoConn ~ 4900 3300
NoConn ~ 6700 3500
NoConn ~ 5600 3800
NoConn ~ 5900 3800
NoConn ~ 5700 3800
NoConn ~ 5800 3800
$Comp
L power:GND #PWR?
U 1 1 5FE2D41B
P 2600 6900
F 0 "#PWR?" H 2600 6650 50  0001 C CNN
F 1 "GND" V 2605 6772 50  0000 R CNN
F 2 "" H 2600 6900 50  0001 C CNN
F 3 "" H 2600 6900 50  0001 C CNN
	1    2600 6900
	0    1    1    0   
$EndComp
Text GLabel 2600 6700 0    50   BiDi ~ 0
US_TRIG
Text GLabel 2600 6800 0    50   BiDi ~ 0
US_ECHO
Text GLabel 6700 2800 2    50   BiDi ~ 0
US_TRIG
Text GLabel 6700 2900 2    50   BiDi ~ 0
US_ECHO
$Comp
L Device:Antenna AE?
U 1 1 5FE2F824
P 6000 5750
F 0 "AE?" V 5954 5880 50  0000 L CNN
F 1 "Antenna" V 6045 5880 50  0000 L CNN
F 2 "" H 6000 5750 50  0001 C CNN
F 3 "~" H 6000 5750 50  0001 C CNN
	1    6000 5750
	0    1    1    0   
$EndComp
$Comp
L Transmissor_STM32_BluePill_V1-rescue:FTDI_6_PIN_SERIAL_CABLEPTH_ilha3d-KICAD_ILHA3D_lIBRARY J?
U 1 1 5FE31B47
P 10250 5500
F 0 "J?" H 10258 6310 45  0000 C CNN
F 1 "FTDI_6_PIN_SERIAL_CABLEPTH_ilha3d" H 10258 6226 45  0000 C CNN
F 2 "KICAD_ILHA3D_lIBRARY:FTDI_6_PIN_SERIAL_CABLEPTH_ilha3d" H 10250 6200 20  0001 C CNN
F 3 "" H 10250 5500 50  0001 C CNN
F 4 "XXX-00000" H 10258 6131 60  0000 C CNN "Field4"
	1    10250 5500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5FE343BE
P 10450 5000
F 0 "#PWR?" H 10450 4750 50  0001 C CNN
F 1 "GND" V 10455 4872 50  0000 R CNN
F 2 "" H 10450 5000 50  0001 C CNN
F 3 "" H 10450 5000 50  0001 C CNN
	1    10450 5000
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 5FE35180
P 10450 5200
F 0 "#PWR?" H 10450 5050 50  0001 C CNN
F 1 "+5V" V 10465 5328 50  0000 L CNN
F 2 "" H 10450 5200 50  0001 C CNN
F 3 "" H 10450 5200 50  0001 C CNN
	1    10450 5200
	0    1    1    0   
$EndComp
NoConn ~ 10450 5100
NoConn ~ 10450 5500
Text GLabel 10450 5300 2    50   BiDi ~ 0
FTDI_TX
Text GLabel 10450 5400 2    50   BiDi ~ 0
FTDI_RX
Text GLabel 4900 1700 0    50   BiDi ~ 0
PWR_Lora
Wire Wire Line
	2450 5300 2600 5300
Wire Wire Line
	2600 6600 2600 6300
Connection ~ 2600 5300
Wire Wire Line
	2600 5300 2850 5300
Wire Wire Line
	5800 5350 6600 5350
Wire Wire Line
	6600 5350 6600 4550
Wire Wire Line
	6600 4550 3950 4550
Wire Wire Line
	3950 4550 3950 5300
Wire Wire Line
	3950 5300 3450 5300
$Comp
L Transmissor_STM32_BluePill_V1-rescue:Jack-DC-KICAD_ILHA3D_lIBRARY J?
U 1 1 5FE5DCAA
P 1550 2300
F 0 "J?" H 1321 2258 50  0000 R CNN
F 1 "Jack-DC" H 1321 2349 50  0000 R CNN
F 2 "KICAD_ILHA3D_lIBRARY:Barrel_Jack_ilha3d" H 1600 2260 50  0001 C CNN
F 3 "~" H 1600 2260 50  0001 C CNN
	1    1550 2300
	1    0    0    1   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5FE5FC81
P 1850 2400
F 0 "#PWR?" H 1850 2150 50  0001 C CNN
F 1 "GND" V 1855 2272 50  0000 R CNN
F 2 "" H 1850 2400 50  0001 C CNN
F 3 "" H 1850 2400 50  0001 C CNN
	1    1850 2400
	0    -1   -1   0   
$EndComp
$Comp
L power:+12V #PWR?
U 1 1 5FE61E72
P 1850 2200
F 0 "#PWR?" H 1850 2050 50  0001 C CNN
F 1 "+12V" V 1865 2328 50  0000 L CNN
F 2 "" H 1850 2200 50  0001 C CNN
F 3 "" H 1850 2200 50  0001 C CNN
	1    1850 2200
	0    1    1    0   
$EndComp
$Comp
L power:+12V #PWR?
U 1 1 5FE63668
P 1300 3050
F 0 "#PWR?" H 1300 2900 50  0001 C CNN
F 1 "+12V" V 1315 3178 50  0000 L CNN
F 2 "" H 1300 3050 50  0001 C CNN
F 3 "" H 1300 3050 50  0001 C CNN
	1    1300 3050
	0    -1   -1   0   
$EndComp
$Comp
L power:+12V #PWR?
U 1 1 5FE64C73
P 1850 5300
F 0 "#PWR?" H 1850 5150 50  0001 C CNN
F 1 "+12V" V 1865 5428 50  0000 L CNN
F 2 "" H 1850 5300 50  0001 C CNN
F 3 "" H 1850 5300 50  0001 C CNN
	1    1850 5300
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_Push SW1
U 1 1 5FE67B41
P 9550 1550
F 0 "SW1" V 9504 1698 50  0000 L CNN
F 1 "RESET" V 9595 1698 50  0000 L CNN
F 2 "" H 9550 1750 50  0001 C CNN
F 3 "~" H 9550 1750 50  0001 C CNN
	1    9550 1550
	0    1    1    0   
$EndComp
Text GLabel 6700 1900 2    50   BiDi ~ 0
STM32_RST
Text GLabel 9550 1350 0    50   BiDi ~ 0
STM32_RST
$Comp
L power:GND #PWR?
U 1 1 5FE6AC32
P 9550 1750
F 0 "#PWR?" H 9550 1500 50  0001 C CNN
F 1 "GND" H 9555 1577 50  0000 C CNN
F 2 "" H 9550 1750 50  0001 C CNN
F 3 "" H 9550 1750 50  0001 C CNN
	1    9550 1750
	1    0    0    -1  
$EndComp
Text GLabel 4900 2100 0    50   BiDi ~ 0
FTDI_RX
Text GLabel 4900 2200 0    50   BiDi ~ 0
FTDI_TX
$Comp
L Transmissor_STM32_BluePill_V1-rescue:RFM9X-S-RFM9X U?
U 1 1 5FE0042E
P 5400 5450
F 0 "U?" H 5400 6137 60  0000 C CNN
F 1 "RFM9X-S" H 5400 6031 60  0000 C CNN
F 2 "" H 5400 5450 60  0001 C CNN
F 3 "" H 5400 5450 60  0001 C CNN
	1    5400 5450
	1    0    0    -1  
$EndComp
NoConn ~ 5800 5050
Text Notes 1250 2000 0    50   ~ 0
V Fonte Max = 15V
Text Notes 2300 3550 0    50   ~ 0
Só Alimenta o STM32\n(Será trocado por um\nTS9011SCY ou NCP551SN33T1G\n\nAMS1117: imax = 1A\nVimax = 15V\nVo = 3.3V\niq=5mA\nquando chegarem)
Text Notes 9450 5800 0    50   ~ 0
ATENÇÃO: Para programar com a fonte\nde Alim externa não usar o 5V nem 3V3\nLigar somente (TX, RX e GND)
$Comp
L Device:R R?
U 1 1 5FF6110F
P 1300 5050
F 0 "R?" H 1370 5096 50  0000 L CNN
F 1 "4K7" H 1370 5005 50  0000 L CNN
F 2 "" V 1230 5050 50  0001 C CNN
F 3 "~" H 1300 5050 50  0001 C CNN
	1    1300 5050
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 5FF61E13
P 1300 5650
F 0 "R?" H 1370 5696 50  0000 L CNN
F 1 "100K" H 1370 5605 50  0000 L CNN
F 2 "" V 1230 5650 50  0001 C CNN
F 3 "~" H 1300 5650 50  0001 C CNN
	1    1300 5650
	1    0    0    -1  
$EndComp
Wire Wire Line
	1300 5200 1300 5400
Wire Wire Line
	1850 5400 1300 5400
Connection ~ 1300 5400
Wire Wire Line
	1300 5400 1300 5500
$Comp
L power:GND #PWR?
U 1 1 5FF632D2
P 1300 5800
F 0 "#PWR?" H 1300 5550 50  0001 C CNN
F 1 "GND" H 1305 5627 50  0000 C CNN
F 2 "" H 1300 5800 50  0001 C CNN
F 3 "" H 1300 5800 50  0001 C CNN
	1    1300 5800
	1    0    0    -1  
$EndComp
Text Notes 900  5500 0    50   ~ 0
V=3,15V\ni=30,42uA
Text Notes 1750 4900 0    50   ~ 0
Mini360: Vin = 12 -> 15V\nimax = 1.8A\niq = 15mA\nTrocar Trimpot por (10K +20K + 6K8)\nEn shut em 16ms
Text Notes 2950 6150 0    50   ~ 0
MCP1700: Vin 2.3 - 6V\niq=1.6uA\nVo=3.3V
$Comp
L Transmissor_STM32_BluePill_V1-rescue:SIM800L-sim800l GSM_Module?
U 1 1 6011D998
P 9450 3100
F 0 "GSM_Module?" H 9475 3987 60  0000 C CNN
F 1 "SIM800L" H 9475 3881 60  0000 C CNN
F 2 "" H 9450 3100 60  0001 C CNN
F 3 "" H 9450 3100 60  0001 C CNN
	1    9450 3100
	1    0    0    -1  
$EndComp
NoConn ~ 10650 3050
NoConn ~ 10650 3150
NoConn ~ 10650 3250
NoConn ~ 10650 3350
NoConn ~ 10650 3450
NoConn ~ 10650 3550
$Comp
L power:GND #PWR?
U 1 1 6011EC3E
P 8200 3500
F 0 "#PWR?" H 8200 3250 50  0001 C CNN
F 1 "GND" H 8205 3327 50  0000 C CNN
F 2 "" H 8200 3500 50  0001 C CNN
F 3 "" H 8200 3500 50  0001 C CNN
	1    8200 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	8200 3350 8200 3500
Wire Wire Line
	8200 3350 8300 3350
Text GLabel 8300 3250 0    50   Output ~ 0
SIM800L_TX
Text GLabel 8300 3150 0    50   Output ~ 0
SIM800L_RX
NoConn ~ 8300 3050
NoConn ~ 8300 2850
Text GLabel 6700 2100 2    50   Input ~ 0
SIM800L_RX
Text GLabel 6700 2000 2    50   Output ~ 0
SIM800L_TX
Text GLabel 4900 1600 0    50   BiDi ~ 0
PWR_SIM800L
$Comp
L Transmissor_STM32_BluePill_V1-rescue:Tiny_RTC_DS1307-crumpschemes U?
U 1 1 60124A52
P 8050 5250
F 0 "U?" H 8050 5937 60  0000 C CNN
F 1 "Tiny_RTC_DS1307" H 8050 5831 60  0000 C CNN
F 2 "" H 8100 5350 60  0000 C CNN
F 3 "" H 8100 5350 60  0000 C CNN
	1    8050 5250
	1    0    0    -1  
$EndComp
Text GLabel 7550 5100 0    50   BiDi ~ 0
RTC_SDA
Text GLabel 7550 5000 0    50   BiDi ~ 0
RTC_SCL
Text GLabel 4900 2900 0    50   BiDi ~ 0
RTC_SCL
Text GLabel 4900 3000 0    50   BiDi ~ 0
RTC_SDA
$Comp
L power:GND #PWR?
U 1 1 6012A0BD
P 7550 5300
F 0 "#PWR?" H 7550 5050 50  0001 C CNN
F 1 "GND" H 7555 5127 50  0000 C CNN
F 2 "" H 7550 5300 50  0001 C CNN
F 3 "" H 7550 5300 50  0001 C CNN
	1    7550 5300
	1    0    0    -1  
$EndComp
NoConn ~ 8550 4900
NoConn ~ 8550 5000
NoConn ~ 8550 5100
NoConn ~ 8550 5200
NoConn ~ 8550 5300
NoConn ~ 8550 5400
NoConn ~ 8550 5500
NoConn ~ 7550 4900
Text GLabel 1300 4900 1    50   BiDi ~ 0
PWR_Lora
Wire Wire Line
	2600 6300 7200 6300
Wire Wire Line
	7200 6300 7200 5200
Wire Wire Line
	7200 5200 7550 5200
Connection ~ 2600 6300
Wire Wire Line
	2600 6300 2600 5300
Text Label 2550 5250 0    50   ~ 0
+5VDC
NoConn ~ 4900 3100
NoConn ~ 4900 3200
$Comp
L Switch:SW_Push SW?
U 1 1 606B5E93
P 10450 1550
F 0 "SW?" V 10404 1698 50  0000 L CNN
F 1 "RESET" V 10495 1698 50  0000 L CNN
F 2 "" H 10450 1750 50  0001 C CNN
F 3 "~" H 10450 1750 50  0001 C CNN
	1    10450 1550
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 606B646D
P 10450 1750
F 0 "#PWR?" H 10450 1500 50  0001 C CNN
F 1 "GND" H 10455 1577 50  0000 C CNN
F 2 "" H 10450 1750 50  0001 C CNN
F 3 "" H 10450 1750 50  0001 C CNN
	1    10450 1750
	1    0    0    -1  
$EndComp
Text GLabel 10450 1350 0    50   Output ~ 0
Teste
Text GLabel 4900 2000 0    50   Input ~ 0
Teste
$Comp
L Sensor_Temperature:DS18B20 U?
U 1 1 606B79B6
P 7850 1100
F 0 "U?" H 7620 1146 50  0000 R CNN
F 1 "DS18B20" H 7620 1055 50  0000 R CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 6850 850 50  0001 C CNN
F 3 "http://datasheets.maximintegrated.com/en/ds/DS18B20.pdf" H 7700 1350 50  0001 C CNN
	1    7850 1100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 606BA629
P 7850 1400
F 0 "#PWR?" H 7850 1150 50  0001 C CNN
F 1 "GND" H 7855 1227 50  0000 C CNN
F 2 "" H 7850 1400 50  0001 C CNN
F 3 "" H 7850 1400 50  0001 C CNN
	1    7850 1400
	1    0    0    -1  
$EndComp
Text GLabel 6700 2300 2    50   BiDi ~ 0
Sensor_Temp
Text GLabel 8150 1100 2    50   BiDi ~ 0
Sensor_Temp
Text GLabel 2600 6300 0    50   BiDi ~ 0
+5VDC
Text GLabel 7850 650  0    50   BiDi ~ 0
+5VDC
Wire Wire Line
	7850 650  7850 800 
Text GLabel 6700 2200 2    50   BiDi ~ 0
EC_Meter
$Comp
L Connector:Conn_01x03_Female J?
U 1 1 606BDD74
P 3000 1150
F 0 "J?" H 3028 1176 50  0000 L CNN
F 1 "EC Meter v1.0" H 3028 1085 50  0000 L CNN
F 2 "" H 3000 1150 50  0001 C CNN
F 3 "~" H 3000 1150 50  0001 C CNN
	1    3000 1150
	1    0    0    -1  
$EndComp
Text GLabel 2800 1150 0    50   BiDi ~ 0
+5VDC
$Comp
L power:GND #PWR?
U 1 1 606C0697
P 2800 1250
F 0 "#PWR?" H 2800 1000 50  0001 C CNN
F 1 "GND" H 2805 1077 50  0000 C CNN
F 2 "" H 2800 1250 50  0001 C CNN
F 3 "" H 2800 1250 50  0001 C CNN
	1    2800 1250
	1    0    0    -1  
$EndComp
Text GLabel 2800 1050 0    50   BiDi ~ 0
EC_Meter
Wire Notes Line
	2050 900  3800 900 
Wire Notes Line
	3800 900  3800 1800
Wire Notes Line
	3800 1800 2050 1800
Wire Notes Line
	2050 1800 2050 900 
Text Notes 2900 1750 0    50   ~ 10
Sensor Condutividade\nEC Meter V1.0
$EndSCHEMATC
