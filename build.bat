@echo off
 
cd C:\Users\idea\Desktop\nRF52 DK\nRF5_SDK_11.0.0_89a8197\examples\ble_peripheral\ble_app_hrs\pca10040\s132_with_dfu\arm5_no_packs\_build



::"C:\Keil_v5\ARM\ARMCC\bin\hex2bin.exe" -s 0x1C000 "C:\Users\Teamcity_result\ble_app_pass\%verstr%\%verstr%.hex"

"C:\Program Files (x86)\Nordic Semiconductor\nRFgo Studio\nrfutil.exe" dfu genpkg --application "s132_pca10040.hex" "dfu_package.zip"
pause

