set OPENOCD_PATH=C:\Users\c\.espressif\tools\openocd-esp32\v0.10.0-esp32-20210401\openocd-esp32
%OPENOCD_PATH%\bin\openocd.exe -s %OPENOCD_PATH%/share/openocd/scripts -f interface/ftdi/esp32s2_kaluga_v1.cfg -f target/esp32s2.cfg