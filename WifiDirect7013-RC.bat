@echo off
echo !!!!! 1. Make sure to connect your computer to your phone through wifi direct !!!!!
echo !!!!! 2. Make sure to connect your phone to PC with USB !!!!!
echo !!!!! 3. Make sure to restart Android Studio and open logcat window !!!!!
echo !!!!! 4. Make sure to disconnect from VPN !!!!!
echo !!!!! 5. Make sure to run this Batch file as Administrator !!!!!
echo 
REM 
REM Use this Batch file to wirelessly conenct to an Android Phone for code development
REM This assumes that you are connecting your computer to your phone through wifi direct, and that you have  
REM locked in it's IP addresse. Use "ipconfig /all" to see the ipv4 address for wireless LAN Gateway.
REM Edit the lines at the end of this batch file to match the IP addresses assigned to the phone.
REM
REM *****  For more help, go to www.YouTube.com/user/GEARSinc/playlists
REM
echo --  Starting ADB
adb kill-server
echo --  Make sure Robot Controller is connected to the Router.
echo --  Ensure that the IP matches this script
echo --  ALSO, make sure the phone is connected to the computer via USB
set /p ok= --  Hit enter when phone has been plugged in and recognized: 
adb usb
Timeout 10
adb tcpip 5555
Timeout 10
REM -----   Edit this line to have your Phones IP
adb connect 192.168.49.1:5555
adb devices
Timeout 5
adb connect 192.168.49.1:5555
adb devices
set /p ok= --  Unplug the phone and hit Enter to see the final connection.
adb connect 192.168.49.1:5555
adb devices
Timeout 5
echo -- Choose the other connection with port 5555
adb connect 192.168.49.1:5555
adb devices
Timeout 5



