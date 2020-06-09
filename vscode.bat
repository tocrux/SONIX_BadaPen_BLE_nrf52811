rem Location of Nordic SDK
set NRF_SDK=C:/Nordic/nRF5_SDK_16.0.0_98a08e2

rem Location of Nordic Command Line tools (nrfjprog) 
set NRF_TOOLS=C:/Program Files (x86)/Nordic Semiconductor/nrf-command-line-tools/bin

rem Location of GCC Cross-compiler https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads
set GNU_GCC=C:/Program Files (x86)/GNU Tools Arm Embedded/7 2018-q2-update/bin

rem Location of Gnu Tools (make) https://github.com/gnu-mcu-eclipse/windows-build-tools/releases
set GNU_TOOLS=C:/Nordic/GNU MCU Eclipse/Build Tools/2.12-20190422-1053/bin

rem Location of SEGGER JLink tools
set SEGGER_TOOLS=C:/Program Files (x86)/SEGGER/JLink_V512e

rem Location of java
set JAVA=C:/Program Files/Java/jdk1.8.0_231/bin/java.exe

rem Serial numbers of nRF development boards
set PCA10056e_SN=683142363

"C:/Program Files/Microsoft VS Code/Code.exe" ble_app_uart.code-workspace