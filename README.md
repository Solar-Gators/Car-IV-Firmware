![workflow badge](https://github.com/Solar-Gators/Car-IV-Firmware/actions/workflows/nucleol476rg-CAN-dev.yml/badge.svg)
## Environment Setup for Windows
#### Download Tools
1. [Install Visual Studio Code](https://code.visualstudio.com/download). Recommended to also install the C/C++ Extension Pack, CMake Tools, Gitlens, and Hex Editor extensions.
2. [Install Git for Windows](https://gitforwindows.org/)
	- Optionally select Visual Studio Code as Git's default editor during installation
	- Override the default branch name for new repositories to "main"
	- Leave all other installation settings as default
1. [Install MSYS2](https://www.msys2.org/). MSYS2 provides Windows-native development tools like compilers and build systems.
	- Optionally use MSYS2 as the default integrated terminal in VSCode
2. [Install J-Link Tools](https://www.segger.com/downloads/jlink/). These utilities interface with Segger J-Link probes for programming and debugging.
    - If using a Nucleo board, follow the instructions [here](https://www.segger.com/products/debug-probes/j-link/models/other-j-links/st-link-on-board/) to convert on-board ST-Link into J-Link
3. [Install Segger Ozone](https://www.segger.com/downloads/jlink/#Ozone). This is a graphical debugging utility that works with J-Link.
4. [Install STM32CubeMX](https://www.st.com/content/st_com/en/stm32cubemx.html#get_started_container). This is a graphical tool used to generate initialization code for STM32s. It also contains all the STM32 firmware libraries.
#### Set up Build System
1. Open `C:\msys64\home\${USER}\.bashrc` and add the following line at the end:
	```bash
	export PATH=$PATH:/mingw64/bin:"/c/Program Files/Git/cmd"
	```
2. Open MSYS2 and run the following commands:
	```bash
	pacman -Sy
	pacman -S mingw-w64-x86_64-arm-none-eabi-{binutils,gdb,gcc,newlib} mingw-w64-x86_64-{cmake,ninja}
	```
	This will install the ARM toolchain (arm-none-eabi-\*) and build tools (CMake, Ninja)
3. Confirm that everything installed properly:
	```bash
	git --version
	arm-none-eabi-gcc --version
	cmake --version
	ninja --version
	```
	MSYS2 will be your primary terminal for Solar Gators projects. Optionally you can add the /bin folders to your system PATH variable to run the necessary utilities from the Windows Command Prompt or PowerShell. You can also add the MSYS2 terminal to the Windows Terminal app or the integrated terminal in VSCode.
4. Set the default CMake generator to Ninja by creating an environment variable called `CMAKE_GENERATOR` and setting its value to `Ninja`
#### Build and Run Example Project
1. Run `git clone https://github.com/matlshen/Car-IV-Firmware.git --recursive`
2. Navigate to `Firmware/nucleol476rg-blinky/` and run `cmake -S . -B build` to generate the build system. To build the executable, run either `cmake --build build` or `ninja` from the `build` directory. Verify that the build completes without errors and a .hex file appears in the `build` directory.
3. Open J-Flash Lite, erase the board, then program the .hex file onto the board. Press the black reset button and confirm that the green LED is blinking.

## Optional
#### Use MSYS2 as Integrated Terminal in VSCode
1. In VSCode type `ctrl+shift+p` to bring up the Command Palette and type `Preferences: Open User Settings (JSON)` to open the `settings.json` file
2. Add the following to `"terminal.integrated.profiles.windows"`:
	```json
	"MSYS2": {
		"path": "C:\\msys64\\usr\\bin\\bash.exe",
		"args": [
			"--login",
			"-i"
		],
		"env": {
			"CHERE_INVOKING": "1"
		}
	}
	```
	Change `"terminal.integrated.defaultProfile.windows"` to `"MSYS2"`