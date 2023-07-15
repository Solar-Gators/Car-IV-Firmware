## Environment Setup for Windows
#### Git Bash
1. Download the latest Git Bash setup from here: [https://git-scm.com/download/win](https://git-scm.com/download/win)
2. Run the installer
#### Make
1. Download `make-4.4.1-without-guile-w32-bin.zip` (or whatever the latest version is) from here: [https://sourceforge.net/projects/ezwinports/files/](https://sourceforge.net/projects/ezwinports/files/)
2. Extract the zip, then copy the contents into your `Git\mingw64\bin` directory (located at `C:\Program Files` if you used the default Git installation path). Make sure not to overwrite any existing files.
3. Run `make --version` in Git Bash to confirm make is successfully installed
#### Solar Gators SDK
1. Open Git Bash and run `git clone https://github.com/matlshen/Car-IV-Firmware.git`
2. Open `Firmware/nucleol476rg-base/Makefile` and change the host platform to `windows`
3. In Git Bash, run `make -j12`
4. Verify that the build completes and a .hex file appears in the `build` directory
#### J-Link Tools
1. Download the J-link software and documentation pack from here: [https://www.segger.com/downloads/jlink/](https://www.segger.com/downloads/jlink/)
2. If using a Nucleo board, follow the instructions here to convert on-board ST-Link into J-Link: [https://www.segger.com/products/debug-probes/j-link/models/other-j-links/st-link-on-board/](https://www.segger.com/products/debug-probes/j-link/models/other-j-links/st-link-on-board/)
3. Connect board to PC and open J-Flash