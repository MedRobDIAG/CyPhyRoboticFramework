# MRframework

*1. Prerequisites*
==================

*1.1 Install dependencies*
--------------------------

*(Mandatory)*

- Eigen (tested with v3.3.8)
- CoppeliaSim (tested with v4.2.0)
- Visual Studio 2019 (tested with Platform toolset v142)

*(Optional - depending on the modules to be built)*

- Geomagic Touch device: OpenHaptics 3.5.0
- KUKA LWR robot platform: FastResearchInterface (FRI)
- Weart TouchDIVER device: Weart Middleware, Visual Studio UWP module
- H-Ring device: Polulu drivers


*1.2 Define Environment Variables*
----------------------------------

*(Mandatory)*

- EIGEN (e.g., "C:\Users\[UserName]\Software\eigen-3.3.8")
- VREPx64_coppelia (e.g. "C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu")

*(Optional - for Weart TouchDIVER device)*
- WINDOWS_WINMD_PATH (e.g.,"C:\Program Files (x86)\Windows Kits\10\UnionMetadata\10.0.19041.0" or similar, depending on your Windows SDK version)
- PLATFORM_WINMD_PATH (e.g., "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Tools\MSVC\14.29.30133\lib\x86\store\references" or similar, depending on your Windows SKD and toolset versions)

*(Optional - for OpenHaptics)*
- OPEN_HAPTICS (e.g., "C:\OpenHaptics\Developer\3.5.0")

*(Optional - for KUKA LWR robot platform)*
- FRI (e.g., "C:\Users\[UserName]\Software\FRILibrary\")


*1.3 Configure CoppeliaSim for multi-threading communication*
-------------------------------------------------------------
- Replace the file _remoteApiConnections.txt_ located in $(VREPx64_coppelia) with the one stored in this repository

*2 Build and run*
=================
- Open the directory where the _CMakeLists.txt_ of this repository is stored in Visual Studio
- Set the options WITH_XXX based on the modules you need to use (lines 5-10). IMPORTANT: Keep WITH_FORCE_SENSOR always ON
- Generate the output build files under the configuration _x86-Release_
- Open the generated Visual Studio solution _MRFramework.sln_ located in _${CMAKE_CURRENT_LIST_DIR}/out/build/x86-Release_
- Build the solution
- Copy the _config_ folder stored in this repository to the directory where the MRFramework.exe has been generated (i.e., _${CMAKE_CURRENT_LIST_DIR}/out/build/x86-Release/Release_)
- From the _lib_ folder, copy the dynamic library _LWR_Dynamic_Model_Lib.dll_ to the same directory
- Open CoppeliaSim with one of the scenes located in the _cs-scenes_ folder
- Run the MRFramework.exe


