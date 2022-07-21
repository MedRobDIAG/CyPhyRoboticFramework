# MRframework

*1. Prerequisites*
================

*1.1 Install dependencies*
------------------------

(Mandatory)

- Eigen (tested with v3.3.8)
- CoppeliaSim (tested with v4.2.0)
- Visual Studio 2019 (tested with Platform toolset v142)

(Optional - depending on the modules to be built)

- OpenHaptics 3.5.0 (to interface Geomagic Touch device)
- FastResearchInterface (FRI, to interface KUKA LWR robot)
- Weart Middleware (to interface Weart TouchDIVER device)
- Polulu drivers (to interface H-Ring device)


*1.2 Define Environment Variables*
--------------------------------

*(Mandatory)*

- EIGEN (e.g., "C:\Users\[UserName]\Software\eigen-3.3.8")
- VREPx64_coppelia (e.g. "C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu")

*(Optional)*

(for Weart TouchDIVER device)
- WINDOWS_WINMD_PATH (e.g.,"C:\Program Files (x86)\Windows Kits\10\UnionMetadata\10.0.19041.0" or similar, depending on your Windows SDK version)
- PLATFORM_WINMD_PATH (e.g., "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Tools\MSVC\14.29.30133\lib\x86\store\references" or similar, depending on your Windows SKD and toolset versions)

*(for OpenHaptics)*
- OPEN_HAPTICS (e.g., "C:\OpenHaptics\Developer\3.5.0")

*(for KUKA LWR robot platform)*
- FRI (e.g., "C:\Users\[UserNamee]\Software\FRILibrary\")


*1.3 Configure CoppeliaSim for multi-threading communication*
-----------------------------------------------------------
- Open the file _remoteApiConnections.txt_ located in $(VREPx64_coppelia)

