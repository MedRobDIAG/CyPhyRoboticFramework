cmake_minimum_required(VERSION 3.5)
project(MRframework VERSION 0.1 LANGUAGES C CXX)

# Set external system dependencies for each module
set(WITH_FRI OFF)
set(WITH_FORCE_SENSOR ON)
set(WITH_GEOMAGIC ON)
set(WITH_ARMBAND OFF)
set(WITH_HRING OFF)
set(WITH_WEART ON)

# CMake definitions
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(
    -DCRL_SUPPORT
    -DBUILD_KUKA_DYN_MODEL
    -DWIN32
    -DDO_NOT_USE_SHARED_MEMORY
    -D_CRT_SECURE_NO_WARNINGS
    -DNON_MATLAB_PARSING
    -DMAX_EXT_API_CONNECTIONS=255
    -DDEBUG
    -DNDEBUG
    -D_CONSOLE
)

# Boost 
#SET(CMAKE_INCLUDE_PATH $ENV{BOOST_1_66} ${CMAKE_INCLUDE_PATH})
#SET(CMAKE_LIBRARY_PATH $ENV{BOOST_1_66}\\stage\\lib $ENV{BOOST_1_66}\\..\\lib ${CMAKE_LIBRARY_PATH})
#set(Boost_USE_STATIC_LIBS ON) 
#set(Boost_USE_MULTITHREADED ON)  
#set(Boost_USE_STATIC_RUNTIME OFF) 
#set(Boost_NO_SYSTEM_PATHS FALSE)
#if (Boost_NO_SYSTEM_PATHS)
#  set(BOOST_ROOT $ENV{BOOST_1_66})
#  set(Boost_INCLUDE_DIR ${BOOST_ROOT})
#  set(Boost_LIBRARY_DIR ${BOOST_ROOT}stage\\lib)
#endif (Boost_NO_SYSTEM_PATHS)
#find_package(Boost REQUIRED date_time thread)

### Default include directories
include_directories(
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/include/remoteVREPx64
    ${CMAKE_CURRENT_LIST_DIR}/include/aProxy
    ${CMAKE_CURRENT_LIST_DIR}/include/bController
    ${CMAKE_CURRENT_LIST_DIR}/include/cEntity
    ${CMAKE_CURRENT_LIST_DIR}/include/dDatabase
    #${Boost_INCLUDE_DIRS}
    #$ENV{BOOST_1_66}
    $ENV{EIGEN}
    $ENV{VREPx64_coppelia}/programming/include/
    $ENV{VREPx64_coppelia}/programming/remoteApi/
)

# Set header directories for each module
set(KUKA_HEADERS ${CMAKE_CURRENT_LIST_DIR}/include/aProxy/KukaRobot/)
set(GEOMAGIC_HEADERS ${CMAKE_CURRENT_LIST_DIR}/include/aProxy/Geomagic/)
set(ARMBAND_HEADERS ${CMAKE_CURRENT_LIST_DIR}/include/aProxy/Armband/)
set(HRING_HEADERS ${CMAKE_CURRENT_LIST_DIR}/include/aProxy/HRing/)
set(WEART_HEADERS ${CMAKE_CURRENT_LIST_DIR}/include/aProxy/Weart)
set(FTSENSOR_HEADERS ${CMAKE_CURRENT_LIST_DIR}/include/aProxy/FTSensor/)

# Specify source files for each external module
set(KUKA_SRCS
     ${CMAKE_CURRENT_LIST_DIR}/src/aProxy/KukaRobot/*.cpp
     ${KUKA_HEADERS}/*.h*
)
set(GEOMAGIC_SRCS
    ${CMAKE_CURRENT_LIST_DIR}/src/aProxy/Geomagic/*.cpp
    ${GEOMAGIC_HEADERS}/*.h*
)
set(ARMBAND_SRCS
    ${CMAKE_CURRENT_LIST_DIR}/src/aProxy/Armband/*.cpp
    ${ARMBAND_HEADERS}/*.h*
)
set(HRING_SRCS
    ${CMAKE_CURRENT_LIST_DIR}/src/aProxy/HRing/*.cpp
    ${HRING_HEADERS}/*.h*
)
set(WEART_SRCS
    ${CMAKE_CURRENT_LIST_DIR}/src/aProxy/Weart/*.cpp
)
set(FTSENSOR_SRCS
    ${CMAKE_CURRENT_LIST_DIR}/src/aProxy/FTSensor/*.cpp
    ${FTSENSOR_HEADERS}/*.h*
)


if(WITH_FRI)
    include_directories(
        ${KUKA_HEADERS}
        $ENV{FRI}include
        $ENV{FRI}Windows\\pthread)
    set(SRCS_FILES ${SRCS_FILES} ${KUKA_SRCS})
    add_compile_definitions(WITH_FRI)
endif()
if(WITH_GEOMAGIC)
    include_directories(
        ${GEOMAGIC_HEADERS}
        $ENV{OPEN_HAPTICS}/include
        $ENV{OPEN_HAPTICS}/utilities/include
    )
    set(SRCS_FILES ${SRCS_FILES} ${GEOMAGIC_SRCS})
    add_compile_definitions(WITH_GEOMAGIC)
endif()
if(WITH_ARMBAND)
    include_directories(${ARMBAND_HEADERS})
    set(SRCS_FILES ${SRCS_FILES} ${ARMBAND_SRCS})
    add_compile_definitions(WITH_ARMBAND)
endif()
if(WITH_HRING)
    include_directories(${HRING_HEADERS})
    set(SRCS_FILES ${SRCS_FILES} ${HRING_SRCS})
    add_compile_definitions(WITH_HRING)
endif()
if(WITH_WEART)
    include_directories(${WEART_HEADERS} ${WEART_HEADERS}/WEART_SDK)
    set(SRCS_FILES ${SRCS_FILES} ${WEART_SRCS})
    add_compile_definitions(WITH_WEART)
endif()
if(WITH_FORCE_SENSOR)
    include_directories(${FTSENSOR_HEADERS})
    set(SRCS_FILES ${SRCS_FILES} ${FTSENSOR_SRCS})
    add_compile_definitions(WITH_FORCE_SENSOR)
endif()


### Collect the whole set of sources to be built
set(SRCS_FILES
    ${SRCS_FILES}
    ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/aProxy/*.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/bController/*.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/cEntity/*.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/dDatabase/*.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/remoteVREPx64/*.c
)
file(GLOB SRCS ${SRCS_FILES})

# Project compile definitions
add_compile_definitions(PROJECT_NAME)

message(${SRCS})
# Specify exe file to generate
add_executable(${PROJECT_NAME} ${SRCS})
#add_executable(${PROJECT_NAME} ${SRCS_FILES})

# Set compiling and linker options
#set_target_properties(${PROJECT_NAME} PROPERTIES COMMON_LANGUAGE_RUNTIME "")
target_compile_options(${PROJECT_NAME} PUBLIC "/Gy;/Oi;/sdl;/MP;/ZW;/AI$ENV{WINDOWS_WINMD_PATH};/AI$ENV{PLATFORM_WINMD_PATH}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /FORCE:MULTIPLE /SUBSYSTEM:CONSOLE /ENTRY:mainCRTStartup")

#### Collect the external library dependency list
set(EXT_LIBS ${EXT_LIBS}  
    #${Boost_LIBRARIES} 
    ${CMAKE_CURRENT_LIST_DIR}/lib/LWR_Dynamic_Model_Lib.lib)
if(WITH_FRI)
    #target_link_libraries(${PROJECT_NAME} 
    set(EXT_LIBS ${EXT_LIBS}
    $ENV{FRI}Windows\\pthread\\Release\\pthreadVC.lib
    $ENV{FRI}Windows\\Release\\TypeIRML.lib
    $ENV{FRI}Windows\\Release\\FastResearchLibrary.lib
    )
endif()
if(WITH_GEOMAGIC)
    link_directories(
        $ENV{OPEN_HAPTICS}/lib/x86/Release
        $ENV{OPEN_HAPTICS}/utilities/lib/x86/Release
    )
    set(EXT_LIBS ${EXT_LIBS}
    $ENV{OPEN_HAPTICS}\\lib\\Win32\\Release\\hd.lib
    $ENV{OPEN_HAPTICS}\\lib\\Win32\\Release\\hl.lib
    $ENV{OPEN_HAPTICS}\\utilities\\lib\\Win32\\Release\\hdu.lib
    $ENV{OPEN_HAPTICS}\\utilities\\lib\\Win32\\Release\\hlu.lib
    $ENV{OPEN_HAPTICS}\\utilities\\lib\\Win32\\Release\\glut32.lib
    $ENV{OPEN_HAPTICS}\\utilities\\lib\\Win32\\Release\\glui32.lib
    )
endif()
if(WITH_ARMBAND)
    link_directories(${ARMBAND_HEADERS})
endif()
if(WITH_HRING)
    link_directories(${HRING_HEADERS})
endif()
if(WITH_FORCE_SENSOR)
    set(EXT_LIBS ${EXT_LIBS}
        ${CMAKE_CURRENT_LIST_DIR}/lib/crlFTSensorWrapper.lib)
endif()

### Link the list of libraries
target_link_libraries(${PROJECT_NAME} ${EXT_LIBS})

