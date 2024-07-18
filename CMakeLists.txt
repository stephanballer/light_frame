cmake_minimum_required(VERSION 3.10)
project(BacklightCapture)
set(CMAKE_CXX_COMPILER "/usr/bin/g++")

if (APPLE)
  if (CMAKE_SYSTEM_PROCESSOR MATCHES "arm64")
      message(STATUS "Building for arm64 architecture")
      set(CMAKE_OSX_ARCHITECTURES "arm64")
  elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
      message(STATUS "Building for x86_64 architecture")
      set(CMAKE_OSX_ARCHITECTURES "x86_64")
  endif ()
endif()

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} src)

add_executable(backlight_capture src/main.cpp)
target_link_libraries(backlight_capture ${OpenCV_LIBS})

if (USE_X11)
    target_compile_definitions(backlight_capture PRIVATE USE_X11)
    target_link_libraries(backlight_capture X11)
elseif (USE_WAYLAND)
    target_compile_definitions(backlight_capture PRIVATE USE_WAYLAND)

elseif (WIN32)
    target_compile_definitions(backlight_capture PRIVATE _WIN32)
    target_link_libraries(backlight_capture gdi32)

    # Install the executable
    install(TARGETS backlight_capture DESTINATION "C:/Program Files/BacklightCapture")

    # Create a batch script to run the program
    configure_file(${CMAKE_SOURCE_DIR}/src/backlight_capture.bat.in ${CMAKE_BINARY_DIR}/backlight_capture.bat @ONLY)
    install(FILES ${CMAKE_BINARY_DIR}/backlight_capture.bat DESTINATION "C:/Program Files/BacklightCapture")
elseif (APPLE)
    target_compile_definitions(backlight_capture PRIVATE __APPLE__)
    find_library(COREFOUNDATION_LIBRARY CoreFoundation)
    find_library(APPLICATIONSERVICES_LIBRARY ApplicationServices)
    target_link_libraries(backlight_capture ${COREFOUNDATION_LIBRARY} ${APPLICATIONSERVICES_LIBRARY})

    # Install the executable
    install(TARGETS backlight_capture DESTINATION "/Library/Application Support/BacklightCapture")

    # Create a Launch Agent plist
    set(PLIST_FILE "${CMAKE_BINARY_DIR}/com.backlight_capture.backlight_capture.plist")
    configure_file(${CMAKE_SOURCE_DIR}/src/com.backlight_capture.backlight_capture.plist.in ${PLIST_FILE} @ONLY)
    install(FILES ${PLIST_FILE} DESTINATION /Library/LaunchAgents)
endif()

if (UNIX AND NOT APPLE)
    # Define the path to the udev rule
    set(UDEV_RULE "${CMAKE_BINARY_DIR}/99-backlight.rules")

    # Create the udev rule content
    if(USE_WAYLAND)
      execute_process(COMMAND swaymsg -t get_outputs | jq -r '.[0].name' OUTPUT_VARIABLE OUTPUT_DEV OUTPUT_STRIP_TRAILING_WHITESPACE)
      file(WRITE ${UDEV_RULE} "ACTION==\"add\", KERNEL==\"ttyUSB[0-9]*\", RUN+=\"/bin/bash -c /usr/bin/backlight_capture /dev/%k -o ${OUTPUT_DEV}'\"\n")
    else()
      file(WRITE ${UDEV_RULE} "ACTION==\"add\", KERNEL==\"ttyUSB[0-9]*\", RUN+=\"/bin/bash -c /usr/bin/backlight_capture /dev/%k'\"\n")
    endif()

    # Install the backlight_capture executable
    install(TARGETS backlight_capture DESTINATION /usr/bin)

    # Install the udev rule
    #install(FILES ${UDEV_RULE} DESTINATION /etc/udev/rules.d/)
endif()