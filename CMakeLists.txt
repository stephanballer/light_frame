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

    install(TARGETS backlight_capture DESTINATION "C:/Program Files/BacklightCapture")

    configure_file(${CMAKE_SOURCE_DIR}/src/backlight_capture.bat.in ${CMAKE_BINARY_DIR}/backlight_capture.bat @ONLY)
    install(FILES ${CMAKE_BINARY_DIR}/backlight_capture.bat DESTINATION "C:/Program Files/BacklightCapture")
elseif (APPLE)
    target_compile_definitions(backlight_capture PRIVATE __APPLE__)
    find_library(COREFOUNDATION_LIBRARY CoreFoundation)
    find_library(APPLICATIONSERVICES_LIBRARY ApplicationServices)
    target_link_libraries(backlight_capture ${COREFOUNDATION_LIBRARY} ${APPLICATIONSERVICES_LIBRARY})

    install(TARGETS backlight_capture DESTINATION "/Library/Application Support/BacklightCapture")

    set(PLIST_FILE "${CMAKE_BINARY_DIR}/com.backlight_capture.backlight_capture.plist")

    set_target_properties(backlight_capture PROPERTIES
    MACOSX_BUNDLE TRUE
    MACOSX_BUNDLE_INFO_PLIST ${CMAKE_SOURCE_DIR}/src/info.plist
)
    configure_file(${CMAKE_SOURCE_DIR}/src/com.backlight_capture.backlight_capture.plist.in ${PLIST_FILE} @ONLY)
    install(FILES ${PLIST_FILE} DESTINATION /Library/LaunchDaemons)
endif()

if (UNIX AND NOT APPLE)
    set(UDEV_RULE "${CMAKE_BINARY_DIR}/99-backlight.rules")
    file(WRITE ${UDEV_RULE} "ACTION==\"add\", KERNEL==\"ttyUSB[0-9]*\", TAG+=\"systemd\", ENV{SYSTEMD_WANTS}=\"backlight_capture@%k.service\"\n")
    
    set(SYSTEMD_SERVICE_FILE "${CMAKE_BINARY_DIR}/backlight_capture@.service")
    file(WRITE ${SYSTEMD_SERVICE_FILE} "[Unit]\n")
    file(APPEND ${SYSTEMD_SERVICE_FILE} "Description=Backlight Capture Service\n")
    file(APPEND ${SYSTEMD_SERVICE_FILE} "After=network.target\n\n")
    file(APPEND ${SYSTEMD_SERVICE_FILE} "[Service]\n")
    file(APPEND ${SYSTEMD_SERVICE_FILE} "ExecStart=/usr/bin/backlight_capture /dev/%I\n")
    file(APPEND ${SYSTEMD_SERVICE_FILE} "StandardOutput=append:/tmp/backlight_capture.log\n")
    file(APPEND ${SYSTEMD_SERVICE_FILE} "StandardError=append:/tmp/backlight_capture.log\n")
    file(APPEND ${SYSTEMD_SERVICE_FILE} "Restart=always\n\n")
    file(APPEND ${SYSTEMD_SERVICE_FILE} "[Install]\n")
    file(APPEND ${SYSTEMD_SERVICE_FILE} "WantedBy=multi-user.target\n")

    install(TARGETS backlight_capture DESTINATION /usr/bin)
    install(FILES ${UDEV_RULE} DESTINATION /etc/udev/rules.d/)
    install(FILES ${SYSTEMD_SERVICE_FILE} DESTINATION /etc/systemd/system/)
endif()
