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

    configure_file(${CMAKE_SOURCE_DIR}/scripts/backlight_capture.bat.in ${CMAKE_BINARY_DIR}/backlight_capture.bat @ONLY)
    install(FILES ${CMAKE_BINARY_DIR}/backlight_capture.bat DESTINATION "C:/Program Files/BacklightCapture")
elseif (APPLE)
    target_compile_definitions(backlight_capture PRIVATE __APPLE__)
    find_library(COREFOUNDATION_LIBRARY CoreFoundation)
    find_library(APPLICATIONSERVICES_LIBRARY ApplicationServices)
    target_link_libraries(backlight_capture ${COREFOUNDATION_LIBRARY} ${APPLICATIONSERVICES_LIBRARY})

    install(TARGETS backlight_capture DESTINATION "/Library/Application Support/BacklightCapture")

    set(PLIST_FILE "${CMAKE_BINARY_DIR}/com.backlight_capture.backlight_capture.plist")
    set(BASH_FILE "${CMAKE_BINARY_DIR}/backlight_capture.sh")

    set_target_properties(
      backlight_capture PROPERTIES
      MACOSX_BUNDLE TRUE
      MACOSX_BUNDLE_INFO_PLIST ${CMAKE_SOURCE_DIR}/scripts/info.plist
      )

    configure_file(${CMAKE_SOURCE_DIR}/scripts/com.backlight_capture.backlight_capture.plist.in ${PLIST_FILE} @ONLY)
    configure_file(${CMAKE_SOURCE_DIR}/scripts/backlight_capture.sh.in ${BASH_FILE} @ONLY)

    install(FILES ${PLIST_FILE} DESTINATION /Library/LaunchDaemons)
    install(
      FILES ${BASH_FILE}
      PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
      DESTINATION /Library/Application\ Support/BacklightCapture
      )
endif()

if (UNIX AND NOT APPLE)
    set(UDEV_RULE "${CMAKE_BINARY_DIR}/99-backlight.rules")
    set(SYSTEMD_SERVICE_FILE "${CMAKE_BINARY_DIR}/backlight_capture@.service")

    configure_file(${CMAKE_SOURCE_DIR}/scripts/99-backlight.rules.in ${UDEV_RULE} @ONLY)
    configure_file(${CMAKE_SOURCE_DIR}/scripts/backlight_capture@.service.in ${SYSTEMD_SERVICE_FILE} @ONLY)

    install(TARGETS backlight_capture DESTINATION /usr/bin)
    install(FILES ${UDEV_RULE} DESTINATION /etc/udev/rules.d/)
    install(FILES ${SYSTEMD_SERVICE_FILE} DESTINATION /etc/systemd/system/)
endif()
