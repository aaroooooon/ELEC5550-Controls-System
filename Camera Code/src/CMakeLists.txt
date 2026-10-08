FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

# This component is registered as "src" rather than "main", so it does not
# inherit the implicit dependency on every component in the build. Anything
# main.cpp includes has to be listed here.
#
# esp32-camera is not listed: it arrives as a managed component via
# src/idf_component.yml, which makes it a dependency of this component already.
# esp_jpeg comes with it as a public dependency of esp32-camera.
#
# weed_detection_model is the Edge Impulse library, wrapped as a component in
# components/weed_detection_model.
idf_component_register(
    SRCS ${app_sources}
    REQUIRES esp_driver_usb_serial_jtag esp_timer driver log weed_detection_model
)
