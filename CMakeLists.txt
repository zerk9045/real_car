cmake_minimum_required(VERSION 3.16)
project(real_car LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  real_car
  SHARED
  hardware/real_car.cpp
)
target_compile_features(real_car PUBLIC cxx_std_17)
target_include_directories(real_car PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/real_car>
)
ament_target_dependencies(
  real_car PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "REAL_CAR_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface real_car.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/real_car
)
install(
  DIRECTORY description
  DESTINATION share/real_car
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/real_car
)
install(TARGETS real_car
  EXPORT export_real_car
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_real_car HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
