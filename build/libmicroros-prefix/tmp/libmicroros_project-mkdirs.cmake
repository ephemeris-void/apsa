# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/home/spider/complete_clean_zyphr_micro_ros_lora_sensor_backup_final_clean/extracted/zephyr_project/micro_ros_zephyr_module/modules/libmicroros")
  file(MAKE_DIRECTORY "/home/spider/complete_clean_zyphr_micro_ros_lora_sensor_backup_final_clean/extracted/zephyr_project/micro_ros_zephyr_module/modules/libmicroros")
endif()
file(MAKE_DIRECTORY
  "/home/spider/complete_clean_zyphr_micro_ros_lora_sensor_backup_final_clean/extracted/zephyr_project/micro_ros_zephyr_module/modules/libmicroros"
  "/home/spider/complete_clean_zyphr_micro_ros_lora_sensor_backup_final_clean/extracted/zephyr_project/build/libmicroros-prefix"
  "/home/spider/complete_clean_zyphr_micro_ros_lora_sensor_backup_final_clean/extracted/zephyr_project/build/libmicroros-prefix/tmp"
  "/home/spider/complete_clean_zyphr_micro_ros_lora_sensor_backup_final_clean/extracted/zephyr_project/build/libmicroros-prefix/src/libmicroros_project-stamp"
  "/home/spider/complete_clean_zyphr_micro_ros_lora_sensor_backup_final_clean/extracted/zephyr_project/build/libmicroros-prefix/src"
  "/home/spider/complete_clean_zyphr_micro_ros_lora_sensor_backup_final_clean/extracted/zephyr_project/build/libmicroros-prefix/src/libmicroros_project-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/spider/complete_clean_zyphr_micro_ros_lora_sensor_backup_final_clean/extracted/zephyr_project/build/libmicroros-prefix/src/libmicroros_project-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/spider/complete_clean_zyphr_micro_ros_lora_sensor_backup_final_clean/extracted/zephyr_project/build/libmicroros-prefix/src/libmicroros_project-stamp${cfgdir}") # cfgdir has leading slash
endif()
