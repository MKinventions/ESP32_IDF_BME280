# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/MK_ESP-IDF/esp-idf-v4.4.2/components/bootloader/subproject"
  "C:/Users/thoshiba/eclipse/BME280_2/build/bootloader"
  "C:/Users/thoshiba/eclipse/BME280_2/build/bootloader-prefix"
  "C:/Users/thoshiba/eclipse/BME280_2/build/bootloader-prefix/tmp"
  "C:/Users/thoshiba/eclipse/BME280_2/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Users/thoshiba/eclipse/BME280_2/build/bootloader-prefix/src"
  "C:/Users/thoshiba/eclipse/BME280_2/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/thoshiba/eclipse/BME280_2/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
