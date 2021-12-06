# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)


set(EXTRA_COMPONENT_DIRS    ./components)

add_compile_options(-fdiagnostics-color=always)

#project(OLM2_PRO_174)

#project(ESP_OLM2_PRO_186)
#project(ESP_OLM2_PRO_S2_186)
#project(ESP_AUFERO2_186)
project(ESP_AUFERO1_186)
#project(ESP_OLM_2_186)
#project(ESP_OLM2_S2_186)
