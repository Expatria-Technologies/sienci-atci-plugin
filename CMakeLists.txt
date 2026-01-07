add_library(sienci_atci INTERFACE)

target_sources(sienci_atci INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/sienci-atci-plugin.c
)

target_include_directories(sienci_atci INTERFACE ${CMAKE_CURRENT_LIST_DIR})
