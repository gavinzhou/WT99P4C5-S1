idf_component_register(
    SRCS main.cpp c5_flasher.c shutter.c
    INCLUDE_DIRS .
    # 2026-04-24 ADR-023 M3.1 note:
    # Adding PRIV_REQUIRES triggers ESP-IDF strict mode — auto-inherited
    # "common" components are no longer implicit. Must list every component
    # whose headers are #include'd from main.cpp / c5_flasher.c / shutter.c.
    # (Previously `main` was leeching the default common-component set by
    # having no explicit REQUIRES at all.)
    PRIV_REQUIRES
        # Build system + core
        esp_common esp_system esp_timer log
        # FreeRTOS (tasks, event groups)
        freertos
        # Device / system services
        nvs_flash esp_netif esp_event esp_wifi
        lwip driver esp_driver_gpio
        # PSRAM heap allocation (heap_caps_aligned_alloc)
        heap
        # M3.1 algorithm acceleration
        esp-dsp
        # Custom components / managed components
        espressif__esp_hosted
        espressif__esp-serial-flasher
    EMBED_FILES c5_fw.bin)

idf_component_get_property(LVGL_LIB lvgl__lvgl COMPONENT_LIB)
target_compile_options(
    ${LVGL_LIB} 
    PRIVATE 
        -DLV_LVGL_H_INCLUDE_SIMPLE
        -DLV_USE_DEMO_MUSIC
)

if(CONFIG_ESP_BROOKESIA_MEMORY_USE_CUSTOM)
    idf_build_get_property(build_components BUILD_COMPONENTS)
    foreach(COMPONENT ${build_components})
        if(COMPONENT MATCHES "esp-brookesia" OR COMPONENT MATCHES "espressif__esp-brookesia")
            set(TARGET_COMPONENT ${COMPONENT})
            break()
        endif()
    endforeach()

    if(TARGET_COMPONENT STREQUAL "")
        message(FATAL_ERROR "Component 'esp-brookesia' not found.")
    else()
        idf_component_get_property(ESP_BROOKESIA_LIB ${TARGET_COMPONENT} COMPONENT_LIB)
    endif()

    target_compile_options(
        ${ESP_BROOKESIA_LIB}
        PUBLIC
            "-DESP_BROOKESIA_MEMORY_CUSTOM_MALLOC(x)=heap_caps_aligned_alloc(1, x, MALLOC_CAP_SPIRAM)"  "-DESP_BROOKESIA_MEMORY_CUSTOM_FREE=free"
    )
endif()

if(CONFIG_LV_MEM_CUSTOM)
    target_compile_options(
        ${LVGL_LIB} 
        PRIVATE 
            "-DLV_MEM_CUSTOM_ALLOC(x)=heap_caps_malloc(x, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)"
    )
endif()

spiffs_create_partition_image(storage ../spiffs FLASH_IN_PROJECT)