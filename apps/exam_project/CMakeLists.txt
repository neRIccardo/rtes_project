# Nome dell'eseguibile del progetto
add_executable(exam_project
    main.c
)

# Linka le librerie necessarie: Pico SDK + FreeRTOS
target_link_libraries(exam_project
    pico_stdlib
    pico_async_context_freertos
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
    hardware_adc
)

# Abilita l'uso della USB per la seriale e disabilita la UART hardware
pico_enable_stdio_usb(exam_project 1)
pico_enable_stdio_uart(exam_project 0)

# Imposta la directory di output per il file UF2
set_target_properties(exam_project PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

# Genera anche i file UF2 e binari per il flashing
pico_add_extra_outputs(exam_project)
