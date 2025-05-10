set(TARGET bencher-defaults)

add_library(${TARGET} INTERFACE)

target_compile_features(${TARGET}
    INTERFACE
        cxx_std_20
)

target_compile_options(${TARGET}
    INTERFACE
        -fdiagnostics-color=always
 
        -Wno-interference-size

        -Wall
        -Wextra
        -Wpedantic
        -Wshadow
        -Wconversion
        -Wsign-conversion
        -Wunused
        -Wformat=2
        -Wnon-virtual-dtor

        # -fstack-protector-strong # inserts checks for buffer overflow
        # -fPIE -pie
        
        $<$<CONFIG:Debug>:
            -Og
            -g3
            -ggdb
            -fsanitize=address,leak,undefined
            -fno-omit-frame-pointer

            -march=native
        >

        $<$<CONFIG:Release>:
            -g
            -O2
            -march=native
            -flto
            -DNDEBUG
        >
)

target_link_options(${TARGET}
    INTERFACE
        -fdiagnostics-color=always

        -Wall
        -Wextra
        # -fPIE -pie
        # -fuse-ld=mold
        -rdynamic

        $<$<CONFIG:Debug>:
            -Og
            -g3
            -ggdb
            -fsanitize=address,leak,undefined
            -fno-omit-frame-pointer
        >

        $<$<CONFIG:Release>:
            -g
            -O2
            -march=native
            -flto
        >
)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # to generate compile_commands.json
