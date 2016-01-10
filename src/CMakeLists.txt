add_subdirectory(states)

rock_library(common 
        CommonStartup.cpp
        Eo2Robot.cpp
    DEPS_PKGCONFIG
        state_machine
        state_machine_viz
)

rock_executable(Drive
    SOURCES 
        Drive.cpp
    HEADERS
    DEPS        
        states
        common
    DEPS_PKGCONFIG 
        state_machine
        state_machine_viz
)