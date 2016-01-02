rock_library(taskmanagement
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(taskmanagement_bin Main.cpp
    DEPS taskmanagement)

