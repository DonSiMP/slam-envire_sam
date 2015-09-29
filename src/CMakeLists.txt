rock_library(sam
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(sam_bin Main.cpp
    DEPS sam)

