
set(HEADERS         A_star.h
                    config.h
                    environmentoptions.h
                    gl_const.h
                    ilogger.h
                    map.h
                    mission.h
                    node.h
                    node_comparator.h
                    safeinterval.h
                    safeintervals.h
                    search.h
                    searchresult.h
                    sipp.h
                    tinyxml2.h
                    xmllogger.h)

set(SOURCES         A_star.cpp
                    config.cpp
                    environmentoptions.cpp
                    map.cpp
                    mission.cpp
                    node_comparator.cpp
                    safeintervals.cpp
                    search.cpp
                    sipp.cpp
                    tinyxml2.cpp
                    xmllogger.cpp)

add_executable(${PROJECT_NAME} ${HEADERS} ${SOURCES} main.cpp)

add_library(PPLIB ${HEADERS} ${SOURCES})

install(TARGETS ${PROJECT_NAME} 
        CONFIGURATIONS Debug
        DESTINATION Bin/Debug)

install(TARGETS ${PROJECT_NAME}
        CONFIGURATIONS Release
        DESTINATION Bin/Release)
