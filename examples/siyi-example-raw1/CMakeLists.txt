project(siyi-example-raw1)

SET(SRCS
        siyi-example-raw1.cpp
        )

add_executable(siyi-example-raw1 ${SRCS})
target_link_libraries(siyi-example-raw1 ${SIYI_LIBS})
