project(siyi-example-get-info)

SET(SRCS
        siyi-example-get-info.cpp
        )

add_executable(siyi-example-get-info ${SRCS})
target_link_libraries(siyi-example-get-info ${SIYI_LIBS})
