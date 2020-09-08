file(GLOB FOUR_D_ASSOCIATION_HEADER_FILES
    LIST_DIRECTORIES false
    associater.h
    camera.h
    color_util.h
    hungarian_algorithm.h
    kruskal_associater.h
    math_util.h
    openpose.h
    skel.h
    skel_driver.h
    skel_painter.h
    skel_solver.h
    skel_updater.h
    )
file(GLOB FOUR_D_ASSOCIATION_SOURCE_FILES
    LIST_DIRECTORIES false
    associater.cpp
    camera.cpp
    hungarian_algorithm.cpp
    kruskal_associater.cpp
    openpose.cpp
    skel_driver.cpp
    skel_painter.cpp
    skel_solver.cpp
    skel_updater.cpp
    )

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/../include/
    ${JSONCPP_INCLUDE_DIRS}
    )

link_directories(
    ${PROJECT_SOURCE_DIR}/../library/
    )

add_library(four_d_association STATIC
    ${FOUR_D_ASSOCIATION_HEADER_FILES}
    ${FOUR_D_ASSOCIATION_SOURCE_FILES}
    )

add_executable(mocap
    main.cpp
    )

add_executable(evaluate_shelf
    evaluate_shelf_main.cpp
    )

if   (WIN32)
    target_link_libraries(mocap
        four_d_association
        ${OpenCV_LIBS}
        debug jsoncppd.lib
        optimized jsoncpp.lib
        )
    target_link_libraries(evaluate_shelf
        four_d_association
        ${OpenCV_LIBS}
        debug jsoncppd.lib
        optimized jsoncpp.lib
        )
else (WIN32)
    target_link_libraries(four_d_association
    Eigen3::Eigen
        )
    target_link_libraries(mocap
        four_d_association
        ${OpenCV_LIBS}
        ${JSONCPP_LIBRARIES}
        )
    target_link_libraries(evaluate_shelf
        four_d_association
        ${OpenCV_LIBS}
        ${JSONCPP_LIBRARIES}
        )
endif(WIN32)
