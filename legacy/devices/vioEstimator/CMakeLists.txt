set(VIO_SRC src/VIOEstimatorDevice.cpp)
set(VIO_HDR include/VIOEstimatorDevice.h)

yarp_prepare_plugin(VIOEstimatorDevice CATEGORY device
                                       TYPE wbe::dev::VIOEstimatorDevice
                                       INCLUDE ${VIO_HDR}
                                       DEFAULT ON)
yarp_add_plugin(VIOEstimatorDevice ${VIO_SRC} ${VIO_HDR})
target_include_directories(VIOEstimatorDevice PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(VIOEstimatorDevice PUBLIC YARP::YARP_eigen
                                                YARP::YARP_os
                                                YARP::YARP_sig
                                                YARP::YARP_dev
                                                YARP::YARP_cv
                                                YARPRobotsHelper::yarp-robots-helper
                                                ${iDynTree_LIBRARIES}
                                         PRIVATE Eigen3::Eigen)

yarp_install(TARGETS VIOEstimatorDevice
             COMPONENT runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
             YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

add_subdirectory(app)

