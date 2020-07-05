set(ESTIMATORS_SRC src/VIOState.cpp
                   src/VIOEstimator.cpp)

set(ESTIMATORS_HDR include/Estimators/VIOState.h
                   include/Estimators/VIOObservation.h
                   include/Estimators/VIOEstimator.h)

set(libraryname vio-estimators)

add_library(${libraryname} ${ESTIMATORS_SRC} ${ESTIMATORS_HDR})
add_library(Viper::${libraryname} ALIAS ${libraryname})

target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                         "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}>")
target_link_libraries(${libraryname} PUBLIC  ${OpenCV_LIBS}
                                             ${iDynTree_LIBRARIES}
                                     PRIVATE Eigen3::Eigen)

# Specify installation targets, typology and destination folders.
install(TARGETS ${libraryname}
        EXPORT  ${PROJECT_NAME}
        COMPONENT runtime
        LIBRARY       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                            COMPONENT shlib
        ARCHIVE       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                            COMPONENT lib
        RUNTIME       DESTINATION "${CMAKE_INSTALL_BINDIR}"                            COMPONENT bin
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/Viper/Estimators" COMPONENT dev)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${libraryname})

message(STATUS "Created target ${libraryname} for export ${PROJECT_NAME}.")

if(BUILD_TESTING)
    add_subdirectory(test)
endif()

