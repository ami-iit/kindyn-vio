# KinDynVIO: A Multimodal Sensor Fusion Framework for Whole Body Estimation of a Humanoid Robot ![C++ CI Workflow](https://github.com/ami-iit/kindyn-vio/actions/workflows/conda-ci.yml/badge.svg)


<p align="center">
  <b>:warning: REPOSITORY UNDER DEVELOPMENT :warning:</b>
  <br>This project is a Work in Progress and is still in its preliminary development stage. It does not guarantee (yet) any support in terms of usage and documentation.  The libraries implemented in this repository are still experimental and we cannot guarantee stable API.
</p>


 Basic information can be found below. More details **to be updated soon**.



##  :hammer: Dependencies

- [iDynTree](https://github.com/robotology/idyntree)
- [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) (**3.3.9** REQUIRED)
- [manif](https://github.com/artivis/manif)
- [BipedalLocomotionFramework](https://github.com/ami-iit/bipedal-locomotion-framework)
- [spdlog](https://github.com/gabime/spdlog)
- [matioCpp](https://github.com/ami-iit/matio-cpp)
- [OpenCV](https://github.com/opencv/opencv)
- [GTSAM](https://github.com/borglab/gtsam)
- [Catch2](https://github.com/catchorg/Catch2) Optional for tests

**NOTE:** This project uses C++17 standard as a default.  In order to avoid unexpected segmentation faults related to Eigen, we recommend to compile `GTSAM` with the CMake flags, `-DGTSAM_COMPILE_FEATURES_PUBLIC=cxx_std_17`, `-DGTSAM_USE_SYSTEM_EIGEN=ON`,  `-DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF`.



## Build the software

```sh
git clone https://github.com/ami-iit/kindyn-vio.git
cd kindyn-vio
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=<path/where/you/want/to/install> \
      -DCMAKE_BUILD_TYPE=Release \
cmake --build . --config Release --target install
```


### Acknowledgements

- CMake structure of the repository is derived from the [`BipedalLocomotionFramework`](https://github.com/ami-iit/bipedal-locomotion-framework) project.

- Perception related libraries are derived from open-source projects [`VINS-MONO`](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) and [`PL-VINS`](https://github.com/cnqiangfu/PL-VINS). It must be noted that these repositories are distributed under GPL v3, and since the classes `PointsTracker` and `LinesTracker` which are modified versions of the original source code, these classes are distributed under GPL v3 license as well, while the rest of our repository is distributed under LGPL license v3.0 or more.

- Overall software architecture is inspired from the [VILENS](https://ori.ox.ac.uk/labs/drs/vilens-tightly-fused-multi-sensor-odometry/) project from the DRS group in Oxford Research Institute.



### Roadmap

Please check  [doc/support-files/roadmap.md](./doc/support-files/roadmap.md) to get an overview of the roadmap of this project.

