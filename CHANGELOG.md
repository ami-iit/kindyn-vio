# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Fixed
- Fixed implementation of `PointsTracker` class (https://github.com/dic-iit/kindyn-vio/pull/23).

### Changed
- Modified `ImageProcessor` class into a `BipedalLocomotion::System::Advanceable` type. (https://github.com/dic-iit/kindyn-vio/pull/29)
- Modified `IMUPreintegrator` class to account only for preintegrated measurements by refactoring the factor out of the class. (https://github.com/dic-iit/kindyn-vio/pull/30)
- Move `KinDynVIO::Perception::TimeStampedImg` from `KinDynVIO/Perception/Features/ImageProcessor.h` to `KinDynVIO/Perception/Features/DataTypes.h`. (https://github.com/dic-iit/kindyn-vio/pull/30)
- Move Estimator relevant input-output data structures to a `IO.h` file in `Estimators` library.

### Added
- Implement `PerceptionCameraModels` library containing `PinHoleCamera` class.
- Implement `PerceptionFeatures` library containing `PointsTracker` class and `ImageProcessor` class.
- Add `ArucoDetectorExample` and `PointsTrackerTest` (https://github.com/dic-iit/kindyn-vio/pull/21).
- Implement `LinesTracker` class in `PerceptionFeatures` library and add `LinesTrackerTest`. (https://github.com/dic-iit/kindyn-vio/pull/23).
- Improve `ImageProcessor` class to consider `LinesTracker` class (https://github.com/dic-iit/kindyn-vio/pull/23).
- Add `IMUPreintegrator` class into `Estimators` library. This class wraps GTSAM `CombinedIMUFactor` preintegration into a `BipedalLocomotion::System::Advanceable` type. (https://github.com/dic-iit/kindyn-vio/pull/29)
- Add `ArucoWrapper` class into `Estimators` library to wrap `BipedalLocomotion::Perception::ArucoDetector` with `gtsam` relevant outputs. (https://github.com/dic-iit/kindyn-vio/pull/30)
- Add `TestData` library to maintain common datasets for test-cases. (https://github.com/dic-iit/kindyn-vio/pull/30)
- Add `Factors` library containing `NoiseModelFactor10`, `CentroidalDynamicsFactor`, `PreintegratedCDMCumulativeBias` and `CDMBiasCumulative` classes. (https://github.com/ami-iit/kindyn-vio/pull/32).
- Add `CentroidalDynamicsPreintegrator` class to `Estimators` library wrapping the `CentroidalDynamicsFactor` preintegration into a `BipedalLocomotion::System::Advanceable` type. (https://github.com/ami-iit/kindyn-vio/pull/32).
- Add iCub walking experiment dataset to `TestData`. (https://github.com/ami-iit/kindyn-vio/pull/33).
- Add `ArucoPointsLinesTracker` class in `PerceptionFeatures` to track the point and line features on detected Aruco markers.
- Add `FeaturesManager` class in `PerceptionFeatures` to maintain and manage feature tracks.
- Add `VisionFrontEnd` class in `PerceptionPipelines` to consolidate `ImageProcessor` and `FeaturesManager` into a vision front end pipeline.
- Add `GraphManager` class in `Estimators` to manage KinDynVIO related measurement factors and to run estimation using ISAM2 based Incremental Fixed Lag Smoother.
