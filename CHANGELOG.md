# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Fixed
- Fixed implementation of `PointsTracker` class (https://github.com/dic-iit/kindyn-vio/pull/23).

### Added
- Implement `PerceptionCameraModels` library containing `PinHoleCamera` class.
- Implement `PerceptionFeatures` library containing `PointsTracker` class and `ImageProcessor` class.
- Add `ArucoDetectorExample` and `PointsTrackerTest` (https://github.com/dic-iit/kindyn-vio/pull/21).
- Implement `LinesTracker` class in `PerceptionFeatures` library and add `LinesTrackerTest`. (https://github.com/dic-iit/kindyn-vio/pull/23).
- Improve `ImageProcessor` class to consider `LinesTracker` class (https://github.com/dic-iit/kindyn-vio/pull/23).
