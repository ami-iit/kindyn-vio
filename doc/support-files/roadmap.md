# KinDynVIO

KinDynVIO is a factor graph-based, tightly-coupled multimodal sensor fusion framework that aims at fusing inertial, kinematic, visual and centroidal factors of a humanoid robot in order to achieve a reliable whole body estimation.



### Roadmap

- [x] Add points tracker class for image features
- [x] Add lines tracker class for image features
- [x] Add features manager class for tracking the features across images
- [ ] Add an example of BLF filter with GTSAM iSAM2 based fixed lag smoother with  odometry factors
- [x] Add IMU preintegration class
- [ ] BLF filter as odometry factor + IMU preintegration for kinematic-inertial smoothing
- [ ] Add an example of Visual-inertial smoothing based on image features
- [ ] Test on a realsense recorded dataset with Aruco marker tracking as ground truth
- [ ] Connect Kinematic-inertial smoother with visual-inertial smoother
- [ ] Add centroidal dynamics factor
- [ ] Add centroidal kinematics factor
- [ ] Add legged odometry based velocity-preintegration factor
- [ ] Test  kinematic-inertial-centroidal smoother with iCub 2.5 dataset
- [ ] Add Gazebo simulated experiment of kinematic-inertial-visual-centroidal smoother with iCub3 robot walking experiment
- [ ] ...