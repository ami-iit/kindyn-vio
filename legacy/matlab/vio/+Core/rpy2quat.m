function quat = rpy2quat(roll, pitch, yaw)
rot = iDynTree.Rotation.RPY(roll, pitch, yaw);
quat = rot.asQuaternion().toMatlab();
end

