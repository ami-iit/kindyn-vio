function publishIMU(publisher, imu_name, acc, omega, rot)
imu_msg = rosmessage('sensor_msgs/Imu');
imu_msg.Header.Stamp = rostime('now');
imu_msg.Header.FrameId = imu_name;
imu_msg.LinearAcceleration.X = acc(1);
imu_msg.LinearAcceleration.Y = acc(2);
imu_msg.LinearAcceleration.Z = acc(3);

imu_msg.AngularVelocity.X = omega(1);
imu_msg.AngularVelocity.Y = omega(2);
imu_msg.AngularVelocity.Z = omega(3);

q = Core.rpy2quat(rot(1), rot(2), rot(3));
imu_msg.Orientation.W = q(1);
imu_msg.Orientation.X = q(2);
imu_msg.Orientation.Y = q(3);
imu_msg.Orientation.Z = q(4);

send(publisher, imu_msg);
end
