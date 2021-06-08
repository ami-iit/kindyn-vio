function publishJointStates(publisher, joint_list, joint_pos, joint_vel, joint_effort)

joint_msg = rosmessage('sensor_msgs/JointState');
joint_msg.Header.Stamp = rostime('now');
joint_msg.Name = joint_list;
joint_msg.Position = joint_pos;
joint_msg.Velocity = joint_vel;
joint_msg.Effort = joint_effort;
send(publisher, joint_msg);
end

