function publishWrench(publisher, wrench, frame_id)

wrench_msg = rosmessage('geometry_msgs/WrenchStamped');
wrench_msg.Header.Stamp = rostime('now');

wrench_msg.Header.FrameId = frame_id;

wrench_msg.Wrench.Force.X = wrench(1);
wrench_msg.Wrench.Force.Y = wrench(2);
wrench_msg.Wrench.Force.Z = wrench(3);

wrench_msg.Wrench.Torque.X = wrench(4);
wrench_msg.Wrench.Torque.Y = wrench(5);
wrench_msg.Wrench.Torque.Z = wrench(6);

send(publisher, wrench_msg);

end

