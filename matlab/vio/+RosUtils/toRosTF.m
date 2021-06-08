function tform = toRosTF(tftree, relative_pose, parent, child)
tform = rosmessage('geometry_msgs/TransformStamped');
tform.Header.FrameId = parent;
tform.ChildFrameId = child;

[R, p] = LieGroups.SE3.extractSE3(relative_pose);
quat = Core.rot2quat(R);

tform.Transform.Translation.X = p(1);
tform.Transform.Translation.Y = p(2);
tform.Transform.Translation.Z = p(3);

tform.Transform.Rotation.W = quat(1);
tform.Transform.Rotation.X = quat(2);
tform.Transform.Rotation.Y = quat(3);
tform.Transform.Rotation.Z = quat(4);

tform.Header.Stamp = rostime('now');
sendTransform(tftree, tform);
end

