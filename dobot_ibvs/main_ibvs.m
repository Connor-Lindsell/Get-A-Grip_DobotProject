function main_ibvs()
% expects ROS to be up and robot in OPERATING
cfg = dobot_ibvs.config.camera_topics();

colorSub = rossubscriber(cfg.rgb_topic,   'sensor_msgs/Image');
depthSub = rossubscriber(cfg.depth_topic, 'sensor_msgs/Image');
infoSub  = rossubscriber(cfg.info_topic,  'sensor_msgs/CameraInfo');

K = reshape(receive(infoSub,2).K,[3 3]); fx=K(1,1); fy=K(2,2); u0=K(1,3); v0=K(2,3);
data = load('dobot_ibvs/config/handeye_T_base_cam.mat'); T_base_cam = data.T_base_cam;

[targetEEPub, targetEEMsg] = rospublisher('/dobot_magician/target_end_effector_pose','geometry_msgs/Pose');
eeSub  = rossubscriber('/dobot_magician/end_effector_poses','geometry_msgs/PoseStamped');
[toolPub, toolMsg] = rospublisher('/dobot_magician/target_tool_state','std_msgs/Int32MultiArray');

gains = struct('kx',0.003,'ky',0.003,'kz',0.0,'Zdes',0.10);
rate  = rateControl(15);
center_u = u0; center_v = v0;

fprintf('IBVS: starting centering loop...\n');
Pc = [NaN NaN NaN].'; % last 3D point buffer

for step = 1:200
    rgbMsg = receive(colorSub,1); if isempty(rgbMsg), waitfor(rate); continue; end
    dptMsg = receive(depthSub,1); if isempty(dptMsg), waitfor(rate); continue; end
    rgb = rosReadImage(rgbMsg);
    depth = rosReadImage(dptMsg);

    blocks = dobot_ibvs.perception.segmentBlocks(rgb);
    if isempty(blocks), waitfor(rate); continue; end
    blk = dobot_ibvs.perception.pickTarget(blocks);

    Pc = dobot_ibvs.perception.getBlockPose3D(blk.centroid, depth, fx, fy, u0, v0, cfg.depth_in_mm);
    Z  = Pc(3);

    eemsg = receive(eeSub,1);
    T_base_ee = trvec2tform([eemsg.Pose.Position.X, eemsg.Pose.Position.Y, eemsg.Pose.Position.Z]) * ...
                quat2tform([eemsg.Pose.Orientation.W, eemsg.Pose.Orientation.X, ...
                            eemsg.Pose.Orientation.Y, eemsg.Pose.Orientation.Z]);

    [p_next, R_next] = dobot_ibvs.control.ibvsStep(blk.centroid(1), blk.centroid(2), ...
                                center_u, center_v, Z, fx, fy, T_base_cam, T_base_ee, gains);
    eeQuat = rotm2quat(R_next);
    targetEEMsg.Position.X = p_next(1);
    targetEEMsg.Position.Y = p_next(2);
    targetEEMsg.Position.Z = p_next(3);
    targetEEMsg.Orientation.W = eeQuat(1);
    targetEEMsg.Orientation.X = eeQuat(2);
    targetEEMsg.Orientation.Y = eeQuat(3);
    targetEEMsg.Orientation.Z = eeQuat(4);
    send(targetEEPub, targetEEMsg);

    waitfor(rate);
end

fprintf('IBVS: centering done, executing pick and place...\n');
Pb = T_base_cam * [Pc;1]; targetXYZ_base = Pb(1:3).';
dropXYZ_base  = targetXYZ_base + [0.0, -0.15, 0.0];  % adjust to your bin
dobot_ibvs.motion.pickPlace(targetXYZ_base, dropXYZ_base, toolPub, toolMsg, targetEEPub, targetEEMsg);
fprintf('Sequence complete.\n');
end
