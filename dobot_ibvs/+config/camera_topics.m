function cfg = camera_topics()
% central place to change topics and depth units
cfg.rgb_topic   = '/camera/color/image_raw';
cfg.depth_topic = '/camera/aligned_depth_to_color/image_raw';
cfg.info_topic  = '/camera/color/camera_info';
cfg.depth_in_mm = true;  % set false if your depth is already in metres
end
