function pickPlace(targetXYZ_base, dropXYZ_base, toolPub, toolMsg, targetEEPub, targetEEMsg)
hoverUp = 0.05; quat = [1 0 0 0]; % keep yaw as-is
dobot_ibvs.motion.moveEE(targetEEPub, targetEEMsg, targetXYZ_base+[0 0 hoverUp], quat);
dobot_ibvs.motion.moveEE(targetEEPub, targetEEMsg, targetXYZ_base, quat); pause(0.3);
toolMsg.Data = int32(1); send(toolPub, toolMsg); pause(0.3);
dobot_ibvs.motion.moveEE(targetEEPub, targetEEMsg, targetXYZ_base+[0 0 hoverUp], quat);
dobot_ibvs.motion.moveEE(targetEEPub, targetEEMsg, dropXYZ_base+[0 0 hoverUp], quat);
dobot_ibvs.motion.moveEE(targetEEPub, targetEEMsg, dropXYZ_base, quat); pause(0.3);
toolMsg.Data = int32(0); send(toolPub, toolMsg); pause(0.3);
dobot_ibvs.motion.moveEE(targetEEPub, targetEEMsg, dropXYZ_base+[0 0 hoverUp], quat);
end
