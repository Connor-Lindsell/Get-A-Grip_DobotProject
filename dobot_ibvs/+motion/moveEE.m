function moveEE(pub,msg,xyz,quat)
if nargin<4, quat = [1 0 0 0]; end
msg.Position.X = xyz(1); msg.Position.Y = xyz(2); msg.Position.Z = xyz(3);
msg.Orientation.W = quat(1); msg.Orientation.X = quat(2);
msg.Orientation.Y = quat(3); msg.Orientation.Z = quat(4);
send(pub,msg);
pause(0.2);
end
