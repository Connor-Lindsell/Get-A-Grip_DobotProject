function L = Lx(x,y,Z)
% L relates camera spatial velocity [vx vy vz wx wy wz] to feature rates [x_dot; y_dot]
L = [ -1/Z,   0,   x/Z,   x*y,     -(1 + x^2),   y;
        0,  -1/Z,  y/Z,   1 + y^2,   -x*y,     -x ];
end