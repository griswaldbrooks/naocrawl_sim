function plotFrame3(H, scale, width)

d = [0,0,0,1]';
x = [scale,0,0,1]';
y = [0,scale,0,1]';
z = [0,0,scale,1]';

p = H*d;
frame_x = H*x;
frame_y = H*y;
frame_z = H*z;

line([p(1),frame_x(1)],[p(2),frame_x(2)],[p(3),frame_x(3)],'Color','m','LineWidth',width);
line([p(1),frame_y(1)],[p(2),frame_y(2)],[p(3),frame_y(3)],'Color','g','LineWidth',width);
line([p(1),frame_z(1)],[p(2),frame_z(2)],[p(3),frame_z(3)],'Color','c','LineWidth',width);
