function plotFrame(H, scale, width)

d = [0,0,1]';
x = [scale,0,1]';
y = [0,scale,1]';

p = H*d;
frame_x = H*x;
frame_y = H*y;

line([p(1),frame_x(1)],[p(2),frame_x(2)],'Color','r','LineWidth',width);
%line([p(1),frame_y(1)],[p(2),frame_y(2)],'Color','g','LineWidth',width);
