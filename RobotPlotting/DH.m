% Produces 3x3 Homogeneous Transformation Matrix
function T = DH(theta, link_length)

T = [   cos(theta), -sin(theta), link_length*cos(theta);
        sin(theta),  cos(theta), link_length*sin(theta);
                  0,          0,                       1];
