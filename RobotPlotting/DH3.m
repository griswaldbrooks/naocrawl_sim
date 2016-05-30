% Produces 4x4 Homogeneous Transformation Matrix
function T = DH3(d, theta, a, alpha)

T_znm1 = [  1,0,0,0;
            0,1,0,0;
            0,0,1,d;
            0,0,0,1];
R_znm1 = [  cos(theta), -sin(theta), 0, 0;
            sin(theta),  cos(theta), 0, 0;
                     0,           0, 1, 0;
                     0,           0, 0, 1];
T_xn = [  1,0,0,a;
          0,1,0,0;
          0,0,1,0;
          0,0,0,1];
R_xn = [    1,          0,           0, 0;
            0, cos(alpha), -sin(alpha), 0;
            0, sin(alpha),  cos(alpha), 0;
            0,          0,           0, 1];

T = T_znm1*R_znm1*T_xn*R_xn;