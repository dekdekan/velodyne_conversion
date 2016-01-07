syms r p y x y z x1 y1 z1

ROLL=[1 0 0 0; 0 cos(r) -sin(r) 0; 0 sin(r) cos(r) 0; 0 0 0 1]
PITCH=[cos(p) 0 sin(p) 0; 0 1 0 0; -sin(p) 0 cos(p) 0; 0 0 0 1]
YAW=[cos(y) -sin(y) 0 0;sin(y) cos(y) 0 0; 0 0 1 0;0 0 0 1 ]

TRANS=[1 0 0 x;0 1 0 y; 0 0 1 z; 0 0 0 1]
T=YAW*PITCH*ROLL*TRANS*[x1;y1;z1;1]
% [ cos(p)*cos(y), cos(y)*sin(p)*sin(r) - cos(r)*sin(y), sin(r)*sin(y) + cos(r)*cos(y)*sin(p), z*(sin(r)*sin(y) + cos(r)*cos(y)*sin(p)) - y*(cos(r)*sin(y) - cos(y)*sin(p)*sin(r)) + x*cos(p)*cos(y)]
% [ cos(p)*sin(y), cos(r)*cos(y) + sin(p)*sin(r)*sin(y), cos(r)*sin(p)*sin(y) - cos(y)*sin(r), y*(cos(r)*cos(y) + sin(p)*sin(r)*sin(y)) - z*(cos(y)*sin(r) - cos(r)*sin(p)*sin(y)) + x*cos(p)*sin(y)]
% [       -sin(p),                        cos(p)*sin(r),                        cos(p)*cos(r),                                                          z*cos(p)*cos(r) - x*sin(p) + y*cos(p)*sin(r)]
% [             0,                                    0,                                    0,                                                                                                     1]


%  z*(sin(r)*sin(y) + cos(r)*cos(y)*sin(p)) - y1*(cos(r)*sin(y) - cos(y)*sin(p)*sin(r)) - y*(cos(r)*sin(y) - cos(y)*sin(p)*sin(r)) + z1*(sin(r)*sin(y) + cos(r)*cos(y)*sin(p)) + x*cos(p)*cos(y) + x1*cos(p)*cos(y)
%  y*(cos(r)*cos(y) + sin(p)*sin(r)*sin(y)) + y1*(cos(r)*cos(y) + sin(p)*sin(r)*sin(y)) - z*(cos(y)*sin(r) - cos(r)*sin(p)*sin(y)) - z1*(cos(y)*sin(r) - cos(r)*sin(p)*sin(y)) + x*cos(p)*sin(y) + x1*cos(p)*sin(y)
%                                                                                                                    z*cos(p)*cos(r) - x1*sin(p) - x*sin(p) + z1*cos(p)*cos(r) + y*cos(p)*sin(r) + y1*cos(p)*sin(r)
%                                                                                                                                                                                                                 1
