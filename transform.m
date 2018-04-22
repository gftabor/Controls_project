function [matrix] = transform(roll, pitch, yaw)


t = roll;
first = [1 0 0;...
 0 cos(t) -sin(t);...
 0 sin(t) cos(t)];
t = pitch;
second = [cos(t) 0 sin(t)
          0 1 0;
          -sin(t) 0 cos(t)];
t = yaw;
third = [cos(t) -sin(t) 0;
    sin(t) cos(t) 0;
    0 0 1];
%%ZYX order
matrix = third * second * first;

end

