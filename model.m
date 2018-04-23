%%variables

clc
clear
syms x y z roll pitch yaw

syms t

syms weapon_location




%weapon_vel = diff(weapon_location,t);

global state state_dot state_ddot
syms x_dot y_dot z_dot roll_dot pitch_dot yaw_dot weapon_dot
syms x_ddot y_ddot z_ddot roll_ddot pitch_ddot yaw_ddot weapon_ddot

state = [x y z roll pitch yaw,weapon_location];
state_dot = [x_dot y_dot z_dot roll_dot pitch_dot yaw_dot weapon_dot];
state_ddot = [x_ddot y_ddot z_ddot roll_ddot pitch_ddot yaw_ddot weapon_ddot];


%%robot constants 
chassis_length = 0.6; %real
chassis_width = 0.6;  %real
weapon_offset = 0.2;
wheel_radius = 0.1; %estimate
weapon_height = wheel_radius;
weapon_mass = 33; %real
weapon_radius = 0.085; %real

chassis_mass = 80; %real
%robot point mass
%weapon point mass

 

%%Trust me this was best way to do it
first = [1, 0, 0;...
 0, cos(roll), -sin(roll);...
 0, sin(roll), cos(roll)];

second = [cos(pitch) 0 sin(pitch)
          0 1 0;
          -sin(pitch) 0 cos(pitch)];
third = [cos(yaw) -sin(yaw) 0;
    sin(yaw) cos(yaw) 0
    0 0 1];

%%ZYX order
matrix = third * second * first;
global world_to_robot robot_to_weapon wep_front_transform wep_back_transform
world_to_robot = [[matrix;0,0,0],[x;y;z;1]];
robot_to_weapon = [[transform(0,pi/2,0);0,0,0],[weapon_offset;0;0;1]];


weapon_spin = [cos(weapon_location) -sin(weapon_location) 0;
    sin(weapon_location) cos(weapon_location) 0
    0 0 1];

weapon_to_front_tip = [[weapon_spin;0,0,0],[weapon_radius;0;0;1]];
wep_front_transform = world_to_robot * robot_to_weapon * weapon_to_front_tip;
weapon_to_back_tip = [[weapon_spin;0,0,0],[-weapon_radius;0;0;1]];
wep_back_transform = world_to_robot * robot_to_weapon * weapon_to_back_tip;

global jacob_weapon_front
global jacob_weapon_back
global jacob_chassis
jacob_weapon_front = jacobian(wep_front_transform * [0;0;0;1], state);
jacob_weapon_back = jacobian(wep_back_transform * [0;0;0;1], state) ;

jacob_chassis = jacobian(world_to_robot * [0;-0;0;1], state);

state_pos = [1,0,wheel_radius, 0,0,0,  pi/2];
state_vel = [0,0,0, 0,0,0,  0];
t = 5;


[k1,p1] = Dynamics(jacob_chassis,world_to_robot,chassis_mass,state_dot);
[k2,p2] = Dynamics(jacob_weapon_front,wep_front_transform,weapon_mass/2,state_dot);
[k3,p3] = Dynamics(jacob_weapon_back,wep_back_transform,weapon_mass/2,state_dot);

Lagrange = simplify(k1 + k2 + k3 - p1 -p2 - p3)

syms x_t(t) y_t(t) z_t(t) roll_t(t) pitch_t(t) yaw_t(t) weapon_location_t(t)
state_t = [x_t(t) y_t(t) z_t(t) roll_t(t) pitch_t(t) yaw_t(t) weapon_location_t(t)];
syms 
Lagrange_dstate = jacobian(Lagrange,state);
Lagrange_dstate_dot = jacobian(Lagrange,state_dot);

torque = diff(subs(Lagrange_dstate_dot,[state,state_dot],[state_t,diff(state_t,t)])) -...
              subs(Lagrange_dstate,[state,state_dot],[state_t,diff(state_t,t)]);
torque = subs(torque,diff(diff(state_t,t),t),state_ddot);
torque = simplify(subs(torque,[state_t,diff(state_t,t)],[state,state_dot]))




BattleBotODE(t,[state_pos,state_vel])



%Visualiziation setup
global chassis
chassis = [[-chassis_width/2,-chassis_length/2,0,1];...
   [-chassis_width/2,chassis_length/2,0,1];...
   [chassis_width/2,chassis_length/2,0,1];...
   [chassis_width/2,-chassis_length/2,0,1]];
wheel_angle = pi/6;
global left_wheel right_wheel
left_wheel = [[0,-chassis_width/2,0,1];...%center
              [-wheel_radius*sin(wheel_angle),-chassis_width/2,-wheel_radius*cos(wheel_angle),1];...%forward
              [0,-chassis_width/2,-wheel_radius*cos(0),1];...%middle
              [wheel_radius*sin(wheel_angle),-chassis_width/2,-wheel_radius*cos(wheel_angle),1];...%backward
              [0,-chassis_width/2,0,1]]; %center
right_wheel = [[0,chassis_width/2,0,1];...%center
              [-wheel_radius*sin(wheel_angle),chassis_width/2,-wheel_radius*cos(wheel_angle),1];...%forward
              [0,chassis_width/2,-wheel_radius*cos(0),1];...%middle
              [wheel_radius*sin(wheel_angle),chassis_width/2,-wheel_radius*cos(wheel_angle),1];...%backward
              [0,chassis_width/2,0,1]]; %center
wheel_points = subs(world_to_robot * transpose([left_wheel;right_wheel]),state,state_pos);

%visualize([state_pos,state_vel]);


% g = 9.81;
% k = .5 * (weapon_mass * sum( velocity_weapon(1:3).^ 2) ...
%     + chassis_mass * sum( jacob_chassis(1:3).^ 2)...
%     + Moment_fly * weapon_vel * weapon_vel);
% p = wep_transform(2,4) * weapon_mass * g + world_to_robot(2,4) * chassis_mass * g;

%% Definging Functions

    function dx = BattleBotODE(t,x)
        global state
        global jacob_chassis
        global world_to_robot
        global left_wheel right_wheel


        state_pos = x(1:7);
        state_vel = x(8:14);
    
        wheel_points = subs(world_to_robot * transpose([left_wheel;right_wheel]),state,state_pos);
        

    
    
    
    end
    
    function visualize(x)
        global state
        global world_to_robot robot_to_weapon wep_front_transform wep_back_transform
        global left_wheel right_wheel
        global chassis

        state_pos = x(1:7);
        state_vel = x(8:14);
    
        wheel_points = subs(world_to_robot * transpose([left_wheel;right_wheel]),state,state_pos);
    


        chassis_corners =  subs(world_to_robot * transpose(chassis),state,state_pos);
        weapon_point =  subs(world_to_robot * robot_to_weapon * [0;0;0;1],state,state_pos);
        front_tip = subs(wep_front_transform * [0;0;0;1],state,state_pos);
        back_tip = subs(wep_back_transform * [0;0;0;1],state,state_pos);
        
        
        plot3([chassis_corners(1,:),chassis_corners(1,1)],[chassis_corners(2,:),chassis_corners(2,1)],[chassis_corners(3,:),chassis_corners(3,1)])
        hold on
        plot3([chassis_corners(1,4),weapon_point(1),chassis_corners(1,3)]...
            ,[chassis_corners(2,4),weapon_point(2),chassis_corners(2,3)],...
            [chassis_corners(3,4),weapon_point(3),chassis_corners(3,3)])
        hold on
        weapon = [back_tip,front_tip];
        plot3(weapon(1,:),weapon(2,:),weapon(3,:))
        hold on
        plot3(wheel_points(1,:),wheel_points(2,:),wheel_points(3,:))
        daspect([1 1 1])
    
    end




    
