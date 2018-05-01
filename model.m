%%variables

clc
close all
clear
syms x y z roll pitch yaw

syms t

syms weapon_location



tic
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
matrix = third *second *   first;  %ditched pitch its annoying
global world_to_robot robot_to_weapon wep_front_transform wep_back_transform
world_to_robot = [[matrix;0,0,0],[x;y;z;1]];
robot_to_weapon = [[transform(pi/2,0,0);0,0,0],[weapon_offset;0;0;1]];


weapon_spin = [cos(weapon_location) -sin(weapon_location) 0;
    sin(weapon_location) cos(weapon_location) 0
    0 0 1];

weapon_to_front_tip = [[weapon_spin;0,0,0],[0;0;0;1]] * [[eye(3);0,0,0],[weapon_radius;0;0;1]];
wep_front_transform = world_to_robot * robot_to_weapon * weapon_to_front_tip;
weapon_to_back_tip = [[weapon_spin;0,0,0],[0;0;0;1]] * [[eye(3);0,0,0],[-weapon_radius;0;0;1]];
wep_back_transform = world_to_robot * robot_to_weapon * weapon_to_back_tip;

global jacob_weapon_front
global jacob_weapon_back
global jacob_chassis
jacob_weapon_front = jacobian(wep_front_transform * [0;0;0;1], state);
jacob_weapon_back = jacobian(wep_back_transform * [0;0;0;1], state) ;

jacob_chassis = jacobian(world_to_robot * [0;-0;0;1], state);

state_pos = [1,0,wheel_radius, 0,0,0,   pi/2];
state_vel = [0,0,0, 0,0,0,  0];
t = 5;


[k1,p1] = Dynamics(jacob_chassis,world_to_robot,chassis_mass,state_dot);
[k2,p2] = Dynamics(jacob_weapon_front,wep_front_transform,weapon_mass/2,state_dot);
[k3,p3] = Dynamics(jacob_weapon_back,wep_back_transform,weapon_mass/2,state_dot);

disp('After energy')
toc

Lagrange = simplify(k1 + k2 + k3 - p1 -p2 - p3);

syms x_t(t) y_t(t) z_t(t) roll_t(t) pitch_t(t) yaw_t(t) weapon_location_t(t)
state_t = [x_t(t) y_t(t) z_t(t) roll_t(t) pitch_t(t) yaw_t(t) weapon_location_t(t)];
Lagrange_dstate = jacobian(Lagrange,state);
Lagrange_dstate_dot = jacobian(Lagrange,state_dot);

torque = diff(subs(Lagrange_dstate_dot,[state,state_dot],[state_t,diff(state_t,t)])) -...
              subs(Lagrange_dstate,[state,state_dot],[state_t,diff(state_t,t)]);
torque = subs(torque,diff(diff(state_t,t),t),state_ddot);
torque = transpose(simplify(subs(torque,[state_t,diff(state_t,t)],[state,state_dot])));
disp('After torque')
toc
M = [
simplify(torque - subs(torque,state_ddot(1),0)),...
simplify(torque - subs(torque,state_ddot(2),0)),...
simplify(torque - subs(torque,state_ddot(3),0)),...
simplify(torque - subs(torque,state_ddot(4),0)),...
simplify(torque - subs(torque,state_ddot(5),0)),...
simplify(torque - subs(torque,state_ddot(6),0)),...
simplify(torque - subs(torque,state_ddot(7),0))];

m = subs(M,state_ddot,[1,1,1,1,1,1,1]);


G = subs(torque,[state_dot,state_ddot],[0,0,0,0,0,0,0, 0,0,0,0,0,0,0]);

c = simplify(torque - m* transpose(state_ddot) - G);

global m_matrix cqdot_matrix g_matrix robot_transform
m_matrix = matlabFunction(m);%magic function I needed the whole time
cqdot_matrix = matlabFunction(c);
g_matrix = matlabFunction(G);
robot_transform = matlabFunction(world_to_robot);

disp('After matlab function')
toc

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
%wheel_points = subs(world_to_robot * transpose([left_wheel;right_wheel]),state,state_pos);

wheel_points = world_to_robot * transpose([left_wheel;right_wheel]);

left_point = wheel_points(1:3,3);
right_point = wheel_points(1:3,8);
left_point_vel = jacobian(left_point,state) * transpose(state_dot)
left_z_vel = left_point_vel(3)
right_point_vel = jacobian(right_point,state) * transpose(state_dot)
right_z_vel = right_point_vel(3)

global right_compute left_compute z_dot_compute roll_dot_compute
right_compute = matlabFunction(right_z_vel)
left_compute = matlabFunction(left_z_vel)

syms left_vel right_vel
solution = solve([left_vel == left_z_vel,right_vel == right_z_vel],[z_dot,roll_dot])

z_dot_compute = matlabFunction(solution.z_dot)
roll_dot_compute = matlabFunction(solution.roll_dot)

tf = 1;

[T,X] = ode45(@(t,x)BattleBotODE(t,x),[0 tf],[state_pos,state_vel]);
disp('After ODE')
toc

figure('Name','Positions red is x');
plot(T, X(:,1),'r-');
hold on
plot(T, X(:,2),'b-');
hold on
plot(T, X(:,3),'g-');
hold on

figure('Name','Angles red is yaw');
plot(T, mod(X(:,6),2*pi),'r-');
hold on
plot(T, mod(X(:,5),2*pi),'b-');
hold on
plot(T, mod(X(:,4),2*pi),'g-');
hold on

figure('Name','Visualize robot');

visualize_bot([state_pos,state_vel]);


%% Definging Functions

    function dx = BattleBotODE(t,x)

        state_pos = x(1:7);
        state_vel = x(8:14);
        %x y z roll pitch yaw
        global left_wheel right_wheel
        global m_matrix cqdot_matrix g_matrix robot_transform       
        world_to_robot = robot_transform(state_pos(5),state_pos(4),state_pos(1),state_pos(2),state_pos(6),state_pos(3));
        Mmat = m_matrix(state_pos(5),state_pos(4),state_pos(7),state_pos(6));
        G_matrix = g_matrix(state_pos(5));
        %(roll,roll_dot,weapon_dot,weapon_location,yaw,yaw_dot)
        cq = cqdot_matrix(state_pos(5),state_vel(5),state_pos(4),state_vel(4),...
                          state_vel(7),state_pos(7),state_pos(6),state_vel(6));
        
        goal_vels = [0,0]; %left, right
        wheel_points = world_to_robot * transpose([left_wheel;right_wheel]);
        
        before_tau = Mmat\(-cq - G_matrix );
        
        global right_compute left_compute z_dot_compute roll_dot_compute
        offset = [0;0;0;0;0;0;0];

        l_vel = left_compute(x(5),x(12),x(4),x(11),x(10));
        r_vel = right_compute(x(5),x(12),x(4),x(11),x(10));
         

        [straight, turn, weapon] = control(state_pos,state_vel,t);
        straight_real = 0;
        turn_real = 0;

        if(min(wheel_points(3,1:5)) > 0)
            %left wheel off ground
        else
            %left wheel on ground
            straight_real = straight_real + straight/2;
            turn_real = turn_real + turn/2;
            offset(3) = offset(3) - before_tau(3)/1.9;
            l_vel = 0 + min(wheel_points(3,1:5))*100;%stop moving it
        end
        if(min(wheel_points(3,6:10)) > 0)
            %right wheel off ground

        else
            %right wheel on ground
            straight_real = straight_real + straight/2;
            turn_real = turn_real + turn/2;
            offset(3) = offset(3) - before_tau(3)/1.9;
            r_vel = 0 + min(wheel_points(3,6:10))*100; %stop moving it
        end
        tau = [cos(state_pos(6)) * straight_real ;sin(state_pos(6)) * straight_real;...
            0;0;0;turn_real;weapon];
        z_dot = z_dot_compute(l_vel,x(5),x(12),r_vel,x(4));
        roll_dot = roll_dot_compute(l_vel,x(5),x(12),r_vel,x(4));
        
        t
        dx=zeros(14,1);
        dx(1:7) = state_vel; 
        dx(8:14) =   before_tau + Mmat\tau ;%+ offset;
        dx(10) = z_dot;
        dx(11) = roll_dot;
        dx(4) = 0;
        dx(11) = 0;
    end
    function [straight, turn, weapon] = control(state,dstate,t)       
        weapon = 0;
        turn = 0;
        straight = 0;
        if(dstate(7) < 1000)
            weapon = 200;
        end
        if(dstate(6) < 0.5)
            turn = 10;

        end    
    end
    function visualize_bot(x)
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




    
