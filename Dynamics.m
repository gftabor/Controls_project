function  [k,p] = Dynamics(jacob,transform, mass, state_dot)
    %UNTITLED2 Summary of this function goes here
    %   Detailed explanation goes here
    g= 9.81;
    velocity = simplify(jacob * transpose(state_dot));
    vel2 = sum(velocity.^2);
    velmag = simplify(vel2 ^.5);
    k = (1/2)*mass *vel2;
    p = simplify(transform(3,4)*mass*g);


end

