function c = trajectoryGen(t0,tf,v0,vf,q0,qf)
    % generates 4x1 matrix of coefficients
    % q are in degrees
    Time = [1, t0, (t0)^2, (t0)^3;
        0, 1, 2*t0, 3*(t0)^2;
        1, tf, (tf)^2, (tf)^3;
        0, 1, 2*tf, 3*(tf)^2];
    Kin = [q0;v0;qf;vf];
    
    c = Time\Kin;
end