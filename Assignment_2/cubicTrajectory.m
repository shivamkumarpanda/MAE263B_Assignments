function [m0, m1, m2, m3, tf] = cubicTrajectory(theta0, theta1, vmax)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    tf_array = 1.5*abs(theta1 - theta0)./vmax;
    tf = max(tf_array);
    m0 = theta0;
    m1 = 0;
    m2 = (3/tf^2)*(theta1 - theta0);
    m3 = (-2/tf^3)*(theta1 - theta0);
end