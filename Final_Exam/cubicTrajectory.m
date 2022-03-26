function [m0, m1, m2, m3] = cubicTrajectory(theta0, theta1, n)

    tf = n;
    m0 = theta0;
    m1 = 0;
    m2 = (3/tf^2)*(theta1 - theta0);
    m3 = (-2/tf^3)*(theta1 - theta0);
end