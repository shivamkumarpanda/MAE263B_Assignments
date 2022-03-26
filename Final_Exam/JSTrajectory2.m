function [theta] = JSTrajectory2(thetaA, thetaB, n)
%     for i=1:6
%          theta(:,i) = linspace(thetaA(i), thetaB(i), n);
%     end
    [m10, m11, m12, m13] = cubicTrajectory(thetaA', thetaB', n);
    t1 = linspace(0, n, n);
    theta = m10 + m11*t1 + m12*t1.^2 + m13*t1.^3;
    theta = theta';
end