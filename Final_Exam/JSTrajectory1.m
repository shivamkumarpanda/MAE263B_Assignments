function [theta] = JSTrajectory1(O1, O2, P0_corner, n)
    
     thetaA = IKPuma3([0 0 0], P0_corner, 0.4318, -0.0203, 0.2435, -0.0934, 0.4331, O1);
     thetaB = IKPuma3([0 0 0], P0_corner, 0.4318, -0.0203, 0.2435, -0.0934, 0.4331, O2);
     

%      for i=1:6
%          theta(:,i) = linspace(thetaA(i), thetaB(i), n);
%      end
%     vmax = [8; 10; 10; 5; 5; 5];
    [m10, m11, m12, m13] = cubicTrajectory(thetaA', thetaB', n);
    t1 = linspace(0, n, n);
    theta = m10 + m11*t1 + m12*t1.^2 + m13*t1.^3;
    theta = theta';
end