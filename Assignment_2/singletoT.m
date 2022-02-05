function [T_0A] = singletoT(C_0A)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
    kx = C_0A(4,1);
    ky = C_0A(5,1);
    kz = C_0A(6,1);
    theta = (kx^2 + ky^2 + kz^2)^0.5;
    if (theta ~= 0)
        kx = kx/theta;
        ky = ky/theta;
        kz = kz/theta;
    end
    T_0A = eye(4);
    T_0A(1,1) = kx*kx*(1-cos(theta)) + cos(theta);
    T_0A(1,2) = kx*ky*(1-cos(theta)) - kz*sin(theta);
    T_0A(1,3) = kx*kz*(1-cos(theta)) + ky*sin(theta);
    T_0A(2,1) = kx*ky*(1-cos(theta)) + kz*sin(theta);
    T_0A(2,2) = ky*ky*(1-cos(theta)) + cos(theta);
    T_0A(2,3) = ky*kz*(1-cos(theta)) - kx*sin(theta);
    T_0A(3,1) = kx*kz*(1-cos(theta)) - ky*sin(theta);
    T_0A(3,2) = ky*kz*(1-cos(theta)) - kx*sin(theta);
    T_0A(3,3) = kz*kz*(1-cos(theta)) + cos(theta);

    T_0A(1:3, 4) = C_0A(1:3); 
end