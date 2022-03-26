function [C_0A] = singleAxis(T_0A)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
    C_0A = T_0A(1:3, 4);
    
    theta = acos((T_0A(1,1) + T_0A(2,2) + T_0A(3,3) -1)/2);
    if(theta == pi)
        K = [0; 0; 1];
    else
        K = (1/2*sin(theta))*[T_0A(3,2) - T_0A(2,3); T_0A(1,3) - T_0A(3,1); T_0A(2,1) - T_0A(1,2)];
    end
    C_0A = [C_0A; K*theta]; 
end