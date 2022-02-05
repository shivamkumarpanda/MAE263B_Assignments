function [theta] = inverseKinematicsScara(M, d1, a1, a2)


    c2 = (M(1,4)^2 + M(2,4)^2 - a1^2 - a2^2)/(2*a1*a2);
    s2 = (1 - c2^2)^(0.5);
    
    t2s1 = atan2(s2, c2);
    t2s2 = atan2(-s2, c2);
    
    if(t2s1 >= 0)
        theta2 = t2s1;
    else
        theta2 = t2s2;
    end
    
    beta = atan2(M(1,4), M(2,4));
    psi = acos((M(1,4)^2 + M(2,4)^2 + a1^2 - a2^2)/(2*a1*(M(1,4)^2 + M(2,4)^2)^(0.5)));
    theta1 = -(beta + psi - pi/2);
    
    theta3 = atan2(M(2,1), M(1,1)) - theta1 - theta2;
    if theta3 < -pi
        theta3 = theta3 + 2*pi;
    elseif theta3 > pi
        theta3 = theta3 - 2*pi;   
    
    end
    d4 = d1 - M(3,4); 
    
    theta = [theta1; theta2; theta3; -d4];

end