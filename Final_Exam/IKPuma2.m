function [theta] = IKPuma2(P, P0_corner, a2, a3, d2, d3, d4, O )
    
    P = P0_corner + P;

    if (O==1)
        M = [   0   1   0   P(1);
                0   0   1   P(2);
                1   0   0   P(3)
                0   0   0   1];
    elseif (O==2)
        M = [   -1   0   0   P(1);
                0   1   0   P(2);
                0   0   -1   P(3)
                0   0   0   1];
    elseif (O==3)
        M = [   0   0   -1   P(1);
                -1   0   0   P(2);
                0   1   0   P(3)
                0   0   0   1];
    end

    T_6T = [    1       0       0       -0.1;
            0       1       0       0;
            0       0       1       0.13625;
            0       0       0       1];
    
    
    M = M*(T_6T^(-1));

    Px = M(1,4); Py = M(2,4); Pz = M(3,4);

    R11 = M(1,1); R12 = M(1,2); R13 = M(1,3);
    R21 = M(2,1); R22 = M(2,2); R23 = M(2,3);
    R31 = M(3,1); R32 = M(3,2); R33 = M(3,3);



    % theta1
    t1s1 = atan2((Px^2 + Py^2 - (d2 + d3)^2)^0.5, d2 + d3) + atan2(-Px,Py);
    t1s2 = atan2(-(Px^2 + Py^2 - (d2 + d3)^2)^0.5, d2 + d3) + atan2(-Px,Py);
    if(abs(t1s1) <= abs(t1s2))
        t1 = t1s1;
    else
        t1 = t1s2;
    end
    
    % theta3
    K3 = ((Px*cos(t1)+Py*sin(t1))^2 + Pz^2 - (a2^2 + a3^2 + d4^2))/(2*a2);
    t3s1 = atan2((a3^2 + d4^2 - K3^2)^0.5, K3) + atan2(d4, a3);
    t3s2 = atan2(-(a3^2 + d4^2 - K3^2)^0.5, K3) + atan2(d4, a3);
    t3s1 = wrapToPi(t3s1);
    t3s2 = wrapToPi(t3s2);
    if (abs(t3s1)>=pi/2)
        t3 = t3s1;
    else
        t3 = t3s2;
    end

    % theta2
    A = Px*cos(t1)+Py*sin(t1);  C = -Pz;    D = a2 + a3*cos(t3) + d4*sin(t3);
    E = -Pz;                F = -A;         G = a3*sin(t3) - d4*cos(t3);
    t2 = atan2(D*E-A*G, C*G-D*F);

   

    %theta4, theta5, theta6
    t4 = atan2(R23*cos(t1)-R13*sin(t1), R23*sin(t1)*cos(t2+t3) - R13*cos(t1)*cos(t2+t3) - R33*sin(t2+t3));
    t4 = wrapToHalfPi(t4);
    t5 = atan2((R31*cos(t2+t3) + R21*sin(t1)*sin(t2+t3) + R11*cos(t1)*sin(t2+t3)), R31*sin(t2+t3)*cos(t4) - R21*(cos(t1)*sin(t4)...
        + sin(t1)*cos(t4)*cos(t2+t3)) - R11*(-sin(t1)*sin(t4) + cos(t1)*cos(t4)*cos(t2+t3)) );
    t6 = atan2((R21*cos(t1)*cos(t4) - R21*sin(t1)*sin(t4)*cos(t2+t3) - R11*sin(t1)*cos(t4) - R11*cos(t1)*sin(t4)*cos(t2+t3) + R31*sin(t2+t3)*sin(t4)),...
        (R22*cos(t1)*cos(t4) - R22*sin(t1)*sin(t4)*cos(t2+t3) - R12*sin(t1)*cos(t4) - R12*cos(t1)*sin(t4)*cos(t2+t3) + R32*sin(t2+t3)*sin(t4))  );
    
    
    t5 = wrapToHalfPi(t5);
    t6 = wrapToHalfPi(t6);


    theta = [t1, t2, t3, t4, t5, t6];

end