function [theta] = IKPuma(M, a2, a3, d2, d3, d4 )
    
    Px = M(1,4); Py = M(2,4); Pz = M(3,4);

    R11 = M(1,1); R12 = M(1,2); R13 = M(1,3);
    R21 = M(2,1); R22 = M(2,2); R23 = M(2,3);
    R31 = M(3,1); R32 = M(3,2); R33 = M(3,3);
    % theta1
    t1s1 = atan2((Px^2 + Py^2 - (d2 + d3)^2)^0.5, d2 + d3) + atan2(-Px,Py);
    t1s2 = atan2(-(Px^2 + Py^2 - (d2 + d3)^2)^0.5, d2 + d3) + atan2(-Px,Py);

    %theta3 with t1 as t1s1
    t1 = t1s1;
    K3 = ((Px*cos(t1)+Py*sin(t1))^2 + Pz^2 - (a2^2 + a3^2 + d4^2))/(2*a2);
    t3s1 = atan2((a3^2 + d4^2 - K3^2)^0.5, K3) + atan2(d4, a3);
    t3s2 = atan2(-(a3^2 + d4^2 - K3^2)^0.5, K3) + atan2(d4, a3);
    t3s1 = wrapToPi(t3s1);
    t3s2 = wrapToPi(t3s2);

    %theta2 with t1s1 and t3s1
    t1 = t1s1;
    t3 = t3s1;
    A = Px*cos(t1)+Py*sin(t1);  C = -Pz;    D = a2 + a3*cos(t3) + d4*sin(t3);
    E = -Pz;                F = -A;         G = a3*sin(t3) - d4*cos(t3);
    t2s1 = atan2(D*E-A*G, C*G-D*F);

    %theta2 with t1s1 and t3s2
    t1 = t1s1;
    t3 = t3s2;
    A = Px*cos(t1)+Py*sin(t1);  C = -Pz;    D = a2 + a3*cos(t3) + d4*sin(t3);
    E = -Pz;                F = -A;         G = a3*sin(t3) - d4*cos(t3);
    t2s2 = atan2(D*E-A*G, C*G-D*F);

    %theta3 with t1 as t1s2
    t1 = t1s2;
    K3 = ((Px*cos(t1)+Py*sin(t1))^2 + Pz^2 - (a2^2 + a3^2 + d4^2))/(2*a2);
    t3s3 = atan2((a3^2 + d4^2 - K3^2)^0.5, K3) + atan2(d4, a3);
    t3s4 = atan2(-(a3^2 + d4^2 - K3^2)^0.5, K3) + atan2(d4, a3);
    t3s3 = wrapToPi(t3s3);
    t3s4 = wrapToPi(t3s4);

    %theta2 with t1s2 and t3s3
    t1 = t1s2;
    t3 = t3s3;
    A = Px*cos(t1)+Py*sin(t1);  C = -Pz;    D = a2 + a3*cos(t3) + d4*sin(t3);
    E = -Pz;                F = -A;         G = a3*sin(t3) - d4*cos(t3);
    t2s3 = atan2(D*E-A*G, C*G-D*F);
    
    %theta2 with t1s2 and t3s4
    t1 = t1s2;
    t3 = t3s4;
    A = Px*cos(t1)+Py*sin(t1);  C = -Pz;    D = a2 + a3*cos(t3) + d4*sin(t3);
    E = -Pz;                F = -A;         G = a3*sin(t3) - d4*cos(t3);
    t2s4 = atan2(D*E-A*G, C*G-D*F);
    
    theta123 = [t1s1, t2s1, t3s1;
            t1s1, t2s2, t3s2;
            t1s2, t2s3, t3s3;
            t1s2, t2s4, t3s4];

    %theta4 with t1s1, t2s1, t3s1 
    t1 = t1s1;  t2 = t2s1;  t3 = t3s1;
    t4s1 = atan2(R23*cos(t1)-R13*sin(t1), R23*sin(t1)*cos(t2+t3) - R13*cos(t1)*cos(t2+t3) - R33*sin(t2+t3));
    t4s1 = wrapToHalfPi(t4s1);
    t4 = t4s1;
    t5s1 = atan2((R31*cos(t2+t3) + R21*sin(t1)*sin(t2+t3) + R11*cos(t1)*sin(t2+t3)), R31*sin(t2+t3)*cos(t4) - R21*(cos(t1)*sin(t4)...
        + sin(t1)*cos(t4)*cos(t2+t3)) - R11*(-sin(t1)*sin(t4) + cos(t1)*cos(t4)*cos(t2+t3)) );
    t6s1 = atan2((R21*cos(t1)*cos(t4) - R21*sin(t1)*sin(t4)*cos(t2+t3) - R11*sin(t1)*cos(t4) - R11*cos(t1)*sin(t4)*cos(t2+t3) + R31*sin(t2+t3)*sin(t4)),...
        (R22*cos(t1)*cos(t4) - R22*sin(t1)*sin(t4)*cos(t2+t3) - R12*sin(t1)*cos(t4) - R12*cos(t1)*sin(t4)*cos(t2+t3) + R32*sin(t2+t3)*sin(t4))  );
    
    
    t5s1 = wrapToHalfPi(t5s1);
    t6s1 = wrapToHalfPi(t6s1);

    %theta4 with t1s1, t2s2, t3s2
    t1 = t1s1; t2 = t2s2; t3 = t3s2;
    t4s2 = atan2(R23*cos(t1)-R13*sin(t1), R23*sin(t1)*cos(t2+t3) - R13*cos(t1)*cos(t2+t3) - R33*sin(t2+t3));
    t4s2 = wrapToHalfPi(t4s2);
    t4 = t4s2;
    t5s2 = atan2((R31*cos(t2+t3) + R21*sin(t1)*sin(t2+t3) + R11*cos(t1)*sin(t2+t3)), R31*sin(t2+t3)*cos(t4) - R21*(cos(t1)*sin(t4)...
        + sin(t1)*cos(t4)*cos(t2+t3)) - R11*(-sin(t1)*sin(t4) + cos(t1)*cos(t4)*cos(t2+t3)) );
    t6s2 = atan2((R21*cos(t1)*cos(t4) - R21*sin(t1)*sin(t4)*cos(t2+t3) - R11*sin(t1)*cos(t4) - R11*cos(t1)*sin(t4)*cos(t2+t3) + R31*sin(t2+t3)*sin(t4)),...
        (R22*cos(t1)*cos(t4) - R22*sin(t1)*sin(t4)*cos(t2+t3) - R12*sin(t1)*cos(t4) - R12*cos(t1)*sin(t4)*cos(t2+t3) + R32*sin(t2+t3)*sin(t4))  );
    
    
    t5s2 = wrapToHalfPi(t5s2);
    t6s2 = wrapToHalfPi(t6s2);
    

    %theta4 with t1s2, t2s3, t3s3
    t1 = t1s2; t2 = t2s3; t3 = t3s3;
    t4s3 = atan2(R23*cos(t1)-R13*sin(t1), R23*sin(t1)*cos(t2+t3) - R13*cos(t1)*cos(t2+t3) - R33*sin(t2+t3));
    t4s3 = wrapToHalfPi(t4s3);
    t4 = t4s3;
    t5s3 = atan2((R31*cos(t2+t3) + R21*sin(t1)*sin(t2+t3) + R11*cos(t1)*sin(t2+t3)), R31*sin(t2+t3)*cos(t4) - R21*(cos(t1)*sin(t4)...
        + sin(t1)*cos(t4)*cos(t2+t3)) - R11*(-sin(t1)*sin(t4) + cos(t1)*cos(t4)*cos(t2+t3)) );
    t6s3 = atan2((R21*cos(t1)*cos(t4) - R21*sin(t1)*sin(t4)*cos(t2+t3) - R11*sin(t1)*cos(t4) - R11*cos(t1)*sin(t4)*cos(t2+t3) + R31*sin(t2+t3)*sin(t4)),...
        (R22*cos(t1)*cos(t4) - R22*sin(t1)*sin(t4)*cos(t2+t3) - R12*sin(t1)*cos(t4) - R12*cos(t1)*sin(t4)*cos(t2+t3) + R32*sin(t2+t3)*sin(t4))  );
    
    
    t5s3 = wrapToHalfPi(t5s3);
    t6s3 = wrapToHalfPi(t6s3);


    %theta4 with t1s2, t2s4, t3s4
    t1 = t1s2; t2 = t2s4; t3 = t3s4;
    t4s4 = atan2(R23*cos(t1)-R13*sin(t1), R23*sin(t1)*cos(t2+t3) - R13*cos(t1)*cos(t2+t3) - R33*sin(t2+t3));
    t4s4 = wrapToHalfPi(t4s4);
    t4 = t4s4;
    t5s4 = atan2((R31*cos(t2+t3) + R21*sin(t1)*sin(t2+t3) + R11*cos(t1)*sin(t2+t3)), (R31*sin(t2+t3)*cos(t4) - R21*(cos(t1)*sin(t4)...
        + sin(t1)*cos(t4)*cos(t2+t3)) - R11*(-sin(t1)*sin(t4) + cos(t1)*cos(t4)*cos(t2+t3))) );
    t6s4 = atan2((R21*cos(t1)*cos(t4) - R21*sin(t1)*sin(t4)*cos(t2+t3) - R11*sin(t1)*cos(t4) - R11*cos(t1)*sin(t4)*cos(t2+t3) + R31*sin(t2+t3)*sin(t4)),...
        (R22*cos(t1)*cos(t4) - R22*sin(t1)*sin(t4)*cos(t2+t3) - R12*sin(t1)*cos(t4) - R12*cos(t1)*sin(t4)*cos(t2+t3) + R32*sin(t2+t3)*sin(t4))  );
    
    
    t5s4 = wrapToHalfPi(t5s4);
    t6s4 = wrapToHalfPi(t6s4);

    theta = [t1s1, t2s1, t3s1, t4s1, t5s1, t6s1;
            t1s1, t2s2, t3s2, t4s2, t5s2, t6s2;
            t1s2, t2s3, t3s3, t4s3, t5s3, t6s3;
            t1s2, t2s4, t3s4, t4s4, t5s4, t6s4];

end