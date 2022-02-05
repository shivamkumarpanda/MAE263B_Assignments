function [t, theta] = totalTrajectory(thetaA , thetaA1, thetaB1, thetaB, vmax)

    [m10, m11, m12, m13, tf1] = cubicTrajectory(thetaA, thetaA1, vmax);
    t1 = [0:0.002:tf1]; t1 = [t1, tf1];
    thetaPiece1 = m10 + m11*t1 + m12*t1.^2 + m13*t1.^3;
    
    [m20, m21, m22, m23, tf2] = cubicTrajectory(thetaA1, thetaB1, vmax);
    t2 = [0:0.002:tf2]; t2 = [t2, tf2];
    thetaPiece2 = m20 + m21*t2 + m22*t2.^2 + m23*t2.^3;
    
    [m30, m31, m32, m33, tf3] = cubicTrajectory(thetaB1, thetaB, vmax);
    t3 = [0:0.002:tf3]; t3 = [t3, tf3];
    thetaPiece3 = m30 + m31*t3 + m32*t3.^2 + m33*t3.^3;
    
    t2 = t2 + tf1;
    t3 = t3 + tf1 + tf2; 
    t = [t1, t2, t3];
    theta = [thetaPiece1, thetaPiece2, thetaPiece3];
end