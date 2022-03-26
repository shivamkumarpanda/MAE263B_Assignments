function [wrapped_angle] = wrapToHalfPi(lambda)
    
    tmp = mod(lambda+pi/2,pi);
    wrapped_angle = tmp+pi*(lambda>0&tmp==0)-pi/2;
end