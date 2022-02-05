function [T] = transformationMatrix(DH_row)

 T = [cos(DH_row(4))                     -sin(DH_row(4))                    0                   DH_row(2);
      sin(DH_row(4))*cos(DH_row(1))      cos(DH_row(4))*cos(DH_row(1))      -sin(DH_row(1))     -sin(DH_row(1))*DH_row(3);
      sin(DH_row(4))*sin(DH_row(1))      cos(DH_row(4))*sin(DH_row(1))      cos(DH_row(1))      cos(DH_row(1))*DH_row(3);
      0                                  0                                  0                   1]; 
end