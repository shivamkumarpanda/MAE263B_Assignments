clear all; 
close all; 
clc
%% define robots
L(1) = Link('revolute','d',0.4,'a',0,'alpha',0,'modified');
L(2) = Link('revolute','d',0,'a',0.325,'alpha',0,'modified');
L(3) = Link('revolute','d',0,'a',0.225,'alpha',0,'modified');
L(4) = Link('prismatic','a',0,'alpha',pi,'modified');
% tool = transl(0, 0, -0.01);
L(4).qlim = [ 0.17  0.32];
% SCARA = SerialLink([L(1),L(2),L(3),L(4)], 'tool', tool);
SCARA = SerialLink([L(1),L(2),L(3),L(4)]);
SCARA.name = 'SCARA';
SCARA.plot([0 pi/2 0 0.01])