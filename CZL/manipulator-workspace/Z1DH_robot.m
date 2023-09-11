clc;clear;
L1 = Link('d', 103.5, 'a', 0,        'alpha', -pi/2 ,'standard' );
L2 = Link('d', 0,        'a', 350,   'alpha',   0  ,'standard' );
L3 = Link('d', 0,        'a', 225.3, 'alpha',   0  ,'standard' );
L4 = Link('d', 0,  'a', 72,        'alpha', pi/2 ,'standard' );
L5 = Link('d', 0,  'a',0,        'alpha', -pi/2 ,'standard','offset',-pi/2);
L6 = Link('d', 98.2,  'a',0 ,        'alpha', 0 ,'standard');

 
robot_Z1=SerialLink([L1,L2,L3,L4,L5,L6],'name','Z1');  
robot_Z1.plot([0 0 0 0 0 0]);
robot_Z1.teach(); 

p = transl(-302.9,-7.1,170)*rpy2tr(-45,-90,0);
q = robot_Z1.ikine(p);
robot_Z1.plot(q);
display(q);