clc;
clear;

% safetyClass = Safety_Circuit;
% safetyState = safetyClass.checkState;

environment = Environment;
environment.generateObjects(environment);
hold on;
robot = KinovaGen3(false);

RedCan = eye(4)*transl(environment.red(1),environment.red(2),environment.red(3));
GreenCan = eye(4)*transl(environment.green(1),environment.green(2),environment.green(3));
BlueCan = eye(4)*transl(environment.blue(1),environment.blue(2),environment.blue(3));
str = input('Which colour should go on top? (red,grn,blu)','s');
if str == 'red'
    xr = 0; yr = 2; zr = 1.9;
    xb = -.3; yb = 2; zb = 1;
    xg = .3; yg = 2; zg = 1;
elseif str == 'blu'
    xr = -.3; yr = 2; zr = 1;
    xb = 0; yb = 2; zb = 1.9;
    xg = .3; yg = 2; zg = 1;
elseif str == 'grn'
    xr = -.3; yr = 2; zr = 1;
    xb = .3; yb = 2; zb = 1;
    xg = 0; yg = 2; zg = 1.9;
else
    disp ('Incorrect Input, placing all cans in middle shelf')
    xr = -.3; yr = 2; zr = 1;
    xb = 0; yb = 2; zb = 1;
    xg = .3; yg = 2; zg = 1;
end
Finalred = makehgtform('translate',[xr,yr,zr]);
Finalblue = makehgtform('translate',[xb,yb,zb]);
Finalgreen = makehgtform('translate',[xg,yg,zg]);
%% Animations
q0 = [0 0 0 0 0 0 0];
t1 = [0:0.5:8];
q1 = robot.model.ikcon(RedCan,q0);
Trajred = jtraj(q0,q1,t1);

for i = 1:size(Trajred,1)
    robot.model.animate(Trajred(i,:));
    drawnow();
end

q2 = robot.model.ikcon(Finalred,q1);
TrajFred = jtraj(q1,q2,t1);

for i = 1:size(TrajFred,1)
    robot.model.animate(TrajFred(i,:));
    drawnow();
end

% Treajectory to blue
q3 = robot.model.ikcon(q2);
Trajblue = jtraj(q2,q3,t1);

for i = 1:size(Trajblue,1)
    robot.model.animate(Trajblue(i,:));
    drawnow();
end

q4 = robot.model.ikcon(Finalblue,q3);
TrajFblue = jtraj(q3,q4,t1);

for i = 1:size(TrajFblue,1)
    robot.model.animate(TrajFblue(i,:));
    drawnow();
end

% Trajectory to green
q5 = robot.model.ikcon(GreenCan,q4);
Trajgreen = jtraj(q4,q5,t1);

for i = 1:size(Trajgreen,1)
    robot.model.animate(Trajgreen(i,:));
    drawnow();
end

q6 = robot.model.ikcon(Finalgreen,q5);
TrajFgreen = jtraj(q5,q6,t1);

for i = 1:size(TrajFgreen,1)
    robot.model.animate(TrajFgreen(i,:));
    drawnow();
end