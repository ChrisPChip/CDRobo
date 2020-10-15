function motionTestscript

close;
clear;
clc;


r = [-0.4 3 (1.08 + 0.06)];
g = [-0.8 3 (1.08 + 0.06)];
b = [0.6 3 (1.08 + 0.06)];

rGoalCo = [0.5 2.5 (1.08 + 0.06)];
gGoalCo = [0.3 2.3 (1.08 + 0.06)];
bGoalCo = [-0.3 2.5 (1.08 + 0.06)];

redGoalPose = transl(rGoalCo)*troty(pi/2)*trotx(pi/2)*trotz(pi);
greenGoalPose = transl(gGoalCo)*troty(pi/2)*trotx(pi/2)*trotz(pi);
blueGoalPose = transl(bGoalCo)*troty(pi/2)*trotx(pi/2)*trotz(pi);

redTransl = transl(r);
greenTransl = transl(g);
blueTransl = transl(b);

redCanLocation = redCan(redTransl);
greenCanLocation = greenCan(greenTransl);
blueCanLocation = blueCan(blueTransl);

redPos = transl(r)*troty(pi/2)*trotx(pi/2)*trotz(pi);
greenPos = transl(g)*troty(pi/2)*trotx(pi/2)*trotz(pi);
bluePos = transl(b)*troty(pi/2)*trotx(pi/2)*trotz(pi);

robot = KinovaGen3;
hold on;
axis equal;

pause(3);

q0 = [0 0 0 0 0 0];
steps = 64;
%% Blue Can Sequence

q1 = robot.model.ikcon(bluePos);
qMatrix = jtraj(q0,q1,steps);

for i = 1:steps
    robot.model.animate(qMatrix(i,:));
    drawnow();
end

q2 = robot.model.ikcon(blueGoalPose);
qMatrix = jtraj(q1, q2, steps);

for i = 1:steps
    robot.model.animate(qMatrix(i,:));
    blueTransl = robot.model.fkine(qMatrix(i,:));
    delete(blueCanLocation);
    blueCanLocation = blueCan(blueTransl*troty(pi/2));
    drawnow();
end

qMatrix = jtraj(q2,q0,steps);

for i = 1:steps
    robot.model.animate(qMatrix(i,:));
    drawnow();
end


%% Green Can Sequence

q3 = robot.model.ikcon(greenPos);
qMatrix = jtraj(q0, q3, steps);

for i = 1:steps
    robot.model.animate(qMatrix(i,:));
    drawnow();
end

q4 = robot.model.ikcon(greenGoalPose);
qMatrix = jtraj(q3, q4, steps);

for i = 1:steps
    robot.model.animate(qMatrix(i,:));
    greenTransl = robot.model.fkine(qMatrix(i,:));
    delete(greenCanLocation);
    greenCanLocation = greenCan(greenTransl*troty(pi/2));
    drawnow();
end

qMatrix = jtraj(q4,q0,steps);

for i = 1:steps
    robot.model.animate(qMatrix(i,:));
    drawnow();
end

%% Red Can Sequence

q5 = robot.model.ikcon(redPos);
qMatrix = jtraj(q0, q5, steps);

for i = 1:steps
    robot.model.animate(qMatrix(i,:));
    drawnow();
end

q6 = robot.model.ikcon(redGoalPose);
qMatrix = jtraj(q5, q6, steps);

for i = 1:steps
    robot.model.animate(qMatrix(i,:));
    redTransl = robot.model.fkine(qMatrix(i,:));
    delete(redCanLocation);
    redCanLocation = redCan(redTransl*troty(pi/2));
    drawnow();
end

qMatrix = jtraj(q6,q0,steps);

for i = 1:steps
    robot.model.animate(qMatrix(i,:));
    drawnow();
end

end