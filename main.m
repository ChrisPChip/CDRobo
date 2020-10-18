close;
clear;
clc;

%% Can Starting Positions

r = [-0.4 0.2 1.14];
g = [-0.5 0 1.14];
b = [-0.6 -0.2 1.14];

% Create starting can transforms
redStart = transl(r);
greenStart = transl(g);
blueStart = transl(b);

% Transforms for Robot Use
redPos = transl(r)*troty(pi/2)*trotx(pi/2)*trotz(pi);
greenPos = transl(g)*troty(pi/2)*trotx(pi/2)*trotz(pi);
bluePos = transl(b)*troty(pi/2)*trotx(pi/2)*trotz(pi);

%% Create Environment, Robots, and Objects

set(figure(1), 'WindowStyle', 'Docked');

% Create Environment
env = Environment;
env.generateObjects(env);

% Plot Collision boxes
col = Collision;

% Create Robot
robot = KinovaGen3;

% Teach Functionality
% robot.model.teach;

% Can Objects in Environment
redCanLocation = redCan(redStart);
greenCanLocation = greenCan(greenStart);
blueCanLocation = blueCan(blueStart);

hold on;
axis equal;
view(3);

%% Get user to select colour configuration

str = input('Which colour should go on top? (Red, Green, Blue)? ','s');

[redGoalCo, greenGoalCo, blueGoalCo] = selectColour(str);

redGoalPose = transl(redGoalCo)*troty(pi/2)*trotz(pi);
greenGoalPose = transl(greenGoalCo)*troty(pi/2)*trotz(pi);
blueGoalPose = transl(blueGoalCo)*troty(pi/2)*trotz(pi);

%% Initial Values

q0 = [0 0 0 0 0 0];
steps = 64;

%% Animation

% Red Can Sequence

q1 = robot.model.ikcon(redPos);
qMatrix = jtraj(q0, q1, steps);

for i = 1:steps
    robot.model.animate(qMatrix(i,:));
    drawnow();
end

q2 = robot.model.ikcon(redGoalPose);
qMatrix = jtraj(q1, q2, steps);

for i = 1:steps
    robot.model.animate(qMatrix(i,:));
    redStart = robot.model.fkine(qMatrix(i,:));
    delete(redCanLocation);
    redCanLocation = redCan(redStart*troty(pi/2));
    drawnow();
end

qMatrix = jtraj(q2,q0,steps);

for i = 1:steps
    robot.model.animate(qMatrix(i,:));
    drawnow();
end

% Green Can Sequence

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
    greenStart = robot.model.fkine(qMatrix(i,:));
    delete(greenCanLocation);
    greenCanLocation = greenCan(greenStart*troty(pi/2));
    drawnow();
end

qMatrix = jtraj(q4,q0,steps);

for i = 1:steps
    robot.model.animate(qMatrix(i,:));
    drawnow();
end

% Blue Can Sequence

q5 = robot.model.ikcon(bluePos);
qMatrix = jtraj(q0,q5,steps);

for i = 1:steps
    robot.model.animate(qMatrix(i,:));
    drawnow();
end

q6 = robot.model.ikcon(blueGoalPose);
qMatrix = jtraj(q5, q6, steps);

for i = 1:steps
    robot.model.animate(qMatrix(i,:));
    blueStart = robot.model.fkine(qMatrix(i,:));
    delete(blueCanLocation);
    blueCanLocation = blueCan(blueStart*troty(pi/2));
    drawnow();
end

qMatrix = jtraj(q6,q0,steps);

for i = 1:steps
    robot.model.animate(qMatrix(i,:));
    drawnow();
end