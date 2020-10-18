function motionTestscript

close;
clear;
clc;
set(figure(1), 'WindowStyle', 'Docked');
env = Environment;
env.generateObjects(env);

%% Can Initial Positions

% Initial starting Coordinates
r = [-0.4 0.2 1.14];
g = [-0.5 0 1.14];
b = [-0.6 -0.2 1.14];

% Create starting can transforms
redStart = transl(r);
greenStart = transl(g);
blueStart = transl(b);

% Build cans in the environment
redCanLocation = redCan(redStart);
greenCanLocation = greenCan(greenStart);
blueCanLocation = blueCan(blueStart);

redPos = transl(r)*troty(pi/2)*trotx(pi/2)*trotz(pi);
greenPos = transl(g)*troty(pi/2)*trotx(pi/2)*trotz(pi);
bluePos = transl(b)*troty(pi/2)*trotx(pi/2)*trotz(pi);

%% Select order of Cans on the shelf

str = input('Which colour should go on top? (Red, Green, Blue)? ','s');
if strcmp(str, 'Red')
    colourValue = 1;
    
elseif strcmp(str, 'Green')
    colourValue = 2;
    
elseif strcmp(str, 'Blue')
    colourValue = 3;
    
else
    colourValue = 4;
end

% Assign goal positions
[rGoalCo, gGoalCo, bGoalCo] = selectColour(colourValue);

% Final Positions of the cans
redGoalPose = transl(rGoalCo);
greenGoalPose = transl(gGoalCo);
blueGoalPose = transl(bGoalCo);

%% TEST FINAL POSITIONS

% Build cans in the environment
redGLocation = redCan(redGoalPose);
greenGLocation = greenCan(greenGoalPose);
blueGLocation = blueCan(blueGoalPose);

%% To be Removed
hold on;
axis equal;
view(3);
camlight;

%% IF STATEMENT USED FOR TESTING
go = 0;
if go

robot = KinovaGen3;
hold on;
axis equal;
view(3);

pause(3);

q0 = [0 0 0 0 0 0];
steps = 64;

%% GET RID OF THIS
% go = 0;
% if go

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
        blueStart = robot.model.fkine(qMatrix(i,:));
        delete(blueCanLocation);
        blueCanLocation = blueCan(blueStart*troty(pi/2));
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
        redStart = robot.model.fkine(qMatrix(i,:));
        delete(redCanLocation);
        redCanLocation = redCan(redStart*troty(pi/2));
        drawnow();
    end
    
    qMatrix = jtraj(q6,q0,steps);
    
    for i = 1:steps
        robot.model.animate(qMatrix(i,:));
        drawnow();
    end
end

end