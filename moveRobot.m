function moveRobot(robot, q1, q2)
%% This function is used to move the robot in a trapezoidal ramp-up and ramp-down velocity control

% Poses and respective transforms

steps = 50;

%% LSPB - Scalar trajectory that uses a constant velocity segmont and parabolic bends (a trapezoidal path)

% First, create the scalar function
s = lspb(0, 1, steps);

% Create memory allocation for the variables
qMatrix = nan(steps, 6);

% Generate interpolated joint angles
for i = 1:steps
    qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
end

% Initialise velocity and acceleration containers
% velocity = zeros(steps, 6);
% acceleration = zeros(steps, 6);

for i = 2:steps
    
    robotMove = qMatrix(i,:);
%     velocity(i,:) = qMatrix(i,:) - qMatrix(i-1,:);
%     acceleration(i,:) = velocity(i,:) - velocity(i-1,:);
    robot.model.animate(robotMove);
    drawnow();
end
end