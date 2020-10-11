clc;
clear;

%% Create Environment and insert robot
environment = Environment;
environment.generateObjects(environment);
hold on;
robot = KinovaGen3(false);
view(3);

% Rotate camera
% for ax = -5:3:5 % #ok<UNRCH>
%     for by = [[30:-3:0],[0:3:30]]
%         view(ax,by);
%         drawnow();
%         pause(0.01);
%     end
% end


% RGB Can transforms
RedCan = eye(4)*transl(environment.red(1),environment.red(2),environment.red(3));
GreenCan = eye(4)*transl(environment.green(1),environment.green(2),environment.green(3));
BlueCan = eye(4)*transl(environment.blue(1),environment.blue(2),environment.blue(3));


str = input('Which colour should go on top? (Red, Green, Blue)? ','s');
%% This component can be removed once the application is utilised
if strcmp(str, 'Red')
    colourValue = 1;
elseif strcmp(str, 'Green')
    colourValue = 2;
elseif strcmp(str, 'Blue')
    colourValue = 3;
else
    colourValue = 4;
end

%%
[r, g, b] = selectColour(colourValue);

Finalred = makehgtform('translate',[r(1) r(2) r(3)]);
Finalgreen = makehgtform('translate',[g(1) g(2) g(3)]);
Finalblue = makehgtform('translate',[b(1) b(2) b(3)]);

% Set to 1 once ready to test controlFn(1)
run = controlFn(0);

while run
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
    
    % Trajectory to blue
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
end