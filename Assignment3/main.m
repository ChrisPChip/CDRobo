clear;
close all;
clc;

%% Create Envirnoment
workspace = [0.5 2 -0.5 2 0 3];
axis equal;
view(3);

%% Setup Objects
drumCoord = [1, 0, 0];

% Student number: 1197 1429
studentID = [1197 1429];
robotBase = [(studentID(1))/1000 (studentID(2))/1000 1];

%% Show the drum transform
disp('Drum Transform');
drumTransform = transl(drumCoord(1), drumCoord(2), drumCoord(3))

% Read drum ply data
[f,v,data] = plyread('Drum.ply','tri');
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
floorMesh_h = trisurf(f,v(:,1) + drumCoord(1),v(:,2) + drumCoord(2), v(:,3) + drumCoord(3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

%% Show the robot base transform
disp('Robot base transform');
baseTransform = transl(robotBase(1), robotBase(2),1)

% Starting Joint Angles
q0 = [0 0 0 0 0 0];

% Create Robot
mdl_puma560
p560.base = baseTransform;

%% Tool Transform
toolTransform = transl(0, 0, 0.2) * troty(pi/4);
disp('Tool Transform');
p560.tool = toolTransform

% Plot robot in environment
p560.plot(q0, 'workspace', workspace, 'scale', 0.3);

%% Show  the transform between the robot base and the drum
disp('Transform from base to drum');
b2deltaTransform = inv(baseTransform) * drumTransform

%% Animations
% Time variables required for graphing
time = 0.5;
deltaT = 1/60;
steps = time / deltaT;

% Move to first corner position
firstWindowPos = [1.392 0.6495 0.5976];
firstWindowTrans = transl(firstWindowPos) * troty(-pi/2) * trotz(pi/4) * trotx(-pi/8);
disp('Moving to first corner');
q1 = p560.ikcon(firstWindowTrans);
qMatrix = jtraj(q0, q1, steps);
p560.plot(qMatrix);

% Move to second corner position
secondWindowPos = [1.228 0.6746 0.5976];
secondWindowTrans = transl(secondWindowPos) * troty(-pi/2) * trotz(pi/4) * trotx(-pi/8);
disp('Moving to second corner');
q2 = p560.ikcon(secondWindowTrans);
qMatrix = jtraj(q1, q2, steps);
% p560.plot(qMatrix);   % USED FOR TESTING - REMOVE ONCE COMPLETE

%% Given P560 Values
% qDot = [8 10 10 5 5 5];
% q2Dot = [10 12 12 8 8 8];
tauMax = [97.6 186.4 89.4 24.2 20.1 21.3];

s = lspb(0, 1, steps);
for i = 1:steps
    q(i,:) = (1-s(i)) * q1 + s(i) * q2;
end

qDot = zeros(steps,6);                                                       
q2Dot = nan(steps,6);                                                        
torque = nan(steps,6);                                                        
mass = 2.09;                                                               
p560.payload(mass,[0;0;0]);                                                
J = p560.jacob0(zeros(1,6));
wieighing = [0 0 -209 0 0 0]';

% RMRC and Calucation
for i = 1:steps-1
    q2Dot(i,:) = (1/deltaT)^2 * (q(i+1,:) - q(i,:) - deltaT * qDot(i,:));                 
    intertia = p560.inertia(q(i,:));                                               
    coriolis = p560.coriolis(q(i,:),qDot(i,:));                                     
    gravity = p560.gravload(q(i,:));                                             
    torque(i,:) = (intertia*q2Dot(i,:)' + coriolis*qDot(i,:)' + gravity'+ J' * wieighing)';                     
    for joint = 1:6
        if abs(torque(i,joint)) > tauMax(joint)                                       
            torque(i,joint) = sign(torque(i,joint))*tauMax(joint);                          
        end
    end
          
    q2Dot(i,:) = (inv(intertia)*(torque(i,:)' - coriolis*qDot(i,:)' - gravity'- (J'*wieighing)))';            
    q(i+1,:) = q(i,:) + deltaT*qDot(i,:) + deltaT^2*q2Dot(i,:);                        
    qDot(i+1,:) = qDot(i,:) + deltaT*q2Dot(i,:);                                     
    p560.plot(q(i,:),'trail','r-')
end

%% Plotting

increment = 0:deltaT:(steps - 1) * deltaT;   
% Joint Angle
figure
for joint = 1:p560.n
    subplot(3,2,joint)
    plot(increment,q(:,joint)','k','LineWidth',1);
    refline(0,p560.qlim(joint,1));
    refline(0,p560.qlim(joint,2));
    ylabel('Angle [rad]');
end

% Joint Velocity
figure
for joint = 1:p560.n
    subplot(3,2,joint)
    plot(increment,qDot(:,joint),'k','LineWidth',1);
    refline(0,0);
    ylabel('V [RPM]');
end

% Joint Acceleration
figure
for joint = 1:p560.n
    subplot(3,2,joint)
    plot(increment,q2Dot(:,joint),'k','LineWidth',1);
    ylabel('Accel [rad/s^2]');
    refline(0,0)
end

% Torque: To do