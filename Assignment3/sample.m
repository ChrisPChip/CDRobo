clear all;
close all;
clc;

%% Loading the Drum file

%Drum Transform
display('The Drum transform');
drumTr = transl(1.40, 0.975, 0)

[f,v,data] = plyread('Drum.ply','tri');
% Scale the colours to be 0-to-1 (they are originally 0-to-255
vertexColours = [data.vertex.red, data.vertex.green, databedworks.vertex.blue] / 255;

for xOffset = [drumTr(1,4)]
    for yOffset = [drumTr(2,4)]
        for zOffset = [drumTr(3,4)]
        % Then plot the trisurf with offset verticies
        trisurf(f,v(:,1)+ xOffset,v(:,2) + yOffset, v(:,3) + zOffset, v(:,3) ...
            ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
        end
    end
end

%% Call the Robot
workspace = [-1 2.5 0.5 3 0 3];                                             % Creating a workspace for the simulation
scale = 0.40;    

display('The base transform');
studentNumTr = transl(1.206, 1.899, 1)                                      % Setting base transform according to my student number

mdl_puma560                                                                 % Calling the robot
p560.base = studentNumTr;                                                   % Assigning the base location to the robot

q0 = [0,0.5236,-3.1416,0, 0.7854,0];                                        % Specifying initial joint angles
                                     
toolOffset = transl(0, 0, 0.200)*troty(pi/4) 
p560.tool = toolOffset 

p560.plot(q0, 'workspace', workspace, 'scale', scale);                      % plotting the robot, workspace and scale

% Difference between the base and drum trasnform
display('Transform between the base and drum ');
diffTr = inv(studentNumTr) * drumTr

%% \
display('Tool offset as a homogenous matrix ');
toolOffset = transl(0, 0, 0.200)*troty(pi/4)        % Creating a homogenous transform for the tool offset
p560.tool = toolOffset                              % Assigning the tooloffset matrix to the robot
steps = 50;

% Starting corner          
startTr = transl(1.792, 1.5,0.5976)*troty(-pi);     % Creating a transform of where the starting corner is  
q1 = p560.ikunc(startTr,q0);                        % using Inverse Kinematics (Ikunc function) to find the required joint states to teach the startTr
qMatrix1 = jtraj(q0,q1,steps);                      % Using Quintic Polynomia and jtraj to compute the tranjectory between the two joint states
p560.plot(qMatrix1)                                 % Animating the motion for the trajectory

%%


%%%%%%%%%% Variables to change %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
time = 0.1;                                                                 % Total time to execute the motion
T2 = transl(1.792, 1.624, 0.5976)*troty(-pi);                               % Homegenous matrix of the second Cornor
q2 = p560.ikunc(T2,q0);                                                     % Inverse kinematics for 2nd pose 
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tau_max = [97.6 186.4 89.4 24.2 20.1 21.3]';                                % Maximum joint torque of the Puma560
dt = 1/100;                                                                 % Set control frequency at 100Hz
steps = time/dt;                                                            % No. of steps along trajectory

s = lspb(0,1,steps);                                                        % Generate trapezoidal velocity profile
for i = 1:steps
    q(i,:) = (1-s(i))*q1 + s(i)*q2;
end
% q = jtraj(q1,q2,steps);                                                   % Quintic polynomial profile

qd = zeros(steps,6);                                                        % Array of joint velocities
qdd = nan(steps,6);                                                         % Array of joint accelerations
tau = nan(steps,6);                                                         % Array of joint torques
mass = 2.15;                                                                % Payload mass (kg)
p560.payload(mass,[0;0;0]);                                                 % Set payload mass in Puma 560 model: offset 0.1m in x-direction
J = p560.jacob0(zeros(1,6));
w = [0 0 -215 0 0 0]';
for i = 1:steps-1
    qdd(i,:) = (1/dt)^2 * (q(i+1,:) - q(i,:) - dt*qd(i,:));                 % Calculate joint acceleration to get to next set of joint angles
    M = p560.inertia(q(i,:));                                               % Calculate inertia matrix at this pose
    C = p560.coriolis(q(i,:),qd(i,:));                                      % Calculate coriolis matrix at this pose
    g = p560.gravload(q(i,:));                                              % Calculate gravity vector at this pose
    tau(i,:) = (M*qdd(i,:)' + C*qd(i,:)' + g'+ J'*w)';                      % Calculate the joint torque needed
    for j = 1:6
        if abs(tau(i,j)) > tau_max(j)                                       % Check if torque exceeds limits
            tau(i,j) = sign(tau(i,j))*tau_max(j);                           % Cap joint torque if above limits
        end
    end
          
    qdd(i,:) = (inv(M)*(tau(i,:)' - C*qd(i,:)' - g'- (J'*w)))';             % Re-calculate acceleration based on actual torque
    q(i+1,:) = q(i,:) + dt*qd(i,:) + dt^2*qdd(i,:);                         % Update joint angles based on actual acceleration
    qd(i+1,:) = qd(i,:) + dt*qdd(i,:);                                      % Update the velocity for the next pose
    p560.plot(q(i,:),'trail','r-')
end


t = 0:dt:(steps-1)*dt;   
% Plot joint angles
figure(2)
for j = 1:6
    subplot(3,2,j)
    plot(t,q(:,j)','k','LineWidth',1);
    refline(0,p560.qlim(j,1));
    refline(0,p560.qlim(j,2));
    ylabel('Angle (rad)');
    box off
end

% Plot joint velocities
figure(3)
for j = 1:6
    subplot(3,2,j)
    plot(t,qd(:,j)*30/pi,'k','LineWidth',1);
    refline(0,0);
    ylabel('Velocity (RPM)');
    box off
end

% Plot joint acceleration
figure(4)
for j = 1:6
    subplot(3,2,j)
    plot(t,qdd(:,j),'k','LineWidth',1);
    ylabel('rad/s/s');
    refline(0,0)
    box off
end

% Plot joint torques
figure(5)
for j = 1:6
    subplot(3,2,j)
    plot(t,tau(:,j),'k','LineWidth',1);
    refline(0,tau_max(j));
    refline(0,-tau_max(j));
    ylabel('Nm');
    box off
end
