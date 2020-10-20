close;
clear;
clc;

%% Can Starting Positions

% r = [-0.4 0.2 1.14];
% g = [-0.5 0 1.14];
% b = [-0.6 -0.2 1.14];

r = [-.9 0.2 1.14];
g = [-.9 0 1.14];
b = [-.9 -0.2 1.14];

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
% col = Collision;

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
%% Boxes
tableC = [0 0 0.55];
tableS = [2 1.5 1.1];

% Shelf1
shelf1C = [0 -.7 1.4];
shelf1S = [0.8 0.2 0.02];
% Shelf2
shelf2C = [0 -.7 1.4255];
shelf2S = [0.8 0.2 0.02];

% Shelf3
shelf3C = [0 -.7 1.6925];
shelf3S = [0.8 0.2 0.02];

% Shelf4
shelf4C = [0 -.7 1.9595];
shelf4S = [0.8 0.2 0.02];

% Shelf5
shelf5C = [0 -.7 2.2265];
shelf5S = [0.8 0.2 0.02];

plotOptions.plotFaces = true;
[vertex1,faces1,Normal1] = RectangularPrism(tableC + tableS/2, tableC - tableS/2,plotOptions);
[vertex2,faces2,Normal2] = RectangularPrism(shelf1C + shelf1S, shelf1C - shelf1S,plotOptions);
[vertex3,faces3,Normal3] = RectangularPrism(shelf2C + shelf2S, shelf2C - shelf2S,plotOptions);
[vertex4,faces4,Normal4] = RectangularPrism(shelf3C + shelf3S, shelf3C - shelf3S,plotOptions);
[vertex5,faces5,Normal5] = RectangularPrism(shelf4C + shelf4S, shelf4C - shelf4S,plotOptions);
[vertex6,faces6,Normal6] = RectangularPrism(shelf5C + shelf4S, shelf5C - shelf5S,plotOptions);
axis equal

%% Animation
r = [-.9 0.2 1.14];
g = [-.9 0 1.14];
b = [-.9 -0.2 1.14];

q0 = [0 0 0 0 0 0];
%ce
Roll = [0, .53, -67.836, -.032, -67.836, 81.57, -67.836 ];
Pitch = [0, -.96, 5.01, -75.57, 5.01, -78.49, 5.01];
Yaw = [0, 2.45, -97.55, -.0249, -97.55, 70.372, -97.55];

current = robot.model.fkine(q0);
robot.model.animate(q0)

positions = [current(1,4),current(2,4),current(3,4);
    r(1),r(2),r(3);       %Red Can
    redGoalCo(1),redGoalCo(2),redGoalCo(3);      %Final Red Can
    g(1),g(2),g(3);       %Green Can
    greenGoalCo(1),greenGoalCo(2),greenGoalCo(3);        %Final Green Can
    b(1),b(2),b(3);      %Blue Can
    blueGoalCo(1),blueGoalCo(2),blueGoalCo(3)];      %Final Blue Can

for k = 1:6
    q0 = robot.model.getpos;
    q1 = robot.model.ikcon(transl(positions(k+1,:)),q0);
    x = positions(k,1);
    y = positions(k,2);
    z = positions(k,3);
    fx = (positions(k+1,1)-positions(k,1))/10;
    fy = (positions(k+1,2)-positions(k,2))/10;
    fz = (positions(k+1,3)-positions(k,3))/10;
    qWaypoints = [q0 ; robot.model.ikcon(transl(x,y,z),q0)];
    [~,all] = robot.model.fkine(qWaypoints(end,:));
    for j = 1:6
        for i = 1 : size(all,3)-1
            for faceIndex1 = 1:size(faces1,1)
                futureWaypoints = [qWaypoints ; robot.model.ikcon(transl(x+fx,y+fy,z+fz),qWaypoints(end,:))];
                [~,newall] = robot.model.fkine(futureWaypoints(end,:));
                vertOnPlane1 = vertex1(faces1(faceIndex1,1)',:);
                vertOnPlane2 = vertex2(faces2(faceIndex1,1)',:);
                vertOnPlane3 = vertex3(faces3(faceIndex1,1)',:);
                vertOnPlane4 = vertex4(faces4(faceIndex1,1)',:);
                vertOnPlane5 = vertex5(faces5(faceIndex1,1)',:);
                vertOnPlane6 = vertex6(faces6(faceIndex1,1)',:);
                [intersects1,check1]=LinePlaneIntersection(Normal1(faceIndex1,:),vertOnPlane1,newall(1:3,4,i)',newall(1:3,4,i+1)');
                [intersects2,check2]=LinePlaneIntersection(Normal2(faceIndex1,:),vertOnPlane2,newall(1:3,4,i)',newall(1:3,4,i+1)');
                [intersects3,check3]=LinePlaneIntersection(Normal3(faceIndex1,:),vertOnPlane3,newall(1:3,4,i)',newall(1:3,4,i+1)');
                [intersects4,check4]=LinePlaneIntersection(Normal4(faceIndex1,:),vertOnPlane4,newall(1:3,4,i)',newall(1:3,4,i+1)');
                [intersects5,check5]=LinePlaneIntersection(Normal5(faceIndex1,:),vertOnPlane5,newall(1:3,4,i)',newall(1:3,4,i+1)');
                [intersects6,check6]=LinePlaneIntersection(Normal6(faceIndex1,:),vertOnPlane6,newall(1:3,4,i)',newall(1:3,4,i+1)');
                
                if (check1 == 1 && IsIntersectionPointInsideTriangle(intersects1,vertex1(faces1(faceIndex1,:)',:))) || (check2 == 1 && IsIntersectionPointInsideTriangle(intersects2,vertex2(faces2(faceIndex1,:)',:))) || (check3 == 1 && IsIntersectionPointInsideTriangle(intersects3,vertex3(faces3(faceIndex1,:)',:)))...
                        || (check4 == 1 && IsIntersectionPointInsideTriangle(intersects4,vertex4(faces4(faceIndex1,:)',:)))|| (check5 == 1 && IsIntersectionPointInsideTriangle(intersects5,vertex5(faces5(faceIndex1,:)',:)))|| (check6 == 1 && IsIntersectionPointInsideTriangle(intersects6,vertex6(faces6(faceIndex1,:)',:)))
                    disp('Avoiding Collision')
                    x = x-.2;
                    y = y+.2;
                    z = z+.2;
                end
            end
        end
        qWaypoints = [qWaypoints; robot.model.ikcon(transl(x,y,z),qWaypoints(end,:))];
        x=x+fx;
        y=y+fy;
        z =z+fz;
        
    end
    qWaypoints = [qWaypoints; robot.model.ikcon(transl(x,y,z),q1)];
    
    % RMRC
    % steps = size(qWaypoints,1);
    steps = 40;
    deltaT = 0.05;
    x0 = zeros(3,steps);
    s = lspb(0,1,steps);
    delta = 2*pi/steps;
    epsilon = 0.1;
    W = diag([1 1 1 0.1 0.1 0.1]);
    m = zeros(steps,1);
    qdot = zeros(steps,6);
    theta = zeros(3,steps);
    positionError = zeros(3,steps);
    angleError = zeros(3,steps);
    x1 = robot.model.fkine(q0);
    x2 = robot.model.fkine(qWaypoints(end,:));
    for i = 1:steps
        x0(:,i) = x1(1:3,4)*(1-s(i)) + s(i)*x2(1:3,4);
        theta(1,i) = Roll(k);
        theta(2,i) = Pitch(k);
        theta(3,i) = Yaw(k);
    end
    
    for i = 1:steps-1
        T = robot.model.fkine(real(qWaypoints(i,:)));
        deltaX = x0(:,i+1) - T(1:3,4);
        Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));
        Ra = T(1:3,1:3);
        Rdot = (1/deltaT)*(Rd - Ra);
        S = Rdot*Ra';
        linear_velocity = (1/deltaT)*deltaX;
        angular_velocity = [S(3,2);S(1,3);S(2,1)];
        xdot = W*[linear_velocity;angular_velocity];
        J = robot.model.jacob0(real(qWaypoints(i,:)));
        m(i) = sqrt(det(J*J'));
        if m(i) < epsilon  % If manipulability is less than given threshold
            lambda = (1 - m(i)/epsilon)*5E-2;
        else
            lambda = 0;
        end
        invJ = inv(J'*J + lambda *eye(6))*J';
        qdot(i,:) = (invJ*xdot)';
        for j = 1:6
            if qWaypoints(i,j) + deltaT*qdot(i,j) < robot.model.qlim(j,1)
                qdot(i,j) = 0; % Stop the motor
            elseif qWaypoints(i,j) + deltaT*qdot(i,j) > robot.model.qlim(j,2)
                qdot(i,j) = 0; % Stop the motor
            end
        end
        qWaypoints(i+1,:) = real(qWaypoints(i,:)) + deltaT*real(qdot(i,:));
        positionError(:,i) = x0(:,i+1) - T(1:3,4);
    end
    for anim=1:steps
        robot.model.animate(qWaypoints(anim,:));
        if k == 2
            redStart = robot.model.fkine(qWaypoints(anim,:));
            delete(redCanLocation);
            redCanLocation = redCan(redStart*troty(pi/2));
        elseif k == 4
            greenStart = robot.model.fkine(qWaypoints(anim,:));
            delete(greenCanLocation);
            greenCanLocation = greenCan(greenStart*troty(pi/2));
        elseif k == 6
            blueStart = robot.model.fkine(qWaypoints(anim,:));
            delete(blueCanLocation);
            blueCanLocation = blueCan(blueStart*troty(pi/2));
        end
        drawnow();
    end
    [~,all] = robot.model.fkine(q1);
    for i = 1 : size(all,3)-1
        for faceIndex1 = 1:size(faces1,1)
            vertOnPlane1 = vertex1(faces1(faceIndex1,1)',:);
            vertOnPlane2 = vertex2(faces2(faceIndex1,1)',:);
            vertOnPlane3 = vertex3(faces3(faceIndex1,1)',:);
            vertOnPlane4 = vertex4(faces4(faceIndex1,1)',:);
            vertOnPlane5 = vertex5(faces5(faceIndex1,1)',:);
            vertOnPlane6 = vertex6(faces6(faceIndex1,1)',:);
            [intersects1,check1]=LinePlaneIntersection(Normal1(faceIndex1,:),vertOnPlane1,newall(1:3,4,i)',newall(1:3,4,i+1)');
            [intersects2,check2]=LinePlaneIntersection(Normal2(faceIndex1,:),vertOnPlane2,newall(1:3,4,i)',newall(1:3,4,i+1)');
            [intersects3,check3]=LinePlaneIntersection(Normal3(faceIndex1,:),vertOnPlane3,newall(1:3,4,i)',newall(1:3,4,i+1)');
            [intersects4,check4]=LinePlaneIntersection(Normal4(faceIndex1,:),vertOnPlane4,newall(1:3,4,i)',newall(1:3,4,i+1)');
            [intersects5,check5]=LinePlaneIntersection(Normal5(faceIndex1,:),vertOnPlane5,newall(1:3,4,i)',newall(1:3,4,i+1)');
            [intersects6,check6]=LinePlaneIntersection(Normal6(faceIndex1,:),vertOnPlane6,newall(1:3,4,i)',newall(1:3,4,i+1)');
            
            if (check1 == 1 && IsIntersectionPointInsideTriangle(intersects1,vertex1(faces1(faceIndex1,:)',:))) || (check2 == 1 && IsIntersectionPointInsideTriangle(intersects2,vertex2(faces2(faceIndex1,:)',:))) || (check3 == 1 && IsIntersectionPointInsideTriangle(intersects3,vertex3(faces3(faceIndex1,:)',:)))...
                    || (check4 == 1 && IsIntersectionPointInsideTriangle(intersects4,vertex4(faces4(faceIndex1,:)',:)))|| (check5 == 1 && IsIntersectionPointInsideTriangle(intersects5,vertex5(faces5(faceIndex1,:)',:)))|| (check6 == 1 && IsIntersectionPointInsideTriangle(intersects6,vertex6(faces6(faceIndex1,:)',:)))
                disp('Cannot get to final destination, please clear the area')
                %uiwait
            end
        end
    end
    disp('Moving to final');
    qWaypoints = [qWaypoints(end,:); q1];
    
    %  RMRC To final Position
    
    q0 = robot.model.getpos;
    x1 = robot.model.fkine(q0);
    x2 = robot.model.fkine(q1);
    for i = 1:steps
        x0(:,i) = x1(1:3,4)*(1-s(i)) + s(i)*x2(1:3,4);
        theta(1,i) = Roll(k+1);
        theta(2,i) = Pitch(k+1);
        theta(3,i) = Yaw(k+1);
    end
    
    for i = 1:steps-1
        T = robot.model.fkine(real(qWaypoints(i,:)));
        deltaX = x0(:,i+1) - T(1:3,4);
        Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));
        Ra = T(1:3,1:3);
        Rdot = (1/deltaT)*(Rd - Ra);
        S = Rdot*Ra';
        linear_velocity = (1/deltaT)*deltaX;
        angular_velocity = [S(3,2);S(1,3);S(2,1)];
        xdot = W*[linear_velocity;angular_velocity];
        J = robot.model.jacob0(real(qWaypoints(i,:)));
        m(i) = sqrt(det(J*J'));
        if m(i) < epsilon
            lambda = (1 - m(i)/epsilon)*5E-2;
        else
            lambda = 0;
        end
        invJ = inv(J'*J + lambda *eye(6))*J';
        qdot(i,:) = (invJ*xdot)';
        for j = 1:6
            if qWaypoints(i,j) + deltaT*qdot(i,j) < robot.model.qlim(j,1)
                qdot(i,j) = 0; % Stop the motor
            elseif qWaypoints(i,j) + deltaT*qdot(i,j) > robot.model.qlim(j,2)
                qdot(i,j) = 0; % Stop the motor
            end
        end
        qWaypoints(i+1,:) = real(qWaypoints(i,:)) + deltaT*real(qdot(i,:));
        positionError(:,i) = x0(:,i+1) - T(1:3,4);
    end
    for anim = 1:steps
        robot.model.animate(qWaypoints(anim,:));
        if k == 2
            redStart = robot.model.fkine(qWaypoints(anim,:));
            delete(redCanLocation);
            redCanLocation = redCan(redStart*troty(pi/2));
        elseif k == 4
            greenStart = robot.model.fkine(qWaypoints(anim,:));
            delete(greenCanLocation);
            greenCanLocation = greenCan(greenStart*troty(pi/2));
        elseif k == 6
            blueStart = robot.model.fkine(qWaypoints(anim,:));
            delete(blueCanLocation);
            blueCanLocation = blueCan(blueStart*troty(pi/2));
        end
        drawnow();
    end
end

%% Retreat from symbol

pStar = [ 662 362 362 662; 362 362 662 662];


P=[-1.2,-1.2,-1.2,-1.2;
    -0.25,0.25,0.25,-0.25;
    2,2,1.5,1.5];

q0 = [0; 0; pi/2; pi/10; 0; pi/4];

cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
    'resolution', [1024 1024], 'centre', [512 512],'name','kinova');

fps = 25;
lambda = 0.6;
depth = mean (P(1,:));
Tc0= robot.model.fkine(q0);
robot.model.animate(q0');
drawnow
cam.T = Tc0;
% cam.plot_camera('Tcam',Tc0, 'label','scale',0.05);
% plot_sphere(P, 0.05, 'b')
lighting gouraud
light
p = cam.plot(P, 'Tcam', Tc0);
cam.clf()
cam.plot(pStar, '*');
cam.hold(true);
cam.plot(P, 'Tcam', Tc0, 'o');
pause(2)
cam.hold(true);
cam.plot(P);
vel_p = [];
uv_p = [];
history = [];
ksteps = 0;
for i=1:10
    ksteps = ksteps + 1;
    
    % compute the view of the camera
    uv = cam.plot(P);
    
    % compute image plane error as a column
    e = pStar-uv;   % feature error
    e = e(:);
    Zest = [];
    
    % compute the Jacobian
    if isempty(depth)
        % exact depth from simulation
        pt = homtrans(inv(Tcam), P);
        J = cam.visjac_p(uv, pt(3,:) );
    elseif ~isempty(Zest)
        J = cam.visjac_p(uv, Zest);
    else
        J = cam.visjac_p(uv, depth );
    end
    
    % compute the velocity of camera in camera frame
    try
        v = lambda * pinv(J) * e;
    catch
        status = -1;
        return
    end
    fprintf('v: %.3f %.3f %.3f %.3f %.3f %.3f\n', v);
    
    %compute robot's Jacobian and inverse
    J2 = robot.model.jacobn(q0);
    Jinv = pinv(J2);
    % get joint velocities
    qp = Jinv*v;
    
    
    %Maximum angular velocity cannot exceed 180 degrees/s
    ind=find(qp>pi);
    if ~isempty(ind)
        qp(ind)=pi;
    end
    ind=find(qp<-pi);
    if ~isempty(ind)
        qp(ind)=-pi;
    end
    
    %Update joints
    q = q0 - (1/fps)*qp;
    deltaT = 0.05;
    s = lspb(0,1,steps);
    qMatrix = nan(steps,6);
    for wtg = 1:steps
        qMatrix(i,:) = (1-s(wtg))*q0+s(wtg)*qp;
    end
    robot.model.animate(q');
    
    %Get camera location
    Tc = robot.model.fkine(q);
    cam.T = Tc;
    
    drawnow
    
    % update the history variables
    hist.uv = uv(:);
    vel = v;
    hist.vel = vel;
    hist.e = e;
    hist.en = norm(e);
    hist.jcond = cond(J);
    hist.Tcam = Tc;
    hist.vel_p = vel;
    hist.uv_p = uv;
    hist.qp = qp;
    hist.q = q;
    
    history = [history hist];
    
    pause(1/fps)
    
    if ~isempty(200) && (ksteps > 200)
        break;
    end
    
    %update current joint position
    q0 = q;
end
%% Trajectory with jtraj
% q1 = robot.model.ikcon(redPos);
% qMatrix = jtraj(q0, q1, steps);
%
% for i = 1:steps
%     robot.model.animate(qMatrix(i,:));
%     drawnow();
% end
%
% q2 = robot.model.ikcon(redGoalPose);
% qMatrix = jtraj(q1, q2, steps);
%
% for i = 1:steps
%     robot.model.animate(qMatrix(i,:));
%     redStart = robot.model.fkine(qMatrix(i,:));
%     delete(redCanLocation);
%     redCanLocation = redCan(redStart*troty(pi/2));
%     drawnow();
% end
%
% qMatrix = jtraj(q2,q0,steps);
%
% for i = 1:steps
%     robot.model.animate(qMatrix(i,:));
%     drawnow();
% end
%
% % Green Can Sequence
%
% q3 = robot.model.ikcon(greenPos);
% qMatrix = jtraj(q0, q3, steps);
%
% for i = 1:steps
%     robot.model.animate(qMatrix(i,:));
%     drawnow();
% end
%
% q4 = robot.model.ikcon(greenGoalPose);
% qMatrix = jtraj(q3, q4, steps);
%
% for i = 1:steps
%     robot.model.animate(qMatrix(i,:));
%     greenStart = robot.model.fkine(qMatrix(i,:));
%     delete(greenCanLocation);
%     greenCanLocation = greenCan(greenStart*troty(pi/2));
%     drawnow();
% end
%
% qMatrix = jtraj(q4,q0,steps);
%
% for i = 1:steps
%     robot.model.animate(qMatrix(i,:));
%     drawnow();
% end
%
% % Blue Can Sequence
%
% q5 = robot.model.ikcon(bluePos);
% qMatrix = jtraj(q0,q5,steps);
%
% for i = 1:steps
%     robot.model.animate(qMatrix(i,:));
%     drawnow();
% end
%
% q6 = robot.model.ikcon(blueGoalPose);
% qMatrix = jtraj(q5, q6, steps);
%
% for i = 1:steps
%     robot.model.animate(qMatrix(i,:));
%     blueStart = robot.model.fkine(qMatrix(i,:));
%     delete(blueCanLocation);
%     blueCanLocation = blueCan(blueStart*troty(pi/2));
%     drawnow();
% end
%
% qMatrix = jtraj(q6,q0,steps);
%
% for i = 1:steps
%     robot.model.animate(qMatrix(i,:));
%     drawnow();
% end
