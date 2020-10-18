%% GetRobot
workspace = [-2 2 -1 4 0 3];
name = ('kinova');

L1 = Link('d',0.2848,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
L2 = Link('d',0.0054,'a',0.41,'alpha',-pi,'offset',-pi/2,'qlim',[deg2rad(-90),deg2rad(90)]);
L3 = Link('d',0.0064,'a',0,'alpha',-pi/2,'offset',-pi/2,'qlim',[deg2rad(-147.8),deg2rad(147.8)]);
L4 = Link('d',0.2084+.1059,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
L5 = Link('d',0,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-120.3),deg2rad(120.3)]);
L6 = Link('d',0.1059+.06153,'a',0,'alpha',0,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);

kinova = SerialLink([L1 L2 L3 L4 L5 L6],'name',name);
kinova.base = transl(0,3,1);
q0 = [0 0 0 0 0 0];

kinova.plot(q0,'workspace',workspace,'scale',.5);

%% Determine if someone goes through red line

centerpnt = [-1,3,1];
side = 1;
plotOptions.plotFaces = true;
[vertex,faces,Normal] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);
axis equal
Start = [1.6 0 .5];
End = [2.6 0 .5];
plot3([Start(1),End(1)],[Start(2),End(2)],[Start(3),End(3)],'r');
for faceIndex = 1:size(faces,1)
    vertOnPlane = vertex(faces(faceIndex,1)',:);
    [intersects,check]=LinePlaneIntersection(Normal(faceIndex,:),vertOnPlane,Start,End);
    if check == 1 && IsIntersectionPointInsideTriangle(intersects,vertex(faces(faceIndex,:)',:))
        disp('Unauthorized person on the area')
        plot3(intersects(1),intersects(2),intersects(3),'g*');
    end
end

%% Active Collision avoidance
positions = [0,3,2.17;
    -1,3,1.08;       %Red Can
    0.0,2,1.9;      %Final Red Can
    -.4,3.5,1.08;       %Blue Can
    .25,2,1;        %Final Blue Can
    0.4,3.3,1.08;      %Green Can
    .3,2,1];      %Final Green Can

%q1 = [-0.0015 -1.5708 -0.5828 -3.1413 1.9777 3.1429];

for k = 1:6
    q0 = kinova.getpos;
    q1 = kinova.ikcon(transl(positions(k+1,:)),q0);
    x = positions(k,1);
    y = positions(k,2);
    z = positions(k,3);
    fx = (positions(k+1,1)-positions(k,1))/10;
    fy = (positions(k+1,2)-positions(k,2))/10;
    fz = (positions(k+1,3)-positions(k,3))/10;
    qWaypoints = [q0 ; kinova.ikcon(transl(x,y,z),q0)];
    [~,all] = kinova.fkine(qWaypoints(end,:));
    for j = 1:6
        for i = 1 : size(all,3)-1
            for faceIndex = 1:size(faces,1)
                futureWaypoints = [qWaypoints ; kinova.ikcon(transl(x+fx,y+fy,z+fz),qWaypoints(end,:))];
                [~,newall] = kinova.fkine(futureWaypoints(end,:));
                vertOnPlane = vertex(faces(faceIndex,1)',:);
                [intersects,check]=LinePlaneIntersection(Normal(faceIndex,:),vertOnPlane,newall(1:3,4,i)',newall(1:3,4,i+1)');
                if check == 1 && IsIntersectionPointInsideTriangle(intersects,vertex(faces(faceIndex,:)',:))
                    disp('Avoiding Collision')
                    x = x+.3;
                    y = y-.3;
                    z = z+.3;
                end
            end
        end
        qWaypoints = [qWaypoints; kinova.ikcon(transl(x,y,z),qWaypoints(end,:))];
        x=x+fx;
        y=y+fy;
        z =z+fz;
        
    end
    qWaypoints = [qWaypoints; kinova.ikcon(transl(x,y,z),q1)];
    qMrix = InterpolateWaypointRadians(qWaypoints,deg2rad(5));
    % RMRC
    % steps = size(qMrix,1);
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
    x1 = kinova.fkine(q0);
    x2 = kinova.fkine(qMrix(end,:));
    for i = 1:steps
        x0(:,i) = x1(1:3,4)*(1-s(i)) + s(i)*x2(1:3,4);
        theta(1,i) = 0;
        theta(2,i) = 5*pi/9;
        theta(3,i) = 0;
    end
    
    for i = 1:steps-1
        T = kinova.fkine(real(qMrix(i,:)));
        deltaX = x0(:,i+1) - T(1:3,4);
        Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));
        Ra = T(1:3,1:3);
        Rdot = (1/deltaT)*(Rd - Ra);
        S = Rdot*Ra';
        linear_velocity = (1/deltaT)*deltaX;
        angular_velocity = [S(3,2);S(1,3);S(2,1)];
        xdot = W*[linear_velocity;angular_velocity];
        J = kinova.jacob0(real(qMrix(i,:)));
        m(i) = sqrt(det(J*J'));
        if m(i) < epsilon  % If manipulability is less than given threshold
            lambda = (1 - m(i)/epsilon)*5E-2;
        else
            lambda = 0;
        end
        invJ = inv(J'*J + lambda *eye(6))*J';
        qdot(i,:) = (invJ*xdot)';
        for j = 1:6
            if qMrix(i,j) + deltaT*qdot(i,j) < kinova.qlim(j,1)
                qdot(i,j) = 0; % Stop the motor
            elseif qMrix(i,j) + deltaT*qdot(i,j) > kinova.qlim(j,2)
                qdot(i,j) = 0; % Stop the motor
            end
        end
        qMrix(i+1,:) = real(qMrix(i,:)) + deltaT*real(qdot(i,:));
        positionError(:,i) = x0(:,i+1) - T(1:3,4);
    end
    
    kinova.animate(qMrix);
    [~,all] = kinova.fkine(q1);
    for i = 1 : size(all,3)-1
        for faceIndex = 1:size(faces,1)
            vertOnPlane = vertex(faces(faceIndex,1)',:);
            [intersects,check]=LinePlaneIntersection(Normal(faceIndex,:),vertOnPlane,all(1:3,4,i)',all(1:3,4,i+1)');
            if check == 1 && IsIntersectionPointInsideTriangle(intersects,vertex(faces(faceIndex,:)',:))
                disp('Cannot get to final destination, please clear the area')
                %uiwait
            end
        end
    end
    qWaypoints = [qMrix(end,:); q1];
    qMatrix = InterpolateWaypointRadians(qWaypoints,deg2rad(5));
    
    %  RMRC To final Position
    q0 = kinova.getpos;
    x1 = kinova.fkine(q0);
    x2 = kinova.fkine(q1);
    for i = 1:steps
        x0(:,i) = x1(1:3,4)*(1-s(i)) + s(i)*x2(1:3,4);
        theta(1,i) = 0;
        theta(2,i) = 5*pi/9;
        theta(3,i) = 0;
    end
    
    for i = 1:steps-1
        T = kinova.fkine(real(qMatrix(i,:)));
        deltaX = x0(:,i+1) - T(1:3,4);
        Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));
        Ra = T(1:3,1:3);
        Rdot = (1/deltaT)*(Rd - Ra);
        S = Rdot*Ra';
        linear_velocity = (1/deltaT)*deltaX;
        angular_velocity = [S(3,2);S(1,3);S(2,1)];
        xdot = W*[linear_velocity;angular_velocity];
        J = kinova.jacob0(real(qMatrix(i,:)));
        m(i) = sqrt(det(J*J'));
        if m(i) < epsilon
            lambda = (1 - m(i)/epsilon)*5E-2;
        else
            lambda = 0;
        end
        invJ = inv(J'*J + lambda *eye(6))*J';
        qdot(i,:) = (invJ*xdot)';
        for j = 1:6
            if qMatrix(i,j) + deltaT*qdot(i,j) < kinova.qlim(j,1)
                qdot(i,j) = 0; % Stop the motor
            elseif qMatrix(i,j) + deltaT*qdot(i,j) > kinova.qlim(j,2)
                qdot(i,j) = 0; % Stop the motor
            end
        end
        qMatrix(i+1,:) = real(qMatrix(i,:)) + deltaT*real(qdot(i,:));
        positionError(:,i) = x0(:,i+1) - T(1:3,4);
    end
    kinova.animate(qMatrix);
end