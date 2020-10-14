function motionTestscript

close;
clear;
clc;

b = [0.6 3 (1.08 + 0.06)];
blueTransl = transl(b);

blueCanLocation = blueCan(blueTransl);

bluePos = transl(b)*troty(-pi/2);

robot = KinovaGen3;
hold on;
axis equal;

% Had to be rotated by troty(pi/2) as the endefector was going below the
% table to get to destination coordinates
% BlueCan = eye(4)*transl(blue(1), blue(2), blue(3))*troty(pi/2);
pause(3);

q0 = [0 0 0 0 0 0];
steps = 32;

q1 = robot.model.ikcon(bluePos,q0);
qMatrix = jtraj(q0,q1,steps);

for i = 1:steps
    robot.model.animate(qMatrix(i,:));
    drawnow();
end

q2 = [0 0 0 0 0 0];
qMatrix = jtraj(q1, q2, steps);

    for i = 1:steps
        robot.model.animate(qMatrix(i,:));
        blueTransl = robot.model.fkine(qMatrix(i,:));
        delete(blueCanLocation);
        blueCanLocation = blueCan(blueTransl*troty(pi/2));
        drawnow();
    end

end