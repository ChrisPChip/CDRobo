function moveRobot(robot, qMatrix, steps, grip, can)
if grip
    for i = 1:steps
        robot.model.animate(qMatrix(i,:));
        drawnow();
    end
    
else
    for i = 1:steps
        robot.model.animate(qMatrix(i,:));
        drawnow();
    end
end
end