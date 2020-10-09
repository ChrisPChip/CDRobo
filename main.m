clc;
clear;

% safetyClass = Safety_Circuit;
% safetyState = safetyClass.checkState;

environment = Environment;
environment.generateObjects(environment);
hold on;
robot = KinovaGen3(false);

% while startPB == 1
%     disp ('System is ok')
% end