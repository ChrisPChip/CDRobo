latchClass = Latches;
safetyClass = Safety_Circuit;
state = safetyClass.checkState;
% if latchClass.rLatch == false
%     disp('Latch not set');
% end
% 
% if safetyClass.System_Safety == false
%     disp('Safety not ready');
% end

while state
    disp ('System is ok')
end