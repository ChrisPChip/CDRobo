classdef Latches < handle
    
    properties (Access = public)
        runLatch;
    end
    
    methods (Static)
        
        function startupLatches
            runLatch = false;
        end
        
        
        function runLatch = setRunLatch()
            runLatch = true;
            while runLatch == true
                for i = 1:20
                    disp(i);
                end
            end
        end
    end
end