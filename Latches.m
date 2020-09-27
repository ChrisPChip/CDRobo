classdef Latches < handle
    
    properties
        rLatch;
    end
    
    methods (Static)
        
        function startupLatches
            rLatch = false;
        end
        
        
        function rLatch = setRunLatch()
             rLatch = true;
             while rLatch == true
                for i = 1:20
                    disp(i);
                end
             end
        end
    end
end