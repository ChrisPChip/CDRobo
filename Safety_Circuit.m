classdef Safety_Circuit < handle
    
    properties (Access = public)
        %% Default values for the safety circuit
        % Overall State of the safety circuit
        Safety_State;
        
    end
    properties (Access = private)
        % ESTOP true indicates normally closed contact - false signal will
        % signify that it has been pressed
        estop_;
        % Reset true indicates a normally opened RESET pushbutton contact
        reset_;
        
    end
    
    %% Static Methods
    methods (Static)
        
        function startupSafety()
            reset_ = false;
            estop_ = false;
        end
        
        function setEstop()
            estop_ = false;
        end
        
        function setReset()
            reset_ = true;
            estop_ = true;
        end
        
        function [state] = checkState()
            if ~estop_
                reset_ = false;
                state = false;
            elseif ~estop_ && reset_
                state = false;
            elseif estop_ && reset_
                state = true;
            end
        end
        
    end
end