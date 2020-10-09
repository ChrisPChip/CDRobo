classdef Safety_Circuit < handle
    %% Public variables
    properties (GetAccess = 'private', SetAccess = 'private')
        
        % Overall State of the safety circuit
        safetyState; 
        
        % ESTOP true indicates normally closed contact - false signal will
        % signify that it has been pressed
        estop;
        
        % Reset true indicates a normally opened RESET pushbutton contact
        reset;
        
    end
    
    %% Methods
    methods
        
        % Constructor
        function obj = Safety_Circuit()
            obj.estop = false;
            obj.reset = false;
        end
        
        function setEstop(obj)
            obj.estop = false;
        end
        
        function setReset(obj)
            obj.reset = true;
            obj.estop = true;
        end
        
        function [safetyState] = checkState(obj, safetyState)
            if (~obj.estop)
                obj.safetyState = false;
            else
                obj.safetyState = true;
            end
        end
        
    end
end