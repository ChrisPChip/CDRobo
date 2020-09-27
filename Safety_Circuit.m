classdef Safety_Circuit < handle
    
    properties
        % Default values for the safety circuit
        % ESTOP true indicates normally closed contact - false signal will
        % signify that it has been pressed
        % Reset true indicates a normally opened RESET pushbutton contact
        System_Estop;
        System_Reset;
        System_Safety;
    end
    
    %% Static Methods
    methods (Static)
        
        function startupSafety()
            setEstopState = false;
            setResetState = false;
        end
        
        function checkSafetyStatus(System_Estop, System_Reset)
            while (System_Estop == true && System_Reset == false)
                if System_Reset == true
                    setEstopState(true);
                    setResetState(true);
                    System_Safety = true;
                else
                    System_Safety = false;
                end
            end
            
            while (System_Estop == false && System_Reset == false)
                if System_Reset == true
                    setEstopState(true);
                    setResetState(true);
                    System_Safety = true;
                else
                    System_Safety = false;
                end
            end
        end
        
        %% Non Static Methods
        
        function setEstopState(state)
            if state == true
                System_Estop = true;
            elseif state == false
                System_Estop = false;
            end
        end
        
        function setResetState(state)
            if state == true
                System_Reset = true;
            elseif state == false
                System_Reset = false;
            end
        end
    end
end
