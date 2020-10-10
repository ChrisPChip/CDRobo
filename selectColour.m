function [r, g, b] = selectColour(colour)
%% 4 Settings available
    % Red = 1
    % Green = 2
    % Blue = 3
    % Default = 4 (Doesn't matter for input value)
    
    if colour == 1
        r = [0 2 1.9];
        g = [0.3 2 1];
        b = [-0.3 2 1];
    elseif colour == 2
        r = [-0.3 2 1];
        g = [0 2 1.9];
        b = [0.3 2 1];
    elseif colour == 3
        r = [-0.3 2 1];
        g = [0.3 2 1];
        b = [0 2 1.9];
    else
        disp ('Incorrect Input, placing all cans in middle shelf')
        r = [-0.3 2 1];
        g = [0 2 1];
        b = [0.3 2 1];
    end
    
end
