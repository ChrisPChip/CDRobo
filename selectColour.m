function [r, g, b] = selectColour(str)
%% 4 Settings available
% Red = 1
% Green = 2
% Blue = 3
% Default = 4 (Doesn't matter for input value)



if strcmpi(str, 'Red')          % colour == 1
    r = [0 -1 2.042];
    g = [0 -1 1.77];
    b = [0 -1 1.503];
    
elseif strcmpi(str, 'Green')    % colour == 2
    g = [0 -1 2.042];
    b = [0 -1 1.77];
    r = [0 -1 1.503];
    
elseif  strcmpi(str, 'Blue')    % colour == 3
    b = [0 -1 2.042];
    g = [0 -1 1.77];
    r = [0 -1 1.503];
    
else
    disp ('Incorrect Input. Placing all cans on shelf 2.')
    r = [0-.25 -1 1.77];
    g = [0 -1 1.77];
    b = [0.25 -1 1.77];
end
end
