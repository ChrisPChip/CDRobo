function [state] = controlFn(val)
    if val == 1
        state = true;
    else
        state = false;
    end
end