function current_output = safetyCheck(current_input)
    current_max = 7000;
    if current_input >= current_max
        disp('current too high!')
        current_output = current_max;
    elseif current_input <= -current_max
        disp('current too low!')
        current_output = -current_max;
    else
        current_output = current_input;
    end
end