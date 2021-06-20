function current_output = safetyCheck(current_input)
    if abs(current_input) >= 7000
        disp('current too high!')
        current_output = 0;
    else
        current_output = current_input;
    end
end