function uniform_data = uniform_data_trials(data_input)
    switch length(num2str(data_input))
        case 1
            uniform_data = strcat('0','0',num2str(data_input));
        case 2
            uniform_data = strcat('0',num2str(data_input));
        case 3
            uniform_data = num2str(data_input);
    end
end