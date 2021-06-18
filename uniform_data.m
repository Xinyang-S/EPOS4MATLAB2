function uniform_data = uniform_data(data_input)
    switch length(num2str(data_input))
        case '1'
            uniform_data = strcat('0','0','0','0','0','0','0',num2str(data_input));
        case '2'
            uniform_data = strcat('0','0','0','0','0','0',num2str(data_input));
        case '3'
            uniform_data = strcat('0','0','0','0','0',num2str(data_input));
        case '4'
            uniform_data = strcat('0','0','0','0',num2str(data_input));
        case '5'
            uniform_data = strcat('0','0','0',num2str(data_input));
        case '6'
            uniform_data = strcat('0','0',num2str(data_input));
        case '7'
            uniform_data = strcat('0',num2str(data_input));
        case '8'
            uniform_data = num2str(data_input);
    end
end