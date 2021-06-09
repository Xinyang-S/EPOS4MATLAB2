function res = SetAnalog(handle, nodeid, inputPin, value)
% This function is for setting the analog input voltage (decimal value in Volts, max 4V) of an analog pin of the EPOS4 controller
%
% use it as:
%
% >> Motor1.SetWithAnalog(inputPin, value)
%
% A. Yurkewich, 2020
%