function [force] = GetForce()
force = textread('H:\GitHub\EPOS4MATLAB2\Force.txt','%f');
while (isempty(force))
    force = textread('H:\GitHub\EPOS4MATLAB2\Force.txt','%f');
end
end
