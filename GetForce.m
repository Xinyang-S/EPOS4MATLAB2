function [force] = GetForce()
force = textread('D:\IC-Msc\Project\EPOS4MATLAB2\Force.txt','%f');
while (isempty(force))
    force = textread('D:\IC-Msc\Project\EPOS4MATLAB2\Force.txt','%f');
end
end
