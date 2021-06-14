function [Force] = GetForce()
force = textread('D:\IC-Msc\Project\EPOS4MATLAB2\Force.txt','%f');
while isempty(force)
    pause(0.01)
    force = textread('D:\IC-Msc\Project\EPOS4MATLAB2\Force.txt','%f');
end
Force = force;
end
