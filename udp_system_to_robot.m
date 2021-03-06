%  UDP SYSTEM
% read udpport (no flush)
% write udp
clear ur;
%fclose(uw);
c=clock();
clockStart = c(4)*3600 + c(5)*60 + c(6);
freq_traj = 1;
port = 5566;
ur = udpport('LocalPort', port+2,'timeout',100);
uw = udp('LocalHost', port+1,'timeout',100);
fopen(uw);
c = clock(); %frequency of desired trajectory
amp_traj = 300; %amplitude of desired trajectory

while(1)
    tic
    %write data to robot's matlab (mode, current) - mode 0: do nothing, mode 1: set current 
    c = clock();
    clockNew = c(4)*3600 + c(5)*60 + c(6);
    current = amp_traj*sin((clockNew-clockStart)*freq_traj)
    current = safetyCheck(current);
    dataW =  typecast(single([1 current]), 'int8');
    fwrite(uw, dataW, 'int8');
    pause(0.001);
    toc
    
    %read data from robot matlab ( encoder )
    dataR = int8(read(ur, 4, 'int8'));
    dataR1 = typecast(dataR, 'single')
end