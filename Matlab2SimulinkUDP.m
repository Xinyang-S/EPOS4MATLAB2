%% Clear
clc; clear all; close all;

%%
u1 = udpport("LocalPort",2500); %increase by one if error
u2 = udpport("LocalPort",1500); %increase by one if error

open 'Matlab2SimulinkUDPSim'
set_param('Matlab2SimulinkUDPSim','SimulationCommand','start')
read(u2,999,"double");  % ensure buffer is multiple of number of electrodes used 
c = clock;
clockNew = c(4)*3600+c(5)*60+c(6); 
clockPrev = clockNew;
while(clockNew < clockPrev + 5)
  c = clock;
  clockNew = c(4)*3600+c(5)*60+c(6); 
  disp(read(u2,999,"double"));  % ensure buffer is multiple of number of electrodes used 
  write(u1,clockNew-clockPrev,"double","LocalHost",5000);   
end

%%Save and Close
set_param('Matlab2SimulinkUDPSim','SimulationCommand','stop')
%save('openloopT3', 'controllerData')