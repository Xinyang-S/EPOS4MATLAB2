%%
%Emergency Stop - Run Section (may need to click Pause and Quit Debugging first)
delete(Motor1);
disp('estop')

%%
% Haptic wall (still a bit bouncy)
% Don't print anything continuously during runtime
clc
clear

%variables
hold = 10;
currentMaxP = 10000;
currentMaxN = -10000;
KW = 4; %8 //spring
DW = 10; %1.1 //damper
MW = 0; %0.001 //wall

%initialize
Motor1 = Epos4(0,0);
Motor1.ClearErrorState;
Motor1.DisableNode;
Motor1.SetOperationMode( OperationModes.CurrentMode );
Motor1.EnableNode;
Motor1.ClearErrorState;

c = clock;
clockStart = c(4)*3600+c(5)*60+c(6);
clockStartPrev = clockStart;
posClockPrev = clockStart;
startPosition = Motor1.ActualPosition;
posError = 0;
posErrorPrev = 0;
posErrorDiff = 0;
posErrorDiffPrev = 0;
posErrorDiffDifff = 0;
    
while (clockStart < clockStartPrev + hold)
    c = clock;
    clockStart = c(4)*3600+c(5)*60+c(6);
    
    %calculate distance into wall
    disp([Motor1.ActualPosition startPosition])
    posError = Motor1.ActualPosition - (startPosition+100);
   
    %calculate error velocity
    posErrorDiff = posError - posErrorPrev;
    posClockDiff = clockStart - posClockPrev;
   
    %if inside wall - current = KW*distance + DW*Velocity
    if (Motor1.ActualPosition > startPosition + 100)
        current = -1*(KW*posError + DW*posErrorDiff);        
        if (current > currentMaxP)
           current = currentMaxP;
           disp('Wall Too Strong!')
        elseif (current < currentMaxN)
           current = currentMaxN;
           disp('Wall Broke!')
        else
           Motor1.MotionWithCurrent(current);
        end
    else
        Motor1.MotionWithCurrent(0);
    end
    
    posErrorPrev = posError;
    posErrorDiffPrev = posErrorDiff;
    posClockPrev = clockStart;
end

delete(Motor1);
disp('Complete!')

%%
%% Open loop FES control - 
% Syncs with Simulink model calibrationSim
clc; clear all; close all;


%% Set up Bluetooth connection with Technalia FES device and Init UDP ports
bt = bluetooth("0016A474B78F", 1);  %MAC address of device 
writeline(bt,"iam DESKTOP");
writeline(bt,"battery ? ");
writeline(bt,"elec 1 *pads_qty 16");

elecname = "testname1"

% Init UDP ports for simulink comms 
% matlab and simulink need to be separate instances (same computer)
% u1 = udpport("LocalPort",12383) %increase by one if error
% u2 = udpport("LocalPort",22383) %increase by one if error
% 

% Write new stim file (last char of string not transmitted -> add space at end of string)
writeline(bt, "sdcard rm default/test/ve5.ptn ")
writeline(bt, "sdcard cat > default/test/ve5.ptn ")
writeline(bt, "sdcard ed default/test/ve5.ptn CONST CONST R 100 100 3000 ") % 
                                     % %pulsewith 
                             % time(us)(1ms -3000ms)
                      % %amplitude 
                      
%% test
% writeline(bt,strcat("freq ",num2str(200)));
% cmd = generate_command([15], [7], [300], elecname);
% writeline(bt,cmd)
% writeline(bt,strcat("stim ",elecname));
%% Select elecs
maxAmp = 12;
elecArray = selectElec(bt, maxAmp)
%% Set parameters/initialise model
buffer = 1000; % Buffer for?? 
% elecArray = [11, 15, 13]; % Electrode number for each finger 
elecArray = [9];   % Currently models and scripts set to only stim with one electrode!
h_mdl_struct = idnlhw([2 3 1], 'pwlinear', []); 
maxStimAmp = 9;
maxForce = 0.2; 

%% Identification of model for each electrode
h_mdls = calibration(elecArray, maxStimAmp, maxForce,h_mdl_struct, bt);

%% Stimulate 
clear u2
u2 = udpport("LocalPort",22392) %increase by one if error

velecnumber = 11;           % Choose velec that has not been defined
elecArray = [11, 16, 13];   % Electrode number for each finger (CURRENTLY ONLY MIDDLE ONE WILL BE STIMMED!) 
                            % do not select 2 bc it is the anode 
stimAmp = 8;%maxStimAmp; 


open 'openloopFEScontrollerSim'
set_param('openloopFEScontrollerSim','SimulationCommand','start')

writeline(bt,strcat("freq ",num2str(200)));      %Set stim frequency
cmd = generate_command(elecArray, [0 0 0], [300 300 300], elecname, velecnumber); % Params for start stimulation
writeline(bt,cmd)                               %Start stimulation
writeline(bt,strcat("stim on "));               %Start stimulation 

c = clock;
clockPrev = c(4)*3600+c(5)*60+c(6);
while true
    pwFES = read(u2,8190,"double");  % ensure buffer is multiple of number of electrodes used 
    c = clock;
    clockNew = c(4)*3600+c(5)*60+c(6); 
    if clockNew > clockPrev+0.02      %Send stim every 0.01s
        round(pwFES(end-2:end))
%         clock
        cmd = generate_command(elecArray, [0 0 0], round(pwFES(end-2:end)), elecname, velecnumber);
        writeline(bt,cmd)
        clockPrev = clockNew; 
    end
end
%%
% press CTRL+C to stop and send: writeline(bt,"stim off ")
% Stop stimulation with: set_param('openloopFEScontrollerSim','SimulationCommand','stop')
% Save recorded data with: save('openloop_0405_T1', 'controllerData')
writeline(bt,"stim off ")
set_param('openloopFEScontrollerSim','SimulationCommand','stop')

filename = sprintf('OpenLoop_%s', datestr(now,'mm-dd-yyyy HH-MM'));
save(filename, 'controllerData')
