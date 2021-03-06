%%
%Emergency Stop - Run Section (may need to click Pause and Quit Debugging first)
delete(Motor1);
disp('estop')

%%
% Haptic wall (still a bit bouncy)
% Don't print anything continuously during runtime
clc
clear

%FES Setup
% Set up Bluetooth connection with Technalia FES device
clear bt
bt = bluetooth("0016A474B78F", 1);  %MAC address of device 
writeline(bt,"iam DESKTOP");
writeline(bt,"battery ? ");
writeline(bt,"elec 1 *pads_qty 16");

elecname = "testname1";

% Write new stim file (last char of string not transmitted -> add space at end of string)
writeline(bt, "sdcard rm default/test/ve5.ptn ")
writeline(bt, "sdcard cat > default/test/ve5.ptn ")
writeline(bt, "sdcard ed default/test/ve5.ptn CONST CONST R 100 100 3000 ") % PW Time Amp

% elecArray = [9];  %modify per person using fesapp % Currently models and scripts set to only stim with one electrode!
extTurn = 0;
elecArrayFlex = [9];
maxStimAmpFlex = 10; %modify per person using fesapp - 1mA below discomfort at 25Hz, 300us PW 
minStimAmpFlex = 3; %modify per person using fesapp - 1mA below feeling
elecArrayExt = [3];
maxStimAmpExt = 10; %modify per person using fesapp - 1mA below discomfort  at 25Hz, 300us PW
minStimAmpExt = 3; %modify per person using fesapp - 1mA below feeling
maxDisp = 40 * 6400 / 90; %max displacement is +-45 degrees * 6400/90 - converted to encoder units
KW_Flex = (maxStimAmpFlex - minStimAmpFlex) / maxDisp;
KW_Ext = (maxStimAmpExt - minStimAmpExt) / maxDisp;

writeline(bt,strcat("freq ",num2str(25), " "));      %Set stim frequency
cmd = generate_command(elecArrayFlex, [0], [300], elecname, velecnumber); % Params for start stimulation - array, amp, PW
writeline(bt,cmd)                               %Start stimulation
writeline(bt,strcat("stim on "));               %Start stimulation 

writeline(bt,strcat("freq ",num2str(25), " "));      %Set stim frequency
cmd = generate_command(elecArrayExt, [0], [300], elecname, velecnumber); % Params for start stimulation - array, amp, PW
writeline(bt,cmd)                               %Start stimulation
writeline(bt,strcat("stim on "));               %Start stimulation 

%Robot Variables
hold = 10;
currentMaxP = 10000;
currentMaxN = -10000;
%Stiffness Equation I = K * R^2 * error_deg * 2 * PI / ( 360 * torqueConst ) % m*(N/m)*m*A/Nm = A
% I = (posError * 90/6400) * 1000mA * 120 * 0.1^2 * 2 * pi / ( 360 * 0.136 )
KW = 2.17; %spring: 1 light, 2 med (best demo), 3 large, 4 heavy (limit), 2.17 = Hi5 papers
DW = 0; %damper: 2 works for spring 1-4, larger for 5+
filterLP_D = 0.7; %filter error velocity: 0.7, not very sensitive
MW = 0; %0.001 //wall
freq_traj = 1; %frequency of desired trajectory
amp_traj = 3000; %amplitude of desired trajectory

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
clockPrevFes = clockStart;
posClockPrev = clockStart;
clockPrev = clockStart;
clockBegin = clockStart;
startPosition = Motor1.ActualPosition;
posError = 0;
posErrorPrev = 0;
posErrorDiff = 0;
posErrorDiffPrev = 0;
posErrorDiffDifff = 0;
current = 0;
currentNew =0;
posErrorDiffNew =0;
loopCounter=0;
data = zeros(1,4);
currentPosition=0;
    
while (clockStart < clockStartPrev + hold)
    c = clock;
    clockStart = c(4)*3600+c(5)*60+c(6);
    
    %desired trajectory 
    startPositionNew = startPosition + amp_traj*sin((clockStart-clockBegin)*freq_traj);

    %calculate distance into wall
        currentPosition = Motor1.ActualPosition;
        posError = currentPosition - startPositionNew;

        %Robot control
        %current = KW*distance
        current = -1 * KW*posError;        
        if (current > currentMaxP)
           current = currentMaxP;
           disp('Cuurent Max!')
        elseif (current < currentMaxN)
           current = currentMaxN;
           disp('Cuurent Max!')
        else
          Motor1.MotionWithCurrent(current);
        end
        
        %FES control - for Hi5
        if (posError < 0)
            fesAmpFlex = (KW_Flex * -posError) + minStimAmpFlex;
            fesAmpExt = 0;
        elseif (posError > 0)
            fesAmpFlex = 0;
            fesAmpExt = (KW_Ext * posError) + minStimAmpExt;
        else
             fesAmpFlex = 0;
             fesAmpExt = 0;
        end
        if ((clockNew > clockPrevFes+0.02) && extTurn == 0)      %Send stim every 0.04s (25Hz)
            cmd = generate_command(elecArrayFlex, [fesAmpFlex], round(pwFES(end)), elecname, velecnumber);
            writeline(bt,cmd)
            clockPrevFes = clockStart; 
            extTurn = 1; 
        elseif ((clockNew > clockPrevFes+0.02) && extTurn == 1)      %Send stim every 0.04s (25Hz)
            cmd = generate_command(elecArrayExt, [fesAmpExt], round(pwFES(end)), elecname, velecnumber);
            writeline(bt,cmd)
            clockPrevFes = clockStart; 
            extTurn = 1;
        end    
end

writeline(bt,strcat("stim off "));               %Start stimulation 
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
