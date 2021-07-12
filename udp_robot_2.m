%  UDP ROBOT
%%
try
    delete(Motor1);
    disp('estop')
catch
end
%%ti
% read udpport (no flush)
% write udp
clear ur;
%fclose(uw);
port = 5566;
ur = udpport('LocalPort', port+1,'timeout',0.001);
uw = udp('LocalHost', port+2,'timeout',100);
fopen(uw);
trajectory_array = [];
time_array = [];
% initialize motor
Motor1 = Epos4(0,0);
Motor1.ClearErrorState;
Motor1.DisableNode;
Motor1.SetOperationMode( OperationModes.CurrentMode );
Motor1.EnableNode;
Motor1.ClearErrorState;

Motor1.SetWithDigital(1, 0);
Motor1.SetWithDigital(2, 0);

c = clock();
clockStart = (c(4)*3600+c(5)*60+c(6))/1000;
current = 0;
encoder = 0;
Motor1.MotionWithCurrent(0);
encoderStart = Motor1.ActualPosition;
KW = 0;
DW= 0;
trajectory = 0;
mode = 4;
init = 0;
current_pre = 0;
filterLP_D = 0.7; posError = 0; posErrorDiffNew=0; posErrorDiff = 0; posErrorPrev = 0;
j = 1;
tic
while(1)
    toc
    tic
    if mod(j,10) == 0
        flush(ur)
    end
    encoder = Motor1.ActualPosition - encoderStart;
    %calculate error velocity
    posError = encoder - trajectory*6400/90;
    posErrorDiffNew = posError - posErrorPrev;
    posErrorDiff = posErrorDiff*(filterLP_D) + posErrorDiffNew*(1-filterLP_D); %damper can cause small vibrations 
    posErrorPrev = posError;
    current = KW*posError + DW*posErrorDiff;
    current = safetyCheck(current);
%     current_pre = current;
%     Motor1.SetWithAnalog(2,encTomV(encoderStart, Motor1.ActualPosition));
%     Motor1.SetWithAnalog(2,encTomV(encoderStart, trajectory*4000/120+4000/2));
%     Motor1.SetWithAnalog(2,trajectory*4000/120+4000/2);
%     encTomV(0,trajectory*4000/120+4000/2)
% encTomV(encoderStart, Motor1.ActualPosition)
% encTomV(encoderStart, trajectory*4000/120+4000/2)

%     encTomV(encoderStart, Motor1.ActualPosition)
    
    if ( mode == 0 )%zero position
        init = 1;
        encoderStart = Motor1.ActualPosition;
    elseif ( mode  == 1 ) % medium stiffness
        init = 0;
        KW = -2;
        DW= 0;
        Motor1.MotionWithCurrent(current);
    elseif ( mode  == 2 ) % full assistance (high stiffness)
        init = 0;
        KW = -4;
        DW = 0;
        Motor1.MotionWithCurrent(current);
    elseif ( mode == 3 )
        init = 1;
        Motor1.MotionWithCurrent(trajectory);
    elseif ( mode == 4 )%zero assistance
        init = 0;
%         Motor1.MotionWithCurrent(0);
    elseif (mode == 5)
        Motor1.MotionWithCurrent(0);
    elseif (mode == 6)% trigger for trial
        Motor1.SetWithDigital(1, 0);
        Motor1.SetWithDigital(1, 1);
        Motor1.SetWithDigital(1, 0);
    elseif (mode == 7)% trigger for task
        Motor1.SetWithDigital(2, 0);
        Motor1.SetWithDigital(2, 1);
        Motor1.SetWithDigital(2, 0);
    elseif ( mode  == 8 ) % medium stiffness
        init = 0;

        KW_zero=-30/abs(encoder/1000);
        if KW_zero < -30
            KW_zero = -30;
        end
        encoder = Motor1.ActualPosition - encoderStart;
        current_zero = KW_zero*0.05*encoder;
        current_zero = safetyCheck(current_zero);
        Motor1.MotionWithCurrent(current_zero);
    else
    end
    %write data
    dataW =  typecast(single(encoder), 'int8');
    fwrite(uw, dataW, 'int8');
    pause(0.001);
    
    %read data
    try
        c = clock();
        c2= c(4)*3600+c(5)*60+c(6);
        dataR = int8(read(ur,8, 'int8'));
        dataR1 = typecast(dataR, 'single');
%         Motor1.SetWithAnalog(2,encTomV(encoderStart, Motor1.ActualPosition));
        mode = dataR1(1);
%         trajectory = dataR1(2)*6400/90;
        trajectory = dataR1(2);
        time_array = [time_array c2];
        trajectory_array = [trajectory_array trajectory];
    catch
        continue
    end
    j = j+1;
end

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
    posError = Motor1.ActualPosition - startPosition;
   
    %calculate error velocity
    posErrorDiff = posError - posErrorPrev;
    posClockDiff = clockStart - posClockPrev;
   
    %if inside wall - current = KW*distance + DW*Velocity
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
