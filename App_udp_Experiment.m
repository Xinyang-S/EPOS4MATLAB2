%% initialize UPD ports and arrays
%addpath 
clear u1 u2 u3 u4 u_force ur
u4 = udpport("LocalPort",1000,'TimeOut',100);
% u2 = udpport("LocalPort",3000); % open udp for FES pw from simulink, clear port if error
u_force = udpport('LocalPort',1263);
u2 = udp("LocalHost",3000);

port = 5566;
ur = udpport('LocalPort', port+2);
uw = udp('LocalHost', port+1);

data_string = [];
Error_array = [];

hi5Torque = {};
hi5Position = {};

%% Run Python Code Here to activate Grip sensor
% StartGripSensor()

%%
while(1)

%% Receive signal from App desinger
%read experiment index, total trial numbers and total block numbers(if have)
%at every start of experiments

flush(u4)
app_data = read(u4,10,'string');
app_data_string = split(app_data,'');
disp(app_data)
exp_num = str2num(app_data_string(2));
total_trial_num = str2num(cell2mat(strcat(app_data_string(3),app_data_string(4),app_data_string(5))));
trial_length = str2num(cell2mat(strcat(app_data_string(6),app_data_string(7),app_data_string(8))));
total_block_num = str2num(cell2mat(strcat(app_data_string(9),app_data_string(10),app_data_string(11))));

switch exp_num
%     
%------------------------Full-assisted target tracking---------------------
    case 1 %hi5 full-assisted target tracking
        
        fopen(uw);
        fopen(u2);
        % PID variables
        currentMaxP = 10000;
        currentMaxN = -10000;
        KW = 2*6400/90; %spring: 1 light, 2 med (best demo), 3 large, 4 heavy (limit)
        DW = 0*6400/90;
        filterLP_D = 0.7; %filter error velocity: 0.7, not very sensitive
        
        %read data from robot matlab ( encoder )
        flush(ur);
        dataR = int8(read(ur, 4, 'int8'));
        Zero_position = typecast(dataR, 'single');
        
        hi5Target_fullAssisted = {};%cell aray for positional data
        hi5WristPos = {};%cell aray for positional data
        hi5TargetPos = {};
        hi5Velocity = {};
        hi5Current = {};
        hi5Error = {};
        subject_traj_10_array = zeros(1,10);
        elapsed_time_10_array = zeros(1,10);
        target_traj_10_array = zeros(1,10);
        velocity_10_array = zeros(1,10);
        current_10_array = zeros(1,10);
        target_traj_array = [];
        subject_traj_array = [];
        velocity_array = [];
        current_array = [];
        error_array = [];
        currentPrev=0;
        current = 0;
        Error = 0;
        
        trial_num = 1;
        while(trial_num <= total_trial_num)
            c = clock;
            clockStart = c(4)*3600+c(5)*60+c(6);
            clockCurrent = clockStart;
            flush(ur);
            posErrorPrev = 0;
            posErrorDiff = 0;
%             pause(5);
            while (clockCurrent < clockStart + trial_length + 5)
                
                if clockCurrent < (clockStart+5)
                    k = 1;
                    target_traj_10_array = zeros(1,10);
                    while k <= 10
                        c = clock;
                        clockCurrent = c(4)*3600+c(5)*60+c(6);
                        
                        dataR = int8(read(ur, 4, 'int8'));
                        subject_current_pos = typecast(dataR, 'single');

                        subject_traj = -(subject_current_pos - Zero_position).*90/6400;
                        subject_traj_10_array(k) = subject_traj;

                        % target position
                        elapsed_time = clockCurrent - clockStart;
                        elapsed_time_10_array(k) = elapsed_time;
                        
                        dataW =  typecast(single([0 current]), 'int8');
                        fwrite(uw, dataW, 'int8');
                        k = k+1;
                    end

                    data_box = [roundn(target_traj_10_array,-5) roundn(subject_traj_10_array,-5) roundn(Error,-5) roundn(elapsed_time_10_array, -5)];
                    newV = typecast(single(data_box), 'int8')
                    fwrite(u2, newV, 'int8')
                    disp('Wrote!')
                else
                
                k = 1;
                
                while k <= 10
                    c = clock;
                    clockCurrent = c(4)*3600+c(5)*60+c(6);
                    % subject position
%                     subject_current_pos = Motor1.ActualPosition;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
                    dataR = int8(read(ur, 4, 'int8'));
                    subject_current_pos = typecast(dataR, 'single');
                    
                    subject_traj = -(subject_current_pos - Zero_position).*90/6400;
                    subject_traj_10_array(k) = subject_traj;
                    
                    % target position
                    elapsed_time = clockCurrent - clockStart;
                    elapsed_time_10_array(k) = elapsed_time;
                    
                    target_traj = 2*18.51*(sin((elapsed_time - 5)*pi/1.547)*sin((elapsed_time - 5)*pi/2.875));
                    target_traj_10_array(k) = target_traj;
                    
%                     ErrorSample = sqrt((target_traj-subject_traj)^2);
%                     Error_array = [Error_array ErrorSample];
%                     Error = mean(Error_array);

                    %control Hi5 - stiffness
                    position_Error = (- subject_traj) - target_traj;
                    posErrorDiffNew = position_Error - posErrorPrev;
                    posErrorDiff = posErrorDiff*(filterLP_D) + posErrorDiffNew*(1-filterLP_D); %damper can cause small vibrations 
                    posErrorPrev = position_Error;

                    current = -1*(KW*position_Error + DW*posErrorDiff);

                    if (current > currentMaxP)
                        disp('Wall Too Strong!')
                    elseif (current < currentMaxN)
                        disp('Wall Broke!')
                    else
                        current = safetyCheck(current);
                        if current == currentPrev
                            dataW =  typecast(single([0 current]), 'int8');
                            fwrite(uw, dataW, 'int8');
                        else
                            dataW =  typecast(single([1 current]), 'int8');
                            fwrite(uw, dataW, 'int8');
%                         Motor1.MotionWithCurrent(current);
                        end
                    end
                    
%                     velocity_10_array(k) = Motor1.ActualVelocity;
                    current_10_array(k) = current;
                    k = k+1;
                    currentPrev=current;
                
                end
                
                data_box = [roundn(target_traj_10_array,-5) roundn(subject_traj_10_array,-5) roundn(Error,-5) roundn(elapsed_time_10_array, -5)];
                newV = typecast(single(data_box), 'int8')
                fwrite(u2, newV, 'int8')
                disp('Wrote!')

%                 velocity_array = [velocity_array velocity_10_array];
                current_array = [current_array current_10_array];
                target_traj_array = [target_traj_array target_traj_10_array];
                subject_traj_array = [subject_traj_array subject_traj_10_array];
                end
            end
%             Motor1.MotionWithCurrent(0);
            dataW =  typecast(single([1 0]), 'int8');
            fwrite(uw, dataW, 'int8');
            
            trial_name = strcat('trial',num2str(trial_num));
            hi5WristPos.(trial_name) = subject_traj_array;
            hi5TargetPos.(trial_name) = target_traj_array;
%             hi5Velocity.(trial_name) = velocity_array;
            hi5Current.(trial_name) = current_array;
            hi5Error.(trial_name) = Error_array;
            target_traj_array = [];
            subject_traj_array = [];
            velocity_array = [];
            current_array = [];
            Error_array = [];
            disp(trial_num);
            trial_num = trial_num+1;
            disp('end of trial');
            % time for ready count down in AppDesigner
        end
        hi5Target_fullAssisted.hi5WristPos = hi5WristPos;
        hi5Target_fullAssisted.hi5TargetPos = hi5TargetPos;
%         hi5Target_fullAssisted.hi5Velocity = hi5Velocity;
        hi5Target_fullAssisted.hi5Current = hi5Current;
        hi5Target_fullAssisted.hi5Error = hi5Error;
        save ('hi5Target_fullAssisted.mat','hi5Target_fullAssisted');
        fclose(u2);
        fclose(uw);
%------------------------Semi-assisted target tracking---------------------
    case 2 %hi5 semi-assisted target tracking
        %hi5 initialization
        Motor1 = Epos4(0,0);
        Motor1.ClearErrorState;
        Motor1.DisableNode;
        Motor1.SetOperationMode( OperationModes.CurrentMode );
        Motor1.EnableNode;
        Motor1.ClearErrorState;
        
        % PID variables
        currentMaxP = 10000;
        currentMaxN = -10000;
        KW = 2.17*6400/90; %spring: 1 light, 2 med (best demo), 3 large, 4 heavy (limit)
        DW = 0*6400/90;
        posErrorDiff = 0;
        
        Zero_position = Motor1.ActualPosition;
        
        hi5Target_semiAssisted = {};%cell aray for positional data
        hi5WristPos = {};%cell aray for positional data
        hi5TargetPos = {};
        hi5Velocity = {};
        hi5Current = {};
        hi5Error = {};
        subject_traj_10_array = zeros(1,10);
        elapsed_time_10_array = zeros(1,10);
        target_traj_10_array = zeros(1,10);
        velocity_10_array = zeros(1,10);
        current_10_array = zeros(1,10);
        target_traj_array = [];
        subject_traj_array = [];
        velocity_array = [];
        current_array = [];
        Error_array = [];
        
        trial_num = 1;
        while(trial_num <= total_trial_num)
            c = clock;
            clockStart = c(4)*3600+c(5)*60+c(6);
            clockCurrent = clockStart;
            while (clockCurrent < clockStart + trial_length)
                k = 1;
                while k <= 10
                    c = clock;
                    clockCurrent = c(4)*3600+c(5)*60+c(6);
                    % subject position
                    subject_current_pos = Motor1.ActualPosition;
                    subject_traj = -(subject_current_pos - Zero_position)*90/6400;
                    subject_traj_10_array(k) = subject_traj;
                    
                    % target position
                    elapsed_time = clockCurrent - clockStart;
                    elapsed_time_10_array(k) = elapsed_time;
                    
                    target_traj = 2*18.51*(sin(elapsed_time*pi/1.547)*sin(elapsed_time*pi/2.875));
                    target_traj_10_array(k) = target_traj;
                    
                    ErrorSample = sqrt((target_traj-subject_traj)^2);
                    Error_array = [Error_array ErrorSample];
                    Error = mean(Error_array);

                    %control Hi5 - stiffness
                    position_Error = (-subject_traj) - target_traj;
                    current = -1*(KW*position_Error + DW*posErrorDiff);

                    if (current > currentMaxP)
                        disp('Wall Too Strong!')
                    elseif (current < currentMaxN)
                        disp('Wall Broke!')
                    else
                        current = safetyCheck(current);
                        Motor1.MotionWithCurrent(current);
                    end
                    
                    velocity_10_array(k) = Motor1.ActualVelocity;
                    current_10_array(k) = Motor1.ActualCurrent;
                    k = k+1;
                end
                
                data_box = [roundn(target_traj_10_array,-5) roundn(subject_traj_10_array,-5) roundn(Error,-5) roundn(elapsed_time_10_array, -5)];

                write(u2,data_box,"double","LocalHost",4000);

                velocity_array = [velocity_array velocity_10_array];
                current_array = [current_array current_10_array];
                target_traj_array = [target_traj_array target_traj_10_array];
                subject_traj_array = [subject_traj_array subject_traj_10_array];
            end
            Motor1.MotionWithCurrent(0);
            trial_name = strcat('trial',num2str(trial_num));
            hi5WristPos.(trial_name) = subject_traj_array;
            hi5TargetPos.(trial_name) = target_traj_array;
            hi5Velocity.(trial_name) = velocity_array;
            hi5Current.(trial_name) = current_array;
            hi5Error.(trial_name) = Error_array;
            target_traj_array = [];
            subject_traj_array = [];
            velocity_array = [];
            current_array = [];
            Error_array = [];
            disp(trial_num);
            trial_num = trial_num+1;
            disp('end of trial');
            pause(5);% time for ready count down in AppDesigner
        end
        hi5Target_semiAssisted.hi5WristPos = hi5WristPos;
        hi5Target_semiAssisted.hi5TargetPos = hi5TargetPos;
        hi5Target_semiAssisted.hi5Velocity = hi5Velocity;
        hi5Target_semiAssisted.hi5Current = hi5Current;
        hi5Target_semiAssisted.hi5Error = hi5Error;
        save ('hi5Target_semiAssisted.mat','hi5Target_semiAssisted');        
        
%------------------------Zero-assisted target tracking---------------------
    case 3 %hi5 zero-assisted target tracking track
        %hi5 initialization
        Motor1 = Epos4(0,0);
        Motor1.ClearErrorState;
        Motor1.DisableNode;
        Motor1.SetOperationMode( OperationModes.CurrentMode );
        Motor1.EnableNode;
        Motor1.ClearErrorState;
        
        Zero_position = Motor1.ActualPosition;
        
        hi5Target_zeroAssisted = {};%cell aray for positional data
        hi5WristPos = {};%cell aray for positional data
        hi5TargetPos = {};
        hi5Velocity = {};
        hi5Current = {};
        hi5Error = {};
        subject_traj_10_array = zeros(1,10);
        elapsed_time_10_array = zeros(1,10);
        target_traj_10_array = zeros(1,10);
        velocity_10_array = zeros(1,10);
        current_10_array = zeros(1,10);
        target_traj_array = [];
        subject_traj_array = [];
        velocity_array = [];
        current_array = [];
        error_array = [];
        
        trial_num = 1;
        while(trial_num <= total_trial_num)
            c = clock;
            clockStart = c(4)*3600+c(5)*60+c(6);
            clockCurrent = clockStart;
            while (clockCurrent < clockStart + trial_length)
                k = 1;
                while k <= 10
                    c = clock;
                    clockCurrent = c(4)*3600+c(5)*60+c(6);
                    % subject position
                    subject_current_pos = Motor1.ActualPosition;
                    subject_traj = -(subject_current_pos - Zero_position)*90/6400;
                    subject_traj_10_array(k) = subject_traj;
                    
                    % target position
                    elapsed_time = clockCurrent - clockStart;
                    elapsed_time_10_array(k) = elapsed_time;
                    
                    target_traj = 2*18.51*(sin(elapsed_time*pi/1.547)*sin(elapsed_time*pi/2.875));
                    target_traj_10_array(k) = target_traj;
                    
                    ErrorSample = sqrt((target_traj-subject_traj)^2);
                    Error_array = [Error_array ErrorSample];
                    Error = mean(Error_array);
                    
                    velocity_10_array(k) = Motor1.ActualVelocity;
                    current_10_array(k) = Motor1.ActualCurrent;
                    k = k+1;
                end
                
                data_box = [roundn(target_traj_10_array,-5) roundn(subject_traj_10_array,-5) roundn(Error,-5) roundn(elapsed_time_10_array, -5)];

                write(u2,data_box,"double","LocalHost",4000);

                velocity_array = [velocity_array velocity_10_array];
                current_array = [current_array current_10_array];
                target_traj_array = [target_traj_array target_traj_10_array];
                subject_traj_array = [subject_traj_array subject_traj_10_array];
            end
            trial_name = strcat('trial',num2str(trial_num));
            hi5WristPos.(trial_name) = subject_traj_array;
            hi5TargetPos.(trial_name) = target_traj_array;
            hi5Velocity.(trial_name) = velocity_array;
            hi5Current.(trial_name) = current_array;
            hi5Error.(trial_name) = error_array;
            target_traj_array = [];
            subject_traj_array = [];
            velocity_array = [];
            current_array = [];
            error_array = [];
            disp(trial_num);
            trial_num = trial_num+1;
            disp('end of trial');
            pause(5);% time for ready count down in AppDesigner
        end
        hi5Target_zeroAssisted.hi5WristPos = hi5WristPos;
        hi5Target_zeroAssisted.hi5TargetPos = hi5TargetPos;
        hi5Target_zeroAssisted.hi5Velocity = hi5Velocity;
        hi5Target_zeroAssisted.hi5Current = hi5Current;
        hi5Target_zeroAssisted.hi5Error = hi5Error;
        save ('hi5Target_zeroAssisted.mat','hi5Target_zeroAssisted');
        
        
%--------------------------HI5 position track------------------------------
    case 4 %hi5 position track
        %hi5 initialization
        Motor1 = Epos4(0,0);
        Motor1.ClearErrorState;
        Motor1.DisableNode;
        Motor1.SetOperationMode( OperationModes.CurrentMode );
        Motor1.EnableNode;
        Motor1.ClearErrorState;
        
        Zero_position = Motor1.ActualPosition;
        
        hi5Position_Track = {};%cell aray for positional data
        hi5WristPos = {};%cell aray for positional data
        hi5TargetPos = {};
        hi5Velocity = {};
        hi5Current = {};
        target_pos_10_array = zeros(1,10);
        elapsed_time_10_array = zeros(1,10);
        subject_traj_10_array = zeros(1,10);
        velocity_10_array = zeros(1,10);
        current_10_array = zeros(1,10);
        target_pos_array = [];
        subject_traj_array = [];
        velocity_array = [];
        current_array = [];
        position_array = [];
        
        Error = 0;
        posFlag = 0;
        trial_index = 1;

        
        block_num = 1;
        while(block_num <= total_block_num)
            trial_num = 1;
            for i =1:1:total_trial_num/8 
                position_array = [position_array 40 20 -10 -20];
            end
            while(trial_num <= total_trial_num)
                c = clock;
                clockStart = c(4)*3600+c(5)*60+c(6);
                clockCurrent = clockStart;
                executed = 0;
                while (clockCurrent < clockStart + trial_length)
                    if posFlag == 0 && executed ==0
                        index = randi(size(position_array));
                        target_pos = position_array(index);
                        position_array(index) = [];
                        executed = 1;
                    elseif posFlag == 1
                        target_pos = 0;
                    end
                    k = 1;
                    while k <= 10
                        
                        target_pos_10_array(k) = target_pos;
                        
                        c = clock;
                        clockCurrent = c(4)*3600+c(5)*60+c(6);
                        elapsed_time = clockCurrent - clockStart;
                        elapsed_time_10_array(k) = elapsed_time;
                        
                        % subject position
                        subject_current_pos = Motor1.ActualPosition;
                        subject_traj = -(subject_current_pos - Zero_position)*90/6400;
                        subject_traj_10_array(k) = subject_traj;
                        
                        velocity_10_array(k) = Motor1.ActualVelocity;
                        current_10_array(k) = Motor1.ActualCurrent;
                        k = k+1;
                    end

                    data_box = [roundn(target_pos_10_array,-5) roundn(subject_traj_10_array,-5) roundn(Error,-5) roundn(elapsed_time_10_array, -5)];
                    write(u2,data_box,"double","LocalHost",4000);
                    velocity_array = [velocity_array velocity_10_array];
                    current_array = [current_array current_10_array];
                    target_pos_array = [target_pos_array target_pos_10_array];
                    subject_traj_array = [subject_traj_array subject_traj_10_array];
                end
                
                posFlag = ~posFlag;
                if block_num == 1
                    trial_name = strcat('Slow_trial',num2str(trial_index));
                elseif block_num == 2
                    trial_name = strcat('Fast_trial',num2str(trial_index));
                else
                    trial_name = strcat('trial',num2str(trial_index));
                end
                
                hi5WristPos.(trial_name) = subject_traj_array;
                hi5TargetPos.(trial_name) = target_pos_array;
                hi5Velocity.(trial_name) = velocity_array;
                hi5Current.(trial_name) = current_array;
                target_pos_array = [];
                subject_traj_array = [];
                velocity_array = [];
                current_array = [];
                disp(trial_num);
                trial_num = trial_num + 1;
                trial_index = trial_index + 1;
                disp('end of trial');
            end
            block_num = block_num + 1;
            disp(['Block number: ', num2str(block_num)]);
            pause(5);% time for ready count down in AppDesigner
        end
        hi5Position_Track.hi5WristPos = hi5WristPos;
        hi5Position_Track.hi5TargetPos = hi5TargetPos;
        hi5Position_Track.hi5Velocity = hi5Velocity;
        hi5Position_Track.hi5Current = hi5Current;
        save ('hi5Position_Track.mat','hi5Position_Track');
        
        
%------------------------HI5 torque stablization---------------------------
    case 5 %torque stablization
        %hi5 initialization
        Motor1 = Epos4(0,0);
        Motor1.ClearErrorState;
        Motor1.DisableNode;
        Motor1.SetOperationMode( OperationModes.CurrentMode );
        Motor1.EnableNode;
        Motor1.ClearErrorState;
        
        Zero_position = Motor1.ActualPosition;
        
        hi5Torque_Stablization = {};%cell aray for positional data
        hi5WristPos = {};%cell aray for positional data
        hi5TargetPos = {};
        hi5Velocity = {};
        hi5Current = {};
        target_strength_array = [];
        subject_traj_array = [];
        velocity_array = [];
        current_array = [];
        
        Error = 0;
        block_num = 1;
        trial_index = 1;
        while(block_num <= total_block_num)
            trial_num = 1;
            Zero_position = Motor1.ActualPosition;
            while(trial_num <= total_trial_num)
                disp(['Trial index: ', num2str(trial_index)]);
                c = clock;
                clockStart = c(4)*3600+c(5)*60+c(6);
                clockCurrent = clockStart;
                strength = randi(3);
                
                % strength = 1,2,3 for first three trials
                if trial_index == 1
                    strength = 1;
                elseif trial_index == 2
                    strength = 2;
                elseif trial_index == 3
                    strength = 3;
                end
                
                while (clockCurrent < clockStart + trial_length)
                    
                    c = clock;
                    clockCurrent = c(4)*3600+c(5)*60+c(6);
                    elapsed_time = clockCurrent - clockStart;
                    
                    if (elapsed_time > 4)
                        strength = 0;
                    end
                    
                    switch strength
                        case 0
                            current = 0;
                            current = safetyCheck(current);
                            Motor1.MotionWithCurrent(current);
                        case 1
                            if (elapsed_time > 3.5)
                                current=-2000*(2*(4-elapsed_time));% current *(0.5 second)*2, make the length within brackets = 1
                                current = safetyCheck(current);
                                Motor1.MotionWithCurrent(current);
                            elseif (elapsed_time > 0.5)
                                current = -2000;
                                current = safetyCheck(current);
                                Motor1.MotionWithCurrent(current);
                            elseif (elapsed_time > 0)
                                current=-2000*(2*(elapsed_time));
                                current = safetyCheck(current);
                                Motor1.MotionWithCurrent(current);
                            end
                        case 2
                            if (elapsed_time > 3.5)
                                current=-3000*(2*(4-elapsed_time));% current *(0.5 second)*2, make the length within brackets = 1
                                current = safetyCheck(current);
                                Motor1.MotionWithCurrent(current);
                            elseif (elapsed_time > 0.5)
                                current = -3000;
                                current = safetyCheck(current);
                                Motor1.MotionWithCurrent(current);
                            elseif (elapsed_time > 0)
                                current=-3000*(2*(elapsed_time));
                                current = safetyCheck(current);
                                Motor1.MotionWithCurrent(current);
                            end
                        case 3
                            if (elapsed_time > 3.5)
                                current=-4000*(2*(4-elapsed_time));% current *(0.5 second)*2, make the length within brackets = 1
                                current = safetyCheck(current);
                                Motor1.MotionWithCurrent(current);
                            elseif (elapsed_time > 0.5)
                                current = -4000;
                                current = safetyCheck(current);
                                Motor1.MotionWithCurrent(current);
                            elseif (elapsed_time > 0)
                                current=-4000*(2*(elapsed_time));
                                current = safetyCheck(current);
                                Motor1.MotionWithCurrent(current);
                            end
                    end
                    
                    subject_position = Motor1.ActualPosition;
                    subject_traj = -(subject_position - Zero_position)*90/6400;
                    
                    data_box = [roundn(strength,-5) roundn(subject_traj,-5) roundn(Error,-5) roundn(elapsed_time, -5)];
                    disp(data_box);
                    for i = 1:(length(data_box))
                        data_string = [data_string uniform_data(data_box(i))];
                    end
                    write(u2,data_string,"string","LocalHost",4000);
                    data_string = [];
                    velocity = Motor1.ActualVelocity;
                    velocity_array = [velocity_array velocity];
                    motor_current = Motor1.ActualCurrent;
                    current_array = [current_array motor_current];
                    target_strength_array = [target_strength_array strength];
                    subject_traj_array = [subject_traj_array subject_traj];
                end
                trial_name = strcat('trial',num2str(trial_num));
                hi5WristPos.(trial_name) = subject_traj_array;
                hi5TargetPos.(trial_name) = target_strength_array;
                hi5Velocity.(trial_name) = velocity_array;
                hi5Current.(trial_name) = current_array;
                target_strength_array = [];
                subject_traj_array = [];
                velocity_array = [];
                current_array = [];
                trial_num = trial_num + 1;
                trial_index = trial_index + 1;
                disp('end of trial');
            end
            block_num = block_num + 1;
            disp(['Block number: ', num2str(block_num)]);
            pause(5);% time for ready count down in AppDesigner
        end
        hi5Torque_Stablization.hi5WristPos = hi5WristPos;
        hi5Torque_Stablization.hi5TargetPos = hi5TargetPos;
        hi5Torque_Stablization.hi5Velocity = hi5Velocity;
        hi5Torque_Stablization.hi5Current = hi5Current;
        save ('hi5Torque_Stablization.mat','hi5Torque_Stablization');

        
%------------------------Grip target tracking------------------------------
    case 6 %grip tracking
        fclose(u2)
        fopen(u2);
        total_trial_num = 2;
        trial_length = 10;
        disp('Task 6: grip tracking')
        force_array = [];
        force_target_array = [];
        Error_array = [];
        elapsed_time = zeros(1,10);
        force_target = [];
        trial_num = 1;
        gripTrack = {};
        gripforce = {};
        gripforce_target = {};
        gripforce_error = {};
        
        while(trial_num <= total_trial_num)
            c = clock;
            clockStart = c(4)*3600+c(5)*60+c(6);
            clockCurrent = clockStart; 
            Error_array = [];
            Error = 0;
            while (clockCurrent < clockStart + trial_length + 5)
                if clockCurrent < (clockStart + 5) % countdown
                    k = 1;
                    while k <= 10
                        c = clock;
                        clockCurrent = c(4)*3600+c(5)*60+c(6);
                        elapsed_time(k) = clockCurrent - clockStart;
                        k = k+1;
                    end
                    
                    force_target = -[37.02 37.02 37.02 37.02 37.02 37.02 37.02 37.02 37.02 37.02];
                    force = read(u_force,10,'single');
                    data_box = [roundn(force_target,-5) roundn(force,-5) roundn(Error,-5) roundn(elapsed_time, -5)];
                    elapsed_time = [];

                    newV = typecast(single(data_box), 'int8')
                    fwrite(u2, newV, 'int8')
                    traj_start_flag = 1;
                elseif clockCurrent >= clockStart + 5 % in trial
                    
                    c = clock;
                    clockCurrent = c(4)*3600+c(5)*60+c(6);
                    k = 1;
                    
                    if traj_start_flag == 1
                        c = clock;
                        traj_start_time = c(4)*3600+c(5)*60+c(6) - clockStart;
                        traj_start_flag = ~traj_start_flag;
                    end
                    while k <= 10
                        c = clock;
                        clockCurrent = c(4)*3600+c(5)*60+c(6);
                        elapsed_time(k) = clockCurrent - clockStart;
                        k = k+1;
                    end
                    force_target =2.*18.51.*(sin((elapsed_time-traj_start_time+10).*pi/1.547).*sin((elapsed_time-traj_start_time+10).*pi/2.875));
                    force = read(u_force,10,'single');
    %                 ErrorSample = sqrt((force_target-force).^2);
    %                 Error_array = [Error_array ErrorSample];
    %                 Error = mean(Error_array);
                    data_box = [roundn(force_target,-5) roundn(force,-5) roundn(Error,-5) roundn(elapsed_time, -5)];
                    

                    newV = typecast(single(data_box), 'int8')
                    fwrite(u2, newV, 'int8')
                    force_array = [force_array force];
                    force_target_array = [force_target_array force_target];
                    force_target = [];
                end
            end
            
            force_name = strcat('trial',num2str(trial_num));
            gripforce.(force_name) = force_array;
            gripforce_target.(force_name) = force_target_array;
            gripforce_error.(force_name) = Error_array;
            force_target_array = [];
            force_array = [];
            Error_array = [];
            disp(trial_num);
            trial_num = trial_num+1;
            disp('end of trial');
        end
        gripTrack.gripForce = gripforce;
        gripTrack.gripForceTarget = gripforce_target;
%         gripTrack.gripforce_error = gripforce_error;
        save ('gripTrack.mat','gripTrack');
        fclose(u2)
        
%------------------------Grip force maintain-------------------------------
    case 7 %grip force maintain
        fclose(u2)
        fopen(u2);
        disp('Task 7: grip force maintain')
        force_array = [];
        force_target_array = [];
        block_num = 1;
        gripForceMaintain = {};
        gripforce = {};
        gripforce_target = {};
        strength_10_array = zeros(1,10);
        elapsed_time_10_array = zeros(1,10);
        Error = 0;
        trial_index = 1;
        while(block_num <= total_block_num)
            trial_num = 1;
            block_flag = 1;
            while(trial_num <= total_trial_num)
                disp(['Trial index: ', num2str(trial_index)]);
                c = clock;
                clockStart = c(4)*3600+c(5)*60+c(6);
                clockCurrent = clockStart;
                strength = randi(3);
                if trial_index == 1
                    strength = 1;
                elseif trial_index == 2
                    strength = 2;
                elseif trial_index == 3
                    strength = 3;
                end
                
                while (clockCurrent < clockStart + trial_length)
                    if mod(trial_index,total_trial_num) == 1 && (block_flag ==1)
                        c = clock;
                        clock_count_down_start = c(4)*3600+c(5)*60+c(6);
                        clock_count_down_current = clock_count_down_start;
                        
                        while clock_count_down_current < clock_count_down_start + 5
                            c = clock;
                            clock_count_down_current = c(4)*3600+c(5)*60+c(6);
                            force = read(u_force,10,'single');

                            data_box = [roundn(zeros(1,10),-5) roundn(force,-5) roundn(Error,-5) roundn(elapsed_time_10_array, -5)];
        %                     disp(data_box);
                            newV = typecast(single(data_box), 'int8')
                            fwrite(u2, newV, 'int8')
                        end
                        block_flag = ~block_flag;
                        c = clock;
                        clockCurrent = c(4)*3600+c(5)*60+c(6);
                        clockStart = clockCurrent;
                    end
                    c = clock;
                    clockCurrent = c(4)*3600+c(5)*60+c(6);
                    if (clockCurrent > clockStart + 3)
                        strength = 0;
                    end
                    
                    force = read(u_force,10,'single');
                    
                    k = 1;
                    while k <= 10
                        
                        elapsed_time = clockCurrent - clockStart;
                        strength_10_array(k) = strength;
                        elapsed_time_10_array(k) = elapsed_time;
                        
                        k = k+1;
                    end
                    
                    data_box = [roundn(strength_10_array,-5) roundn(force,-5) roundn(Error,-5) roundn(elapsed_time_10_array, -5)];
%                     disp(data_box);
                    newV = typecast(single(data_box), 'int8')
                    fwrite(u2, newV, 'int8')
                    force_array = [force_array force];
                    force_target_array = [force_target_array strength_10_array];
%                     pause(0.002)
                end
                force_name = strcat('trial',num2str(trial_index));
                gripforce.(force_name) = force_array;
                gripforce_target.(force_name) = force_target_array;
                force_target_array = [];
                force_array = [];
                trial_num = trial_num + 1;
                trial_index = trial_index + 1;
                disp('end of trial');
            end
            block_num = block_num + 1;
            disp(['Block number: ', num2str(block_num)]);
        end
        gripForceMaintain.gripforce = gripforce;
        gripForceMaintain.gripforcetarget = gripforce_target;
        save ('gripForceMaintain.mat','gripForceMaintain');
        fclose(u2)
        
        
%------------------------Combine all the Data------------------------------
    case 9 % save all the data in workspace
        load('hi5Target_fullAssisted.mat');
        load('hi5Target_semiAssisted.mat');
        load('hi5Target_zeroAssisted.mat');
        load('hi5Position.mat');
        load('hi5Torque_Stablization.mat');
        load('gripTrack.mat');
        load('gripForceMaintain.mat');
        load('username.mat');
        dt=string(datetime('now','TimeZone','local','Format','uuuu_MM_dd''T''HH_mm_ss'));
        expName=strcat('experimentData_',Username,'_',dt,'.mat');
        save(expName,'hi5Target_fullAssisted','hi5Target_semiAssisted','hi5Target_zeroAssisted','hi5Positioning','hi5Torque_Stablization','gripTrack','gripForceMaintain')
        disp(strcat('Data was saved succesfully! The file name is'," ",expName));
        
end
end
