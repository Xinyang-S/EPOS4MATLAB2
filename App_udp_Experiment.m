%% initialize UPD ports and arrays
%addpath 
clear u1 u2 u3 u4 u_force ur ur_rda u_checkpause
u4 = udpport("LocalPort",1000,'TimeOut',1000);
u_checkpause = udpport('LocalPort', 1111,'timeout', 0.01);
% u2 = udpport("LocalPort",3000); % open udp for FES pw from simulink, clear port if error
u_force = udpport('LocalPort',1263);
u2 = udp("LocalHost",3000);
u2.InputBufferSize = 250;
u2.OutputBufferSize = 250;

port = 5566;
ur = udpport('LocalPort', port+2);
uw = udp('LocalHost', port+1);

port_rda = 6666; 
uw_rda = udp('LocalHost', port_rda+1,'timeout',100);
ur_rda = udpport('LocalPort', port_rda+2);

exp_info = [];
data_string = [];
Score_array = [];

hi5Torque = {};
hi5Position = {};

trigger_10_array = [1 zeros(1,9)];
non_trigger_10_array = zeros(1,10);


%% Run Python Code Here to activate Grip sensor
% StartGripSensor()

%%
while(1)

%% Receive signal from App desinger
%read experiment index, total trial numbers and total block numbers(if have)
%at every start of experiments

flush(u4)
app_data = read(u4,12,'string');
app_data_string = split(app_data,'');
disp(app_data)
exp_num = str2double(cell2mat(strcat(app_data_string(2),app_data_string(3),app_data_string(4))));
total_trial_num = str2double(cell2mat(strcat(app_data_string(5),app_data_string(6),app_data_string(7))));
trial_length = str2double(cell2mat(strcat(app_data_string(8),app_data_string(9),app_data_string(10))));
total_block_num = str2double(cell2mat(strcat(app_data_string(11),app_data_string(12),app_data_string(13))));

countdown = 3;

switch exp_num
%
%------------------------Full-assisted target tracking---------------------
    case 1 %hi5 full-assisted target tracking
        
        fopen(uw);
        fopen(u2);
        %read data from robot matlab ( encoder )
        flush(ur);
        flush(ur_rda);
        
        dataW =  typecast(single([0 0]), 'int8');%set zero position
        fwrite(uw, dataW, 'int8');
        
        dataR = int8(read(ur, 4, 'int8'));
        Zero_position = typecast(dataR, 'single');
        
        hi5Target_fullAssisted = {};%cell aray for positional data
        hi5WristPos = {};%cell aray for positional data
        hi5TargetPos = {};
        hi5Velocity = {};
        hi5Current = {};
        hi5Error = {};
        hi5EEG = {};
        hi5ZeroPoint = {};
        hi5Trigger = {};
        target_traj_array = [];
        subject_traj_array = [];
        velocity_array = [];
        current_array = [];
        error_array = [];
        error_trial_array = [];
        trigger_array = [];
        eeg_data = [];
        currentPrev=0;
        current = 0;
        Error = 0;
        Score = 0;
        
        trial_num = 1;
        
        zero_point_array = zeros(1,(total_trial_num));
        j = 1;
        for i = 1:(floor(((total_trial_num))/3)):(((total_trial_num)) - (floor(((total_trial_num))/3))+1)
            if j == 1
                zero_point_array(i:(i+(floor(((total_trial_num))/3))-1)) = 3.094;
            elseif j == 2
                zero_point_array(i:(i+(floor(((total_trial_num))/3))-1)) = 14.375;
            end
            j = j+1;
        end
        
        dataW =  typecast(single([7 0]), 'int8');%start trigger of task PIN 2
        fwrite(uw, dataW, 'int8');
        sequenceSave(exp_num);
        
        dataW =  typecast(single([4 0]), 'int8');%set current to 0
        fwrite(uw, dataW, 'int8');
        
        while(trial_num <= total_trial_num)
            
            trial_trigger_flag = 1;
            
            disp('start of trial')
            disp(trial_num);
            index = randi(length(zero_point_array));
            zero_point = zero_point_array(index);
            zero_point_array(index) = [];
            
            pause_value = 0;
            
            try
                pause = read(u_checkpause,1,'string');
                pause_string = split(pause,'');
                pause_value = str2double(pause_string(2));
                if pause_value == 1
                    disp('pause value:');
                    disp(pause_value);
                end
            catch
            end
            
            while pause_value
                try
                    pause = read(u_checkpause,1,'string');
                    pause_string = split(pause,'');
                    pause_value = str2double(pause_string(2));
                    if pause_value == 0
                        disp('pause value:');
                        disp(pause_value);
                    end
                catch
                end
            end
            
            flush(ur_rda);
            flush(ur);
            c = clock;
            clockStart = c(4)*3600+c(5)*60+c(6);
            clockCurrent = clockStart;
            k = 1;
            while (clockCurrent < clockStart + trial_length + countdown)
                if clockCurrent < (clockStart+countdown)
                    
                    c = clock;
                    clockCurrent = c(4)*3600+c(5)*60+c(6);

                    dataR = int8(read(ur, 4, 'int8'));
                    subject_current_pos = typecast(dataR, 'single');

                    subject_traj = -(subject_current_pos).*90/6400;

                    % target position
                    elapsed_time = clockCurrent - clockStart;
                    if mod(k,2) == 0
                        dataR_rda = int8(read(ur_rda, 264, 'int8'));
                        eeg_data_vector = typecast(dataR_rda, 'single');
                        eeg_data = [eeg_data eeg_data_vector(1:33)' ,eeg_data_vector(34:66)'];
                    end
                    
                    target_traj = 0;
                    
                    dataW =  typecast(single([8 current]), 'int8');
                    fwrite(uw, dataW, 'int8');
                    
                    trigger_array = [trigger_array 0];
                    target_traj_array = [target_traj_array target_traj];
                    subject_traj_array = [subject_traj_array subject_traj];
                    
                else
                
                    if trial_trigger_flag
                        dataW =  typecast(single([6 0]), 'int8');%start trigger of trial PIN 1
                        fwrite(uw, dataW, 'int8');
                        trial_trigger_flag = ~trial_trigger_flag;
                        trigger_array = [trigger_array 1];
                    else
                        trigger_array = [trigger_array 0];
                    end
                    
                    c = clock;
                    clockCurrent = c(4)*3600+c(5)*60+c(6);

                    dataR = int8(read(ur, 4, 'int8'));
                    subject_current_pos = typecast(dataR, 'single');

                    subject_traj = -(subject_current_pos)*90/6400;

                    % target position
                    elapsed_time = clockCurrent - clockStart;
                    
                    target_traj = 2*18.51*(sin((elapsed_time - countdown + zero_point)*pi/1.547)*sin((elapsed_time - countdown + zero_point)*pi/2.875));
                    
                    if mod(k,2) == 0
                        dataR_rda = int8(read(ur_rda, 264, 'int8'));
                        eeg_data_vector = typecast(dataR_rda, 'single');
                        eeg_data = [eeg_data eeg_data_vector(1:33)' ,eeg_data_vector(34:66)'];
%                         for i = 1:33:628
%                             eeg_data = [eeg_data eeg_data_vector(i:i+32)'];
%                         end
                    end


                    dataW =  typecast(single([2 target_traj]), 'int8');
                    fwrite(uw, dataW, 'int8');

                    
                    target_traj_array = [target_traj_array target_traj];
                    subject_traj_array = [subject_traj_array subject_traj];
                    error_array = [error_array (target_traj + subject_traj)^2];
                    error_trial_array = [error_trial_array (target_traj + subject_traj)^2];
                    
                    Score = 100 - mean(error_trial_array)/3;
                
                    if Score < 0 
                        Score = 0;
                    end
                    
                end
                k = k+1;
                flush(ur)
                
                data_box = [roundn(target_traj,-5) roundn(subject_traj,-5) roundn(Score,-5) roundn(trial_num, -5)];
                newV = typecast(single(data_box), 'int8');
                fwrite(u2, newV, 'int8')
                
                
            end
            error_trial_array = [];
            
%             Motor1.MotionWithCurrent(0);
            dataW =  typecast(single([5 0]), 'int8');
            fwrite(uw, dataW, 'int8');
            disp('zero')
            
            trial_name = strcat('trial',num2str(trial_num));
            hi5WristPos.(trial_name) = subject_traj_array;
            hi5TargetPos.(trial_name) = target_traj_array;
            hi5EEG.(trial_name) = eeg_data;
            hi5ZeroPoint.(trial_name) = zero_point;
            hi5Trigger.(trial_name) = trigger_array;
%             hi5Velocity.(trial_name) = velocity_array;
%             hi5Current.(trial_name) = current_array;
%             hi5Error.(trial_name) = Error_array;
            target_traj_array = [];
            subject_traj_array = [];
            eeg_data = [];
            trigger_array = [];
            
            disp('end of trial');
            trial_num = trial_num+1;
            % time for ready count down in AppDesigner
        end
        hi5Target_fullAssisted.hi5WristPos = hi5WristPos;
        hi5Target_fullAssisted.hi5TargetPos = hi5TargetPos;
%         hi5Target_fullAssisted.hi5Velocity = hi5Velocity;
%         hi5Target_fullAssisted.hi5Current = hi5Current;
%         hi5Target_fullAssisted.hi5Error = hi5Error;
        hi5Target_fullAssisted.hi5EEG = hi5EEG;
        hi5Target_fullAssisted.hi5ZeroPoint = hi5ZeroPoint;
        hi5Target_fullAssisted.hi5Trigger = hi5Trigger;
        save ('hi5Target_fullAssisted.mat','hi5Target_fullAssisted');
        
        Error = mean(error_array);
        Score = 100 - Error/3;
        
        if Score < 0 
            Score = 0;
        end
        
        Score_array  = [Score_array Score];
        Master_score = mean(Score_array)
        fclose(u2);
        fclose(uw);
%------------------------Semi-assisted target tracking---------------------
    case 2 %hi5 semi-assisted target tracking
        fopen(uw);
        fopen(u2);
        
        flush(ur);
        flush(ur_rda);
        
        dataW =  typecast(single([0 0]), 'int8');%set zero position
        fwrite(uw, dataW, 'int8');
        
        dataR = int8(read(ur, 4, 'int8'));
        Zero_position = typecast(dataR, 'single');
        
        hi5Target_semiAssisted = {};%cell aray for positional data
        hi5WristPos = {};%cell aray for positional data
        hi5TargetPos = {};
        hi5Velocity = {};
        hi5Current = {};
        hi5Error = {};
        hi5EEG = {};
        hi5ZeroPoint = {};
        hi5Trigger = {};
        target_traj_array = [];
        subject_traj_array = [];
        velocity_array = [];
        current_array = [];
        error_array = [];
        error_trial_array = [];
        eeg_data = [];
        trigger_array = [];
        currentPrev=0;
        current = 0;
        Error = 0;
        Score = 0;
        trial_num = 1;
        
        zero_point_array = zeros(1,(total_trial_num));
        j = 1;
        for i = 1:(floor(((total_trial_num))/3)):(((total_trial_num)) - (floor(((total_trial_num))/3))+1)
            if j == 1
                zero_point_array(i:(i+(floor(((total_trial_num))/3))-1)) = 3.094;
            elseif j == 2
                zero_point_array(i:(i+(floor(((total_trial_num))/3))-1)) = 14.375;
            end
            j = j+1;
        end
        
        dataW =  typecast(single([4 0]), 'int8');%zero assistance
        fwrite(uw, dataW, 'int8');
        
        dataW =  typecast(single([7 0]), 'int8');%start trigger of task PIN 2
        fwrite(uw, dataW, 'int8');
        sequenceSave(exp_num)
        
        dataW =  typecast(single([4 0]), 'int8');%set current to 0
        fwrite(uw, dataW, 'int8');
        
        while(trial_num <= total_trial_num)
            data_box=[];
            trial_trigger_flag = 1;
            pause_value = 0;
            
            try
                pause = read(u_checkpause,1,'string');
                pause_string = split(pause,'');
                pause_value = str2double(pause_string(2));
                if pause_value == 1
                    disp('pause value:');
                    disp(pause_value);
                end
            catch
            end
            
            while pause_value
                try
                    pause = read(u_checkpause,1,'string');
                    pause_string = split(pause,'');
                    pause_value = str2double(pause_string(2));
                    if pause_value == 0
                        disp('pause value:');
                        disp(pause_value);
                    end
                catch
                end
            end
            
            flush(ur_rda);
            flush(ur);
            c = clock;
            clockStart = c(4)*3600+c(5)*60+c(6);
            clockCurrent = clockStart;
            
            index = randi(length(zero_point_array));
            zero_point = zero_point_array(index);
            zero_point_array(index) = [];
            k = 1;
            
            while (clockCurrent < clockStart + trial_length + countdown)

                if clockCurrent < (clockStart + countdown)
                    
                    c = clock;
                    clockCurrent = c(4)*3600+c(5)*60+c(6);

                    dataR = int8(read(ur, 4, 'int8'));
                    subject_current_pos = typecast(dataR, 'single');

                    subject_traj = -(subject_current_pos)*90/6400;

                    dataW =  typecast(single([8 current]), 'int8');
                    fwrite(uw, dataW, 'int8');
                    
                    target_traj = 0;
                    
                    if mod(k,2) == 0
                        dataR_rda = int8(read(ur_rda, 264, 'int8'));
                        eeg_data_vector = typecast(dataR_rda, 'single');
                        eeg_data = [eeg_data eeg_data_vector(1:33)' ,eeg_data_vector(34:66)'];
                    end
                    
                    trigger_array = [trigger_array 0];
                    target_traj_array = [target_traj_array target_traj];
                    subject_traj_array = [subject_traj_array subject_traj];
                    
                else
                    
                    if trial_trigger_flag
                        dataW =  typecast(single([6 0]), 'int8');%start trigger of trial PIN 1
                        fwrite(uw, dataW, 'int8');
                        trial_trigger_flag = ~trial_trigger_flag;
                        trigger_array = [trigger_array 1];
                    else
                        trigger_array = [trigger_array 0];
                    end
                    
                    c = clock;
                    clockCurrent = c(4)*3600+c(5)*60+c(6);
                    % subject position
%                     subject_current_pos = Motor1.ActualPosition;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
                    dataR = int8(read(ur, 4, 'int8'));
                    subject_current_pos = typecast(dataR, 'single');

                    subject_traj = -(subject_current_pos)*90/6400;

                    % target position
                    elapsed_time = clockCurrent - clockStart;

                    target_traj = 2*18.51*(sin((elapsed_time - countdown + zero_point)*pi/1.547)*sin((elapsed_time - countdown + zero_point)*pi/2.875));

                    if mod(k,2) == 0
                        dataR_rda = int8(read(ur_rda, 264, 'int8'));
                        eeg_data_vector = typecast(dataR_rda, 'single');
                        eeg_data = [eeg_data eeg_data_vector(1:33)' ,eeg_data_vector(34:66)'];
                    end

                    dataW =  typecast(single([1 target_traj]), 'int8');
                    fwrite(uw, dataW, 'int8');
                    
                    error_array = [error_array (target_traj + subject_traj)^2];
                
                    error_trial_array = [error_trial_array (target_traj + subject_traj)^2];
                    
                    Score = 100 - mean(error_trial_array)/3;

                    if Score < 0 
                        Score = 0;
                    end
                    
                    target_traj_array = [target_traj_array target_traj];
                    subject_traj_array = [subject_traj_array subject_traj];
                end
                k = k+1;
                flush(ur)
                
                
                data_box = [roundn(target_traj,-5) roundn(subject_traj,-5) roundn(Score,-5) roundn(trial_num, -5)];
                newV = typecast(single(data_box), 'int8');
                fwrite(u2, newV, 'int8')
            
            end
            error_trial_array = [];
            
            trial_name = strcat('trial',num2str(trial_num));
            hi5WristPos.(trial_name) = subject_traj_array;
            hi5TargetPos.(trial_name) = target_traj_array;
            hi5EEG.(trial_name) = eeg_data;
            hi5ZeroPoint.(trial_name) = zero_point;
            hi5Trigger.(trial_name) = trigger_array;

%             hi5Velocity.(trial_name) = velocity_array;
%             hi5Current.(trial_name) = current_array;
%             hi5Error.(trial_name) = Error_array;
            target_traj_array = [];
            subject_traj_array = [];
            eeg_data = [];
            trigger_array = [];
            disp(trial_num);
            trial_num = trial_num+1;
            disp('end of trial');
        end
        dataW =  typecast(single([5 0]), 'int8');
        fwrite(uw, dataW, 'int8');
            
        hi5Target_semiAssisted.hi5WristPos = hi5WristPos;
        hi5Target_semiAssisted.hi5TargetPos = hi5TargetPos;
%         hi5Target_semiAssisted.hi5Velocity = hi5Velocity;
%         hi5Target_semiAssisted.hi5Current = hi5Current;
%         hi5Target_semiAssisted.hi5Error = hi5Error;
        hi5Target_semiAssisted.hi5EEG = hi5EEG;
        hi5Target_semiAssisted.hi5ZeroPoint = hi5ZeroPoint;
        hi5Target_semiAssisted.hi5Trigger = hi5Trigger;
        save ('hi5Target_semiAssisted.mat','hi5Target_semiAssisted');
        
        Error = mean(error_array);
        Score = 100 - Error/3;
        
        if Score < 0 
            Score = 0;
        end
        
        Score_array = [Score_array Score];
        Master_score = mean(Score_array)
        fclose(uw);
        fclose(u2);
        
%------------------------Zero-assisted target tracking---------------------
    case 3 %hi5 zero-assisted target tracking track
        fopen(uw);
        fopen(u2);
        flush(ur)
        flush(ur_rda);
        
        dataW =  typecast(single([0 0]), 'int8');%set current to 0
        fwrite(uw, dataW, 'int8');
        
        dataR = int8(read(ur, 4, 'int8'));
        Zero_position = typecast(dataR, 'single');
        
        hi5Target_zeroAssisted = {};%cell aray for positional data
        hi5WristPos = {};%cell aray for positional data
        hi5TargetPos = {};
        hi5Velocity = {};
        hi5Current = {};
        hi5Error = {};
        hi5EEG = {};
        hi5ZeroPoint = {};
        hi5Trigger = {};
        target_traj_array = [];
        subject_traj_array = [];
        velocity_array = [];
        current_array = [];
        error_array = [];
        error_trial_array = [];
        eeg_data = [];
        trigger_array = [];
        currentPrev=0;
        current = 0;
        Error = 0;
        Score = 0;
        target_traj = 0;
        subject_traj = 0;
        
        trial_num = 1;
        
        zero_point_array = zeros(1,(total_trial_num));
        j = 1;
        for i = 1:(floor(((total_trial_num))/3)):(((total_trial_num)) - (floor(((total_trial_num))/3))+1)
            if j == 1
                zero_point_array(i:(i+(floor(((total_trial_num))/3))-1)) = 3.094;
            elseif j == 2
                zero_point_array(i:(i+(floor(((total_trial_num))/3))-1)) = 14.375;
            end
            j = j+1;
        end
        
        dataW =  typecast(single([4 0]), 'int8');%zero assistance
        fwrite(uw, dataW, 'int8');
        
        dataW =  typecast(single([7 0]), 'int8');%start trigger of task PIN 2
        fwrite(uw, dataW, 'int8');
        sequenceSave(exp_num)
        
        dataW =  typecast(single([4 0]), 'int8');%set current to 0
        fwrite(uw, dataW, 'int8');
        
        while(trial_num <= total_trial_num)
            
            trial_trigger_flag = 1;
            pause_value = 0;
            
            try
                pause = read(u_checkpause,1,'string');
                pause_string = split(pause,'');
                pause_value = str2double(pause_string(2));
                if pause_value == 1
                    disp('pause value:');
                    disp(pause_value);
                end
            catch
            end
            
            while pause_value
                try
                    pause = read(u_checkpause,1,'string');
                    pause_string = split(pause,'');
                    pause_value = str2double(pause_string(2));
                    if pause_value == 0
                        disp('pause value:');
                        disp(pause_value);
                    end
                catch
                end
            end
            
            
            c = clock;
            clockStart = c(4)*3600+c(5)*60+c(6);
            clockCurrent = clockStart;
            
            index = randi(length(zero_point_array));
            zero_point = zero_point_array(index);
            zero_point_array(index) = [];
            
            trial_name = strcat('trial',num2str(trial_num));
            
            flush(ur_rda);
            flush(ur);
            k = 1;
            while (clockCurrent < clockStart + trial_length + countdown)
                
                if clockCurrent < (clockStart + countdown)
                    
                    c = clock;
                    clockCurrent = c(4)*3600+c(5)*60+c(6);

                    dataR = int8(read(ur, 4, 'int8'));
                    subject_current_pos = typecast(dataR, 'single');

                    subject_traj = -(subject_current_pos).*90/6400;

                    % target position
                    elapsed_time = clockCurrent - clockStart;

                    if mod(k,2) == 0
                        dataR_rda = int8(read(ur_rda, 264, 'int8'));
                        eeg_data_vector = typecast(dataR_rda, 'single');
                        eeg_data = [eeg_data eeg_data_vector(1:33)' ,eeg_data_vector(34:66)'];
                    end

                    target_traj = 0;
                    dataW =  typecast(single([8 current]), 'int8');
                    fwrite(uw, dataW, 'int8');
                    
                    trigger_array = [trigger_array 0];
                    target_traj_array = [target_traj_array target_traj];
                    subject_traj_array = [subject_traj_array subject_traj];
                    
                else
                    dataW =  typecast(single([5 current]), 'int8');
                    fwrite(uw, dataW, 'int8');
                    
                    if trial_trigger_flag
                        dataW =  typecast(single([6 0]), 'int8');%start trigger of trial PIN 1
                        fwrite(uw, dataW, 'int8');
                        trial_trigger_flag = ~trial_trigger_flag;
                        trigger_array = [trigger_array 1];
                    else
                        trigger_array = [trigger_array 0];
                    end
                    
%                     while k <= 10
                        

                    c = clock;
                    clockCurrent = c(4)*3600+c(5)*60+c(6);

                    % subject position                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
                    dataR = int8(read(ur, 4, 'int8'));
                    subject_current_pos = typecast(dataR, 'single');

                    subject_traj = -(subject_current_pos).*90/6400;

                    % target position
                    elapsed_time = clockCurrent - clockStart;

                    target_traj = 2*18.51*(sin((elapsed_time - countdown + zero_point)*pi/1.547)*sin((elapsed_time - countdown + zero_point)*pi/2.875));


                    if mod(k,2) == 0
                        dataR_rda = int8(read(ur_rda, 264, 'int8'));
                        eeg_data_vector = typecast(dataR_rda, 'single');
                        eeg_data = [eeg_data eeg_data_vector(1:33)' ,eeg_data_vector(34:66)'];
                    end

                    dataW =  typecast(single([4 current]), 'int8');
                    fwrite(uw, dataW, 'int8');

                    
%                     flush(ur)
                    target_traj_array = [target_traj_array target_traj];
                    subject_traj_array = [subject_traj_array subject_traj];
                    
                    error_array = [error_array (target_traj + subject_traj)^2];
                    error_trial_array = [error_trial_array (target_traj + subject_traj)^2];
                    
                    Score = 100 - mean(error_trial_array)/3;
                    if Score < 0 
                        Score = 0;
                    end
                end
                k = k+1;
                flush(ur)
                
                
                
                
                data_box = [roundn(target_traj,-5) roundn(subject_traj,-5) roundn(Score,-5) roundn(trial_num, -5)];
                
%                 data_box = [roundn(target_traj_10_array,-5) roundn(subject_traj_10_array,-5) roundn(Score,-5) roundn(trial_num_10_array, -5)]
                newV = typecast(single(data_box), 'int8');
                fwrite(u2, newV, 'int8')
                
            end
            error_trial_array = [];
            dataW =  typecast(single([5 0]), 'int8');
            fwrite(uw, dataW, 'int8');
            
            hi5WristPos.(trial_name) = subject_traj_array;
            hi5TargetPos.(trial_name) = target_traj_array;
            hi5EEG.(trial_name) = eeg_data;
            hi5ZeroPoint.(trial_name) = zero_point;
            hi5Trigger.(trial_name) = trigger_array;
            target_traj_array = [];
            subject_traj_array = [];
            eeg_data = [];
            trigger_array = [];
            disp(trial_num);
            trial_num = trial_num+1;
        end
        hi5Target_zeroAssisted.hi5WristPos = hi5WristPos;
        hi5Target_zeroAssisted.hi5TargetPos = hi5TargetPos;
        hi5Target_zeroAssisted.hi5EEG = hi5EEG;
        hi5Target_zeroAssisted.hi5ZeroPoint = hi5ZeroPoint;
        hi5Target_zeroAssisted.hi5Trigger = hi5Trigger;
        
        save ('hi5Target_zeroAssisted.mat','hi5Target_zeroAssisted');
        
        Error = mean(error_array);
        Score = 100 - Error/3;
        if Score < 0 
            Score = 0;
        end
        Score_array = [Score_array Score];
        Master_score = mean(Score_array)
        fclose(u2)
        fclose(uw)
        
        
%--------------------------HI5 position track------------------------------
    case 4 %hi5 position track
        fopen(uw);
        fopen(u2);
        
        flush(ur)
        flush(ur_rda)
        
        dataW =  typecast(single([0 0]), 'int8');%set zero position
        fwrite(uw, dataW, 'int8');
        
        dataR = int8(read(ur, 4, 'int8'));
        Zero_position = typecast(dataR, 'single');
        
        hi5Position_Track = {};%cell aray for positional data
        hi5WristPos = {};%cell aray for positional data
        hi5TargetPos = {};
        hi5Velocity = {};
        hi5Current = {};
        hi5EEG = {};
        hi5Trigger = {};
        target_pos_10_array = zeros(1,10);
        elapsed_time_10_array = zeros(1,10);
        subject_traj_10_array = zeros(1,10);
        velocity_10_array = zeros(1,10);
        current_10_array = zeros(1,10);
        trial_num_10_array = zeros(1,10);
        target_pos_array = [];
        subject_traj_array = [];
        velocity_array = [];
        current_array = [];
        position_array = [];
        eeg_data = [];
        trigger_array = [];
        
        Error = 0;
        posFlag = 0;
        current = 0;
        trial_index = 1;
        
        block_num = 1;
        speeds=[0,1];
        
        dataW =  typecast(single([4 0]), 'int8');%zero assistance
        fwrite(uw, dataW, 'int8');
        
        dataW =  typecast(single([7 0]), 'int8');%start trigger of task PIN 2
        fwrite(uw, dataW, 'int8');
        sequenceSave(exp_num)
        
        dataW =  typecast(single([4 0]), 'int8');%set current to 0
        fwrite(uw, dataW, 'int8');
        
        flush(ur)
        while(block_num <= total_block_num)
            trial_num = 1;
            block_flag = 1;
            
            speed_index = randi(length(speeds));
            speedFlag =speeds(speed_index);
            speeds(speed_index) = [];
            disp('speed flag')
            speedFlag
            
            data_box = [roundn(0,-5) roundn(0,-5) roundn(0,-5) roundn(0,-5) roundn(speedFlag,-5)];
            newV = typecast(single(data_box), 'int8');
            fwrite(u2, newV, 'int8')
                    
            data_box = [];
            for i =1:1:total_trial_num/8 
                position_array = [position_array 40 20 -10 -20];
            end
            while(trial_num <= total_trial_num)
                trial_trigger_flag = 1;
                pause_value = 0;
            
                try
                    pause = read(u_checkpause,1,'string');
                    pause_string = split(pause,'');
                    pause_value = str2double(pause_string(2));
                    if pause_value == 1
                        disp('pause value:');
                        disp(pause_value);
                    end
                catch
                end

                while pause_value
                    try
                        pause = read(u_checkpause,1,'string');
                        pause_string = split(pause,'');
                        pause_value = str2double(pause_string(2));
                        if pause_value == 0
                            disp('pause value:');
                            disp(pause_value);
                        end
                    catch
                    end
                end

                flush(ur_rda);
                flush(ur);
                
                c = clock;
                clockStart = c(4)*3600+c(5)*60+c(6);
                clockCurrent = clockStart;
                executed = 0;
                k=1;
                while (clockCurrent < clockStart + trial_length)

                    if mod(trial_index, total_trial_num) == 1 && (block_flag == 1)
                        c = clock;
                        clock_count_down_start = c(4)*3600+c(5)*60+c(6);
                        clock_count_down_current = clock_count_down_start;

                        while clock_count_down_current < clock_count_down_start + 5
                            
%                             while k <= 10
                            c = clock;
                            clock_count_down_current = c(4)*3600+c(5)*60+c(6);
%                                 elapsed_time = clock_count_down_current - clock_count_down_start;
%                                 elapsed_time_10_array(k) = elapsed_time;

%                             trial_num_10_array(k) = trial_index;

                            % subject position
                            dataR = int8(read(ur, 4, 'int8'));
                            subject_current_pos = typecast(dataR, 'single');
                            subject_traj = -(subject_current_pos)*90/6400;
%                                 subject_traj_10_array(k) = subject_traj;


%                             if mod(k,2) == 0
%                                 dataR_rda = int8(read(ur_rda, 264, 'int8'));
%                                 eeg_data_vector = typecast(dataR_rda, 'single');
%                                 eeg_data = [eeg_data eeg_data_vector(1:33)' ,eeg_data_vector(34:66)'];
%                             end
%                             
%                             trigger_array = [trigger_array 0];%non_trigger_10_array];
%                             target_pos_array = [target_pos_array 0];%zeros(1,10)];
%                             subject_traj_array = [subject_traj_array subject_traj];%subject_traj_10_array];
                            
                            dataW =  typecast(single([4 current]), 'int8');
                            fwrite(uw, dataW, 'int8');
                            
                            data_box = [roundn(0.0,-5) roundn(subject_traj,-5) roundn(0.0,-5) roundn(trial_index, -5) roundn(speedFlag,-5)];
                            newV = typecast(single(data_box), 'int8');
                            fwrite(u2, newV, 'int8')
                            data_box = [];
                            
                            
                        end
%                         flush(ur)
%                         flush(ur_rda)
                        block_flag = ~block_flag;
                        c = clock;
                        clockCurrent = c(4)*3600+c(5)*60+c(6);
                        clockStart = clockCurrent;
                    else
                        if posFlag == 0 && executed ==0
                            index = randi(size(position_array));
                            target_pos = position_array(index);
                            position_array(index) = [];
                            executed = 1;
                        elseif posFlag == 1
                            target_pos = 0;
                        end
                        
                        if trial_trigger_flag
                            dataW =  typecast(single([6 0]), 'int8');%start trigger of trial PIN 1
                            fwrite(uw, dataW, 'int8');
                            trial_trigger_flag = ~trial_trigger_flag;
                            trigger_array = [trigger_array 1];
                        else
                            trigger_array = [trigger_array 0];
                        end


                        c = clock;
                        clockCurrent = c(4)*3600+c(5)*60+c(6);
                        elapsed_time = clockCurrent - clockStart;
%                         elapsed_time_10_array(k) = elapsed_time;

%                         trial_num_10_array(k) = trial_index;

                        % subject position
                        dataR = int8(read(ur, 4, 'int8'));
                        subject_current_pos = typecast(dataR, 'single');
                        subject_traj = -(subject_current_pos)*90/6400;
                        subject_traj_10_array(k) = subject_traj;

                        if mod(k,2) == 0
%                             disp('eeg')
                            dataR_rda = int8(read(ur_rda, 264, 'int8'));
                            eeg_data_vector = typecast(dataR_rda, 'single');
                            eeg_data = [eeg_data eeg_data_vector(1:33)' ,eeg_data_vector(34:66)'];
                        end
                        
                        dataW =  typecast(single([4 0]), 'int8');
                        fwrite(uw, dataW, 'int8');

                        data_box = [roundn(target_pos,-5) roundn(subject_traj,-5) roundn(Error,-5) roundn(trial_index, -5) roundn(speedFlag,-5)];
                        newV = typecast(single(data_box), 'int8');
                        fwrite(u2, newV, 'int8')
                        data_box = [];
    %                     velocity_array = [velocity_array velocity_10_array];
    %                     current_array = [current_array current_10_array];
                        target_pos_array = [target_pos_array target_pos];
                        subject_traj_array = [subject_traj_array subject_traj];
                    end
                    k=k+1;
                    flush(ur)
                    flush(ur_rda)
                end
                
                dataW =  typecast(single([5 0]), 'int8');%set current to 0
                fwrite(uw, dataW, 'int8');
                
                posFlag = ~posFlag;
                if speedFlag == 0
                    trial_name = strcat('Slow_trial',num2str(trial_index));
                elseif speedFlag == 1
                    trial_name = strcat('Fast_trial',num2str(trial_index));
                end
                
                hi5WristPos.(trial_name) = subject_traj_array;
                hi5TargetPos.(trial_name) = target_pos_array;
%                 hi5Velocity.(trial_name) = velocity_array;
%                 hi5Current.(trial_name) = current_array;
                hi5EEG.(trial_name) = eeg_data;
                hi5Trigger.(trial_name) = trigger_array;

                target_pos_array = [];
                subject_traj_array = [];
                velocity_array = [];
                current_array = [];
                eeg_data =[];
                trigger_array = [];
                disp(trial_num);
                trial_num = trial_num + 1;
                trial_index = trial_index + 1;
                
                disp('end of trial');
            end
            block_num = block_num + 1;
            disp(['Block number: ', num2str(block_num)]);
        end
        hi5Position_Track.hi5WristPos = hi5WristPos;
        hi5Position_Track.hi5TargetPos = hi5TargetPos;
%         hi5Position_Track.hi5Velocity = hi5Velocity;
%         hi5Position_Track.hi5Current = hi5Current;
        hi5Position_Track.hi5EEG = hi5EEG;
        hi5Position_Track.hi5Trigger = hi5Trigger;
        
        save ('hi5Position_Track.mat','hi5Position_Track');
        fclose(uw)
        fclose(u2)
        
        
%------------------------HI5 torque stablization---------------------------
    case 5 %torque stablization
        fopen(uw);
        fopen(u2);
        flush(ur)
        flush(ur_rda)
        
        dataW =  typecast(single([0 0]), 'int8');%set current to 0
        fwrite(uw, dataW, 'int8');
        
        dataR = int8(read(ur, 4, 'int8'));
        Zero_position = typecast(dataR, 'single');
        
        hi5Torque_Stablization = {};%cell aray for positional data
        hi5WristPos = {};%cell aray for positional data
        hi5TargetPos = {};
        hi5Velocity = {};
        hi5Current = {};
        hi5EEG = {};
        hi5Trigger = {};
        hi5ZeroPoint = {};
        target_strength_array = [];
        subject_traj_array = [];
        eeg_data = [];
        error_array = [];
        trigger_array = [];
        currentPrev=0;
        current=0;
        Score = 0;
        
        Error = 0;
        block_num = 1;
        trial_index = 1;
        
        strength_array = ones(1,(total_trial_num*total_block_num)-3);
        j = 1;
        for i = 1:(floor(((total_trial_num*total_block_num)-3)/3)):(((total_trial_num*total_block_num)-3) - (floor(((total_trial_num*total_block_num)-3)/3))+1)
            strength_array(i:(i+(floor(((total_trial_num*total_block_num)-3)/3))-1)) = j;
            j = j+1;
        end
        
        dataW =  typecast(single([7 0]), 'int8');%start trigger of task PIN 2
        fwrite(uw, dataW, 'int8');
        sequenceSave(exp_num)
        
        dataW =  typecast(single([4 0]), 'int8');%set current to 0
        fwrite(uw, dataW, 'int8');
        
        flush(ur);
        
        while(block_num <= total_block_num)
            trial_num = 1;
            block_flag = 1;
            while(trial_num <= total_trial_num)
                disp(['Trial index: ', num2str(trial_index)]);
                trial_trigger_flag = 1;
                pause_value = 0;
            
                try
                    pause = read(u_checkpause,1,'string');
                    pause_string = split(pause,'');
                    pause_value = str2double(pause_string(2));
                    if pause_value == 1
                        disp('pause value:');
                        disp(pause_value);
                    end
                catch
                end

                while pause_value
                    try
                        pause = read(u_checkpause,1,'string');
                        pause_string = split(pause,'');
                        pause_value = str2double(pause_string(2));
                        if pause_value == 0
                            disp('pause value:');
                            disp(pause_value);
                        end
                    catch
                    end
                end

                flush(ur_rda);
                flush(ur);
                
                c = clock;
                clockStart = c(4)*3600+c(5)*60+c(6);
                clockCurrent = clockStart;
                % strength = 1,2,3 for first three trials
                if trial_index == 1
                    strength = 1;
                elseif trial_index == 2
                    strength = 2;
                elseif trial_index == 3
                    strength = 3;
                elseif trial_index >= 4
                    index = randi(length(strength_array));
                    strength = strength_array(index);
                    strength_array(index) = [];
                end
                
                k = 1;
                
                while (clockCurrent < clockStart + trial_length)
                    
                    if mod(trial_index,total_trial_num) == 1 && (block_flag ==1)
                        c = clock;
                        clock_count_down_start = c(4)*3600+c(5)*60+c(6);
                        clock_count_down_current = clock_count_down_start;
                        
                        while clock_count_down_current < clock_count_down_start + 5
                            c = clock;
                            clock_count_down_current = c(4)*3600+c(5)*60+c(6);
                            
                            dataR = int8(read(ur, 4, 'int8'));
                            subject_position = typecast(dataR, 'single');
                            subject_traj = -(subject_position)*90/6400;
                            
%                             trigger_array = [trigger_array 0];
%                             target_strength_array = [target_strength_array 0];
%                             subject_traj_array = [subject_traj_array subject_traj];
                
                            
                            data_box = [roundn(0.0,-5) roundn(subject_traj,-5) roundn(Error,-5) roundn(trial_index, -5)];
                            newV = typecast(single(data_box), 'int8');
                            fwrite(u2, newV, 'int8')
                            
                        end
                        block_flag = ~block_flag;
                        c = clock;
                        clockCurrent = c(4)*3600+c(5)*60+c(6);
                        clockStart = clockCurrent;
                        flush(ur_rda);
                        flush(ur);
                    else
                        
                        if trial_trigger_flag
                            dataW =  typecast(single([6 0]), 'int8');%start trigger of trial PIN 1
                            fwrite(uw, dataW, 'int8');
                            trial_trigger_flag = ~trial_trigger_flag;
                            trigger_array = [trigger_array 1];
                        else
                            trigger_array = [trigger_array 0];
                        end
                        

                        c = clock;
                        clockCurrent = c(4)*3600+c(5)*60+c(6);
    %                     clockStart = clockCurrent;
                        elapsed_time = clockCurrent - clockStart;

                        if (elapsed_time > 4)
                            strength = 0;
                        end

                        switch strength
                            case 0
                                current = 0;
                            case 1
                                if (elapsed_time > 3.5)
                                    current=-2000*(2*(4-elapsed_time));% current *(0.5 second)*2, make the length within brackets = 1

                                elseif (elapsed_time > 0.5)
                                    current = -2000;
                                elseif (elapsed_time > 0)
                                    current=-2000*(2*(elapsed_time));
                                end
                            case 2
                                if (elapsed_time > 3.5)
                                    current=-3000*(2*(4-elapsed_time));% current *(0.5 second)*2, make the length within brackets = 1

                                elseif (elapsed_time > 0.5)
                                    current = -3000;
                                elseif (elapsed_time > 0)
                                    current=-3000*(2*(elapsed_time));
                                end
                            case 3
                                if (elapsed_time > 3.5)
                                    current=-4000*(2*(4-elapsed_time));% current *(0.5 second)*2, make the length within brackets = 1

                                elseif (elapsed_time > 0.5)
                                    current = -4000;
                                elseif (elapsed_time > 0)
                                    current=-4000*(2*(elapsed_time));
                                end
                        end

                        dataW =  typecast(single([3 current]), 'int8');
                        fwrite(uw, dataW, 'int8');

                        if mod(k,2) == 0
                            dataR_rda = int8(read(ur_rda, 264, 'int8'));
                            eeg_data_vector = typecast(dataR_rda, 'single');
                            eeg_data = [eeg_data eeg_data_vector(1:33)' ,eeg_data_vector(34:66)'];
                        end

                        dataR = int8(read(ur, 4, 'int8'));
                        subject_position = typecast(dataR, 'single');
                        subject_traj = -(subject_position)*90/6400;


%                         error_array = [error_array abs(subject_traj)];
%                         Error = mean(error_array);

                        
                        target_strength_array = [target_strength_array strength];
                        subject_traj_array = [subject_traj_array subject_traj];
                    
                        data_box = [roundn(strength,-5) roundn(subject_traj,-5) roundn(Error,-5) roundn(trial_index, -5)];
                        newV = typecast(single(data_box), 'int8');
                        fwrite(u2, newV, 'int8')
                    end
                    
                    k = k+1;
                    flush(ur)
                    
                    
                end
                trial_name = strcat('trial',num2str(trial_index));
                hi5WristPos.(trial_name) = subject_traj_array;
                hi5TargetPos.(trial_name) = target_strength_array;
%                 hi5Velocity.(trial_name) = velocity_array;
%                 hi5Current.(trial_name) = current_array;
                hi5EEG.(trial_name) = eeg_data;
                hi5Trigger.(trial_name) = trigger_array;
                target_strength_array = [];
                subject_traj_array = [];
                eeg_data = [];
                trigger_array = [];
%                 velocity_array = [];
%                 current_array = [];
                trial_num = trial_num + 1;
                trial_index = trial_index + 1;
                disp('end of trial');
            end
            dataW =  typecast(single([5 current]), 'int8');
            fwrite(uw, dataW, 'int8');
            block_num = block_num + 1;
            disp(['Block number: ', num2str(block_num)]);
        end
        hi5Torque_Stablization.hi5WristPos = hi5WristPos;
        hi5Torque_Stablization.hi5TargetPos = hi5TargetPos;
%         hi5Torque_Stablization.hi5Velocity = hi5Velocity;
%         hi5Torque_Stablization.hi5Current = hi5Current;
        hi5Torque_Stablization.hi5EEG = hi5EEG;
        hi5Torque_Stablization.hi5Trigger = hi5Trigger;
        
        save ('hi5Torque_Stablization.mat','hi5Torque_Stablization');
        fclose(uw)
        fclose(u2)

        
%------------------------Grip target tracking------------------------------
    case 6 %grip tracking
        fclose(u2)
        fopen(u2);
        fopen(uw);
        flush(u_force);
        flush(ur_rda);
        trial_length = 10;
        disp('Task 6: grip tracking')
        force_array = [];
        force_target_array = [];
        error_array = [];
        error_trial_array = [];
        elapsed_time = zeros(1,10);
        trial_num_10_array = zeros(1,10);
        force_target = [];
        eeg_data = [];
        trial_num = 1;
        gripTrack = {};
        gripforce = {};
        gripforce_target = {};
        gripforce_error = {};
        gripforce_eeg = {};
        gripZeroPoint = {};
        gripTrigger = {};
        trigger_array = [];
        Error = 0;
        Score = 0;
        
        zero_point_array = 10.058.*ones(1,(total_trial_num));
        j = 1;
        for i = 1:(floor(((total_trial_num))/3)):(((total_trial_num)) - (floor(((total_trial_num))/3))+1)
            if j == 1
                zero_point_array(i:(i+(floor(((total_trial_num))/3))-1)) = 24.085;
            elseif j == 2
                zero_point_array(i:(i+(floor(((total_trial_num))/3))-1)) = 30.172;
            end
            j = j+1;
        end
        
        dataW =  typecast(single([4 0]), 'int8');%zero assistance
        fwrite(uw, dataW, 'int8');
        
        dataW =  typecast(single([7 0]), 'int8');%start trigger of task PIN 2
        fwrite(uw, dataW, 'int8');
        sequenceSave(exp_num)
        dataW =  typecast(single([4 0]), 'int8');%zero assistance
        fwrite(uw, dataW, 'int8');
        
        while(trial_num <= total_trial_num)
            disp('start of trial');
            disp(trial_num);
            
            trial_trigger_flag = 1;
            
            pause_value = 0;
            
            try
                pause = read(u_checkpause,1,'string');
                pause_string = split(pause,'');
                pause_value = str2double(pause_string(2));
                if pause_value == 1
                    disp('pause value:');
                    disp(pause_value);
                end
            catch
            end
            
            while pause_value
                try
                    pause = read(u_checkpause,1,'string');
                    pause_string = split(pause,'');
                    pause_value = str2double(pause_string(2));
                    if pause_value == 0
                        disp('pause value:');
                        disp(pause_value);
                    end
                catch
                end
            end
            
            flush(ur_rda);
            
            c = clock;
            clockStart = c(4)*3600+c(5)*60+c(6);
            clockCurrent = clockStart;
        
            index = randi(length(zero_point_array));
            zero_point = zero_point_array(index);
            zero_point_array(index) = [];
            k = 1;
            while (clockCurrent < clockStart + trial_length + countdown)
                if clockCurrent < (clockStart + countdown) % countdown
%                     k = 1;
%                     while k <= 10
                    c = clock;
                    clockCurrent = c(4)*3600+c(5)*60+c(6);
%                         elapsed_time(k) = clockCurrent - clockStart;
%                     trial_num_10_array(k) = trial_num;
%                         [eeg_current, props] = get_eeg_modified(recorderip, con, stat, header_size, props);
%                         eeg_data = [eeg_data eeg_current];
                        
%                         k = k+1;
%                     end
                    if mod(k,2) == 0
                        dataR_rda = int8(read(ur_rda, 264, 'int8'));
                        eeg_data_vector = typecast(dataR_rda, 'single');
                        eeg_data = [eeg_data eeg_data_vector(1:33)' ,eeg_data_vector(34:66)'];
                    end
                    
%                     force_target = -[37.02 37.02 37.02 37.02 37.02 37.02 37.02 37.02 37.02 37.02];
                    force = read(u_force,1,'single');
                    
                    force_target = -37.02;
                    
                    trigger_array = [trigger_array 0];
                    force_array = [force_array force];
                    force_target_array = [force_target_array force_target];
                    
                    data_box = [roundn(force_target,-5) roundn(force,-5) roundn(Score,-5) roundn(trial_num, -5)];
                    

                    newV = typecast(single(data_box), 'int8');
                    fwrite(u2, newV, 'int8')
                elseif clockCurrent >= (clockStart + countdown) % in trial
                    
                    
                    if trial_trigger_flag
                        dataW =  typecast(single([6 0]), 'int8');%start trigger of trial PIN 1
                        fwrite(uw, dataW, 'int8');
%                         disp('Trial Trigger')
                        dataW =  typecast(single([4 0]), 'int8');%start trigger of trial PIN 1
                        fwrite(uw, dataW, 'int8');
                        trial_trigger_flag = ~trial_trigger_flag;
                        trigger_array = [trigger_array 1];
                    else
                        trigger_array = [trigger_array 0];
                    end
                    
%                     k = 1;
                    
%                     while k <= 10
                    c = clock;
                    clockCurrent = c(4)*3600+c(5)*60+c(6);
                    elapsed_time = clockCurrent - clockStart;
%                     trial_num_10_array(k) = trial_num;
                   
                    if mod(k,2) == 0
                        dataR_rda = int8(read(ur_rda, 264, 'int8'));
                        eeg_data_vector = typecast(dataR_rda, 'single');
                        eeg_data = [eeg_data eeg_data_vector(1:33)' ,eeg_data_vector(34:66)'];
                    end
                    
%                     end
                    force_target = 2.*18.51.*(sin((elapsed_time - countdown + zero_point).*pi/1.547).*sin((elapsed_time - countdown + zero_point).*pi/2.875));
                    force = read(u_force,1,'single');
                    force = force(1);
                    error_array = [error_array abs((mean(force_target - (force*2 - 30))))];
                    error_trial_array = [error_trial_array abs((mean(force_target - (force*2 - 30))))];
                    Score = 100 - 2*mean(error_trial_array);
                        
                    if Score < 0 
                        Score = 0;
                    end
                    
                    data_box = [roundn(force_target,-5) roundn(force,-5) roundn(Score,-5) roundn(trial_num, -5)];

                    newV = typecast(single(data_box), 'int8');
                    fwrite(u2, newV, 'int8')
                    force_array = [force_array force];
                    force_target_array = [force_target_array force_target];
                    force_target = [];
                    
                end
                if mod(k,10) == 0
                    flush(u_force)
                end
                k = k+1;
                
            end
            error_trial_array = [];
            force_name = strcat('trial',num2str(trial_num));
            gripforce.(force_name) = force_array;
            gripforce_target.(force_name) = force_target_array;
            gripforce_eeg.(force_name) = eeg_data;
            gripZeroPoint.(force_name) = zero_point;
            gripTrigger.(force_name) = trigger_array;
            force_target_array = [];
            force_array = [];
            eeg_data = [];
            trigger_array = [];
            trial_num = trial_num+1;
            disp('end of trial');
        end
        gripTrack.gripForce = gripforce;
        gripTrack.gripForceTarget = gripforce_target;
        gripTrack.EEG = gripforce_eeg;
        gripTrack.gripZeroPoint = gripZeroPoint;
        gripTrack.gripTrigger = gripTrigger;
%         gripTrack.gripforce_error = gripforce_error;
        save ('gripTrack.mat','gripTrack');
        
        Error = mean(error_array);
        Score = 100 - Error*2;
        
        if Score < 0 
            Score = 0;
        end
        
        Score_array = [Score_array Score];
%         disp('Your Final Score:');
        Master_score = mean(Score_array)
        
        fclose(u2)
        fclose(uw);
        
%------------------------Grip force maintain-------------------------------
    case 7 %grip force maintain
        fclose(u2)
        fopen(u2);
        fopen(uw);
        
        disp('Task 7: grip force maintain')
        force_array = [];
        force_target_array = [];
        error_array = [];
        error_trial_array = [];
        trigger_array = [];
        block_num = 1;
        gripForceMaintain = {};
        gripforce = {};
        gripforce_target = {};
        gripforce_eeg = {};
        gripforce_trigger = {};
        strength_10_array = zeros(1,10);
        elapsed_time_10_array = zeros(1,10);
        trial_num_10_array = zeros(1,10);
        Error = 0;
        Score = 0;
        
        strength_array = ones(1,(total_trial_num*total_block_num)-3);
        j = 1;
        for i = 1:(floor(((total_trial_num*total_block_num)-3)/3)):(((total_trial_num*total_block_num)-3) - (floor(((total_trial_num*total_block_num)-3)/3))+1)
            strength_array(i:(i+(floor(((total_trial_num*total_block_num)-3)/3))-1)) = j;
            j = j+1;
        end
        
        flush(u_force);
        flush(ur_rda)
        
        dataW =  typecast(single([4 0]), 'int8');%zero assistance
        fwrite(uw, dataW, 'int8');
        dataW =  typecast(single([7 0]), 'int8');%start trigger of task PIN 2
        fwrite(uw, dataW, 'int8');
        sequenceSave(exp_num)
        dataW =  typecast(single([4 0]), 'int8');%start trigger of trial PIN 1
        fwrite(uw, dataW, 'int8');
        
        trial_index = 1;
        while(block_num <= total_block_num)
            trial_num = 1;
            block_flag = 1;
            while(trial_num <= total_trial_num)
                trial_trigger_flag = 1;
                pause_value = 0;
            
                try
                    pause = read(u_checkpause,1,'string');
                    pause_string = split(pause,'');
                    pause_value = str2double(pause_string(2));
                    if pause_value == 1
                        disp('pause value:');
                        disp(pause_value);
                    end
                catch
                end

                while pause_value
                    try
                        pause = read(u_checkpause,1,'string');
                        pause_string = split(pause,'');
                        pause_value = str2double(pause_string(2));
                        if pause_value == 0
                            disp('pause value:');
                            disp(pause_value);
                        end
                    catch
                    end
                end

                flush(ur_rda);
                flush(ur);
                
                
                disp('start of trial');
                disp(['Trial index: ', num2str(trial_index)]);
                c = clock;
                clockStart = c(4)*3600+c(5)*60+c(6);
                clockCurrent = clockStart;
                
                if trial_index == 1
                    strength = 1;
                elseif trial_index == 2
                    strength = 2;
                elseif trial_index == 3
                    strength = 3;
                elseif trial_index >= 4
                    index = randi(length(strength_array));
                    strength = strength_array(index);
                    strength_array(index) = [];
                end
                k = 1;
                while (clockCurrent < clockStart + trial_length)
                    if mod(trial_index,total_trial_num) == 1 && (block_flag ==1)
                        c = clock;
                        clock_count_down_start = c(4)*3600+c(5)*60+c(6);
                        clock_count_down_current = clock_count_down_start;
                        
                        while clock_count_down_current < clock_count_down_start + 5
                            c = clock;
                            clock_count_down_current = c(4)*3600+c(5)*60+c(6);
                            force = read(u_force,1,'single');
                            
%                             trial_num_10_array = trial_index*ones(1,10);
                            
                            trigger_array = [trigger_array 0];
                            force_array = [force_array force];
                            force_target_array = [force_target_array 0];

                            data_box = [roundn(0,-5) roundn(force,-5) roundn(Score,-5) roundn(trial_index, -5)];
        %                     disp(data_box);
                            newV = typecast(single(data_box), 'int8');
                            fwrite(u2, newV, 'int8')
                        end
                        block_flag = ~block_flag;
                        c = clock;
                        clockCurrent = c(4)*3600+c(5)*60+c(6);
                        clockStart = clockCurrent;
                        eeg_data = [];
                        force_array = [];
                        force_target_array = [];
                        trigger_array = [];
                    else
                        
                        if trial_trigger_flag
                            dataW =  typecast(single([6 0]), 'int8');%start trigger of trial PIN 1
                            fwrite(uw, dataW, 'int8');
%                             pause(0.01)
                            dataW =  typecast(single([4 0]), 'int8');%start trigger of trial PIN 1
                            fwrite(uw, dataW, 'int8');
                            trial_trigger_flag = ~trial_trigger_flag;
                            trigger_array = [trigger_array 1];
                        else
                            trigger_array = [trigger_array 0];
                        end
                        
                        c = clock;
                        clockCurrent = c(4)*3600+c(5)*60+c(6);
                        if (clockCurrent > clockStart + 3)
                            strength = 0;
                        end

                        force = read(u_force,1,'single');

                        elapsed_time = clockCurrent - clockStart;
%                             strength_10_array(k) = strength;
%                         elapsed_time_10_array(k) = elapsed_time;
%                             trial_num_10_array(k) = trial_index;

                        if mod(k,2) == 0
                            dataR_rda = int8(read(ur_rda, 264, 'int8'));
                            eeg_data_vector = typecast(dataR_rda, 'single');
                            eeg_data = [eeg_data eeg_data_vector(1:33)' ,eeg_data_vector(34:66)'];
                        end

                        error_array = [error_array abs((mean(10*strength - force)))];
                        error_trial_array = [error_trial_array abs((mean(10*strength - force)))];
                        Score = 100 - mean(error_trial_array);

                        if Score < 0 
                            Score = 0;
                        end
                
                        data_box = [roundn(strength,-5) roundn(force,-5) roundn(Score,-5) roundn(trial_index, -5)];
    %                     disp(data_box);
                        newV = typecast(single(data_box), 'int8');
                        fwrite(u2, newV, 'int8')
                        force_array = [force_array force];
                        force_target_array = [force_target_array strength];
    %                     pause(0.002)
                    end
                    if mod(k,10) == 0
                        flush(u_force)
                    end
                    k = k+1;
                end
                error_trial_array = [];
                force_name = strcat('trial',num2str(trial_index));
                gripforce.(force_name) = force_array;
                gripforce_target.(force_name) = force_target_array;
                gripforce_eeg.(force_name) = eeg_data;
                gripforce_trigger.(force_name) = trigger_array;
                force_target_array = [];
                force_array = [];
                eeg_data = [];
                trigger_array = [];
                trial_num = trial_num + 1;
                trial_index = trial_index + 1;
                disp('end of trial');
            end
            block_num = block_num + 1;
            disp(['Block number: ', num2str(block_num)]);
        end
        gripForceMaintain.gripforce = gripforce;
        gripForceMaintain.gripforceTarget = gripforce_target;
        gripForceMaintain.EEG = gripforce_eeg;
        gripForceMaintain.gripforce_trigger = gripforce_trigger;
        save ('gripForceMaintain.mat','gripForceMaintain');
        
        Error = mean(error_array);
        Score = 100 - (Error^2)/3;
        
        if Score < 0 
            Score = 0;
        end
        
        Score_array = [Score_array Score];
%         disp('Your Final Score:');
        Master_score = mean(Score_array)
        
        fclose(u2)
        fclose(uw)
        
%------------------------Position training-------------------------------

    case 8 %hi5 position training
            fopen(uw);
            fopen(u2);

            flush(ur)
            
            flush(ur_rda);
            
            current = 0;
            
            dataW =  typecast(single([0 0]), 'int8');%set current to 0
            fwrite(uw, dataW, 'int8');

            
            dataR = int8(read(ur, 4, 'int8'));
            Zero_position = typecast(dataR, 'single');

            
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
            trial_length = 3;

            block_num = 1;
            dataW =  typecast(single([5 0]), 'int8');%set current to 0
            fwrite(uw, dataW, 'int8');
            flush(ur)
            while(block_num <= total_block_num)
                trial_num = 1;
                block_flag = 1;
                for i =1:1:total_trial_num/8 
                    position_array = [position_array 40 20 -10 -20];
                end
                while(trial_num <= total_trial_num)
                    
                    pause_value = 0;
            
                    try
                        pause = read(u_checkpause,1,'string');
                        pause_string = split(pause,'');
                        pause_value = str2double(pause_string(2));
                        if pause_value == 1
                            disp('pause value:');
                            disp(pause_value);
                        end
                    catch
                    end

                    while pause_value
                        try
                            pause = read(u_checkpause,1,'string');
                            pause_string = split(pause,'');
                            pause_value = str2double(pause_string(2));
                            if pause_value == 0
                                disp('pause value:');
                                disp(pause_value);
                            end
                        catch
                        end
                    end

                    flush(ur_rda);
                    flush(ur);
                    c = clock;
                    clockStart = c(4)*3600+c(5)*60+c(6);
                    clockCurrent = clockStart;
                    executed = 0;
                    executed2 = 0;

                    while (clockCurrent < clockStart + trial_length)
                        if mod(trial_index, total_trial_num) == 1 && (block_flag == 1)
                            c = clock;
                            clock_count_down_start = c(4)*3600+c(5)*60+c(6);
                            clock_count_down_current = clock_count_down_start;

                            while clock_count_down_current < clock_count_down_start + 5
%                                 k = 1;
%                                 while k <= 10
                                c = clock;
                                clock_count_down_current = c(4)*3600+c(5)*60+c(6);
                                elapsed_time = clock_count_down_current - clock_count_down_start;
%                                 elapsed_time_10_array(k) = elapsed_time;

                                % subject position
                                dataR = int8(read(ur, 4, 'int8'));
                                subject_current_pos = typecast(dataR, 'single');
                                subject_traj = -(subject_current_pos)*90/6400;
%                                 subject_traj_10_array(k) = subject_traj;

%                                 current_10_array(k) = 0;
%                                 k = k+1;
%                                 end
                                
                                dataW =  typecast(single([4 current]), 'int8');
                                fwrite(uw, dataW, 'int8');

%                                 data_box = [roundn(zeros(1,10),-5) roundn(subject_traj_10_array,-5) roundn(Error,-5) roundn(elapsed_time_10_array, -5)];
                                data_box = [roundn(0,-5),roundn(subject_traj,-5),roundn(Error,-5),roundn(elapsed_time,-5)];
                                newV = typecast(single(data_box), 'int8');
                                fwrite(u2, newV, 'int8')
                            end
                            block_flag = ~block_flag;
                            c = clock;
                            clockCurrent = c(4)*3600+c(5)*60+c(6);
                            clockStart = clockCurrent;

                        end
                            
                        
                        if posFlag == 0 && executed ==0
                            index = randi(size(position_array));
                            target_pos = position_array(index);
                            position_array(index) = [];
                            executed = 1;
                            sinPos=0;
                            treshold=target_pos;
                        elseif posFlag == 1 && executed2==0
                            target_pos = 0;
                            sinPos=treshold;
                            executed2=1;
                        end
%                         k = 1;
%                         while k <= 10

                            %target_pos_10_array(k) = target_pos;

                        c = clock;
                        clockCurrent = c(4)*3600+c(5)*60+c(6);
                        elapsed_time = clockCurrent - clockStart;
%                             elapsed_time_10_array(k) = elapsed_time;
                        disp(elapsed_time);
                        % target position
                        if target_pos==0
                            if abs(sinPos) > 0.25
                            sinPos=-(treshold+treshold/2*cos(1.5*block_num*((40/abs(treshold))^1.1)*elapsed_time)-treshold/2);
                            %sinPos=treshold+treshold/2*cos(2*sinTime)-treshold/2;
                            else
                                sinPos=0;
                            end
                        else
                            if abs(sinPos) < abs(treshold)-0.25
                            sinPos=treshold/2*cos(1.5*block_num*((40/abs(treshold))^1.1)*elapsed_time)-treshold/2;
                            %sinPos=treshold/2*cos(2*sinTime)-treshold/2;
                            else
                                sinPos=-target_pos;
                            end
                        end
%                         target_traj_10_array(k) = sinPos;

                        % subject position
                        dataR = int8(read(ur, 4, 'int8'));
                        subject_current_pos = typecast(dataR, 'single');
                        subject_traj = -(subject_current_pos)*90/6400;
%                         subject_traj_10_array(k) = subject_traj;

%                             current_10_array(k) = 0;
%                             k = k+1;
%                         end

%                         data_box = [roundn(target_traj_10_array,-5) roundn(subject_traj_10_array,-5) roundn(Error,-5) roundn(elapsed_time_10_array, -5)];
                        data_box = [roundn(sinPos,-5),roundn(subject_traj,-5),roundn(Error,-5),roundn(elapsed_time,-5)];
                        newV = typecast(single(data_box), 'int8');
                        fwrite(u2, newV, 'int8');
% 
% %                         velocity_array = [velocity_array velocity_10_array];
% %                         current_array = [current_array current_10_array];
%                         target_pos_array = [target_pos_array target_pos_10_array];
%                         subject_traj_array = [subject_traj_array subject_traj_10_array];
                    end
                    
                    dataW =  typecast(single([5 0]), 'int8');%set current to 0
                    fwrite(uw, dataW, 'int8');

                    posFlag = ~posFlag;
%                     if block_num == 1
%                         trial_name = strcat('Slow_trial',num2str(trial_index));
%                     elseif block_num == 2
%                         trial_name = strcat('Fast_trial',num2str(trial_index));
%                     else
%                         trial_name = strcat('trial',num2str(trial_index));
%                     end

%                     hi5WristPos.(trial_name) = subject_traj_array;
%                     hi5TargetPos.(trial_name) = target_pos_array;
%                     hi5Velocity.(trial_name) = velocity_array;
%                     hi5Current.(trial_name) = current_array;
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
            end
%             hi5Position_Track.hi5WristPos = hi5WristPos;
%             hi5Position_Track.hi5TargetPos = hi5TargetPos;
%             hi5Position_Track.hi5Velocity = hi5Velocity;
%             hi5Position_Track.hi5Current = hi5Current;
            %save ('hi5Position_Track.mat','hi5Position_Track');
            fclose(uw)
            fclose(u2)
        
%------------------------Combine all the Data------------------------------
    case 9 % save all the data in workspace
        load('hi5Target_fullAssisted.mat');
        load('hi5Target_semiAssisted.mat');
        load('hi5Target_zeroAssisted.mat');
        load('hi5Position_Track.mat');
        load('hi5Torque_Stablization.mat');
        load('gripTrack.mat');
        load('gripForceMaintain.mat');
        load('gripCalib.mat');
        load('username.mat');
        load('aaaSequence.mat');
        dt=string(datetime('now','TimeZone','local','Format','uuuu_MM_dd''T''HH_mm_ss'));
        expName=strcat('experimentData_',Username,'_',dt,'.mat');
        date=strcat('d',string(datetime('now','TimeZone','local','Format','uuuu_MM_dd')));
        trialsSequence = sequence.(Username).(date);
        save(expName,'hi5Target_fullAssisted','hi5Target_semiAssisted','hi5Target_zeroAssisted','hi5Position_Track','hi5Torque_Stablization','gripCalib','gripTrack','gripForceMaintain','trialsSequence')
%         save(expName,'hi5Target_semiAssisted','hi5Target_zeroAssisted','hi5Position_Track','hi5Torque_Stablization','gripTrack','gripForceMaintain')
        
        disp(strcat('Data was saved succesfully! The file name is'," ",expName));
        
%------------------------Grip Calibration------------------------------        
    case 10
        fclose(u2)
        fopen(u2);
        fopen(uw);
        flush(u_force);
        flush(ur_rda);
        trial_length = 5;
        disp('Task 10: grip Calibration')
        force_array = [];
        force_target_array = [];
        error_array = [];
        error_trial_array = [];
        force_target = 0;
        eeg_data = [];
        trial_num = 1;
        gripTrack = {};
        gripforce = {};
        gripforce_target = {};
        gripforce_error = {};
        gripforce_eeg = {};
        gripZeroPoint = {};
        gripTrigger = {};
        trigger_array = [];
        Error = 0;
        Score = 0;
        
        dataW =  typecast(single([4 0]), 'int8');%zero assistance
        fwrite(uw, dataW, 'int8');
        
        dataW =  typecast(single([7 0]), 'int8');%start trigger of task PIN 2
        fwrite(uw, dataW, 'int8');
        sequenceSave(exp_num)
        dataW =  typecast(single([4 0]), 'int8');%start trigger of trial PIN 1
        fwrite(uw, dataW, 'int8');
        
        while(trial_num <= total_trial_num)
            disp('start of trial');
            disp(trial_num);
            
            trial_trigger_flag = 1;
            
            pause_value = 0;
            
            try
                pause = read(u_checkpause,1,'string');
                pause_string = split(pause,'');
                pause_value = str2double(pause_string(2));
                if pause_value == 1
                    disp('pause value:');
                    disp(pause_value);
                end
            catch
            end
            
            while pause_value
                try
                    pause = read(u_checkpause,1,'string');
                    pause_string = split(pause,'');
                    pause_value = str2double(pause_string(2));
                    if pause_value == 0
                        disp('pause value:');
                        disp(pause_value);
                    end
                catch
                end
            end
            
            flush(ur_rda);
            
            c = clock;
            clockStart = c(4)*3600+c(5)*60+c(6);
            clockCurrent = clockStart;
        
            k = 1;
            while (clockCurrent < clockStart + trial_length + countdown)
                if clockCurrent < (clockStart + countdown) % countdown
%                     k = 1;
%                     while k <= 10
                    c = clock;
                    clockCurrent = c(4)*3600+c(5)*60+c(6);
                    
                    if mod(k,2) == 0
                        dataR_rda = int8(read(ur_rda, 264, 'int8'));
                        eeg_data_vector = typecast(dataR_rda, 'single');
                        eeg_data = [eeg_data eeg_data_vector(1:33)' ,eeg_data_vector(34:66)'];
                    end
                    
                    force = read(u_force,1,'single');
                    
                    trigger_array = [trigger_array 0];
                    force_array = [force_array force];
                    
                    data_box = [roundn(force_target,-5) roundn(force,-5) roundn(Score,-5) roundn(trial_num, -5)];
                    
                    newV = typecast(single(data_box), 'int8');
                    fwrite(u2, newV, 'int8')
                    
                elseif clockCurrent >= (clockStart + countdown) % in trial
                    
                    
                    if trial_trigger_flag
                        dataW =  typecast(single([6 0]), 'int8');%start trigger of trial PIN 1
                        fwrite(uw, dataW, 'int8');
                        dataW =  typecast(single([4 0]), 'int8');%start trigger of trial PIN 1
                        fwrite(uw, dataW, 'int8');
                        trial_trigger_flag = ~trial_trigger_flag;
                        trigger_array = [trigger_array 1];
                    else
                        trigger_array = [trigger_array 0];
                    end
                    
                    c = clock;
                    clockCurrent = c(4)*3600+c(5)*60+c(6);
                    elapsed_time = clockCurrent - clockStart;
                   
                    if mod(k,2) == 0
                        dataR_rda = int8(read(ur_rda, 264, 'int8'));
                        eeg_data_vector = typecast(dataR_rda, 'single');
                        eeg_data = [eeg_data eeg_data_vector(1:33)' ,eeg_data_vector(34:66)'];
                    end
                    
                    force = read(u_force,1,'single');
                    force = force(1);
                    
                    data_box = [roundn(force_target,-5) roundn(force,-5) roundn(Score,-5) roundn(trial_num, -5)];

                    newV = typecast(single(data_box), 'int8');
                    fwrite(u2, newV, 'int8')
                    force_array = [force_array force];
                    
                end
                if mod(k,10) == 0
                    flush(u_force)
                end
                k = k+1;
                
            end
            force_name = strcat('trial',num2str(trial_num));
            gripforce.(force_name) = force_array;
            gripforce_eeg.(force_name) = eeg_data;
            gripTrigger.(force_name) = trigger_array;
            force_target_array = [];
            force_array = [];
            eeg_data = [];
            trigger_array = [];
            trial_num = trial_num+1;
            disp('end of trial');
        end
        gripCalib.gripForce = gripforce;
        gripCalib.EEG = gripforce_eeg;
        gripCalib.gripTrigger = gripTrigger;
%         gripTrack.gripforce_error = gripforce_error;
        save ('gripCalib.mat','gripCalib');
        
        
        fclose(u2)
        fclose(uw);
end
end
