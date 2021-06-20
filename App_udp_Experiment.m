clear u1 u2 u3 u4
u4 = udpport("LocalPort",1000,'TimeOut',100);
u2 = udpport("LocalPort",3000); % open udp for FES pw from simulink, clear port if error
force_array = [];
force_target_array = [];
encoder_array = [];
velocity_array = [];
current_array = [];
data_string = [];
Error_array = [];


hi5Torque = {};

hi5_FA_traj = [];
hi5Target_fullAssisted = {};
hi5Target_FA_wristPos = {};
hi5Target_FA_targetPos = {};

hi5_SA_traj = [];
hi5Target_semiAssisted = {};
hi5Target_SA_wristPos = {};
hi5Target_SA_targetPos = {};

hi5Target_zeroAssisted = {};
hi5Target_ZA_wristPos = {};
hi5Target_ZA_targetPos = {};
hi5Position = {};

%% Run Python Code Here

% hi5 initialization
% Motor1 = Epos4(0,0);
% Motor1.ClearErrorState;
% Motor1.DisableNode;
% Motor1.SetOperationMode( OperationModes.CurrentMode );
% Motor1.EnableNode;
% Motor1.ClearErrorState;

while(1)
%target tracking trials 
% velocity = Motor1.ActualVelocity;
% velocity_array = [velocity_array velocity];
% current = Motor1.ActualCurrent;
% current_array = [current_array current];

flush(u4)
app_data = read(u4,10,'string');
app_data_string = split(app_data,'');
disp(app_data)
exp_num = str2num(app_data_string(2));
total_trial_num = str2num(cell2mat(strcat(app_data_string(3),app_data_string(4),app_data_string(5))));
trial_length = str2num(cell2mat(strcat(app_data_string(6),app_data_string(7),app_data_string(8))));
total_block_num = str2num(cell2mat(strcat(app_data_string(9),app_data_string(10),app_data_string(11))));

switch exp_num
    case 1 %hi5 full-assisted target tracking
        if trial_num ~= 0
            while trial_num ~= 0
                [exp_num, trial_num, traj] = read(u4,1,'string');
                hi5_FA_traj = [hi5_FA_traj traj];
                encoder = Motor1.ActualPosition;
                encoder_array = [encoder_array encoder];
            end
        elseif trial_num == 0
            full_assisted_name = strcat('trial',num2str(trial_num));
            hi5Target_FA_wristPos.(full_assisted_name) = encoder_array;
            hi5Target_FA_targetPos.(full_assisted_name) = hi5_FA_traj;
            hi5_FA_traj = [];
            encoder_array = [];
        end
    case 2 %hi5 semi-assisted target tracking
    case 3 %hi5 zero-assisted target tracking track
    case 4 %hi5 position track
    case 5 %torque stablization
    case 6 %grip tracking
        disp('Task: grip tracking')
        trial_num = 1;
        gripTrack = {};
        gripforce = {};
        gripforce_target = {};
        
        while(trial_num <= total_trial_num)
            c = clock;
            clockStart = c(4)*3600+c(5)*60+c(6);
            clockCurrent = clockStart;
            while (clockCurrent < clockStart + trial_length)
                c = clock;
                clockCurrent = c(4)*3600+c(5)*60+c(6);
                sinTime = clockCurrent - clockStart;
                force_target = 2*18.51*(sin(sinTime*pi/1.547)*sin(sinTime*pi/2.875));
                force = GetForce();
                
                force_display = force;
                ErrorSample = sqrt((force_target-force)^2);
                Error_array = [Error_array ErrorSample];
                Error = mean(Error_array);
                data_box = [roundn(force_target,-5) roundn(force_display,-5) roundn(Error,-5) roundn(sinTime, -5)];
                disp(data_box);
                for i = 1:(length(data_box))
                    data_string = [data_string uniform_data(data_box(i))];
                end
                write(u2,data_string,"string","LocalHost",4000);
                data_string = [];
                force_array = [force_array force];
                force_target_array = [force_target_array force_target];
            end
            force_name = strcat('trial',num2str(trial_num));
            gripforce.(force_name) = force_array;
            gripforce_target.(force_name) = force_target_array;
            force_target_array = [];
            force_array = [];
            disp(trial_num);
            trial_num = trial_num+1;
            disp('end of trial');
            pause(5);% time for ready count down in AppDesigner
        end
        gripTrack.gripForce = gripforce;
        gripTrack.gripForceTarget = gripforce_target;
        save ('gripTrack.mat','gripTrack');
        
    case 7 %grip force maintain
        disp('Task: grip force maintain')
        block_num = 1;
        gripForceMaintain = {};
        gripforce = {};
        gripforce_target = {};
        Error = 0;
        trial_index = 1;
        while(block_num <= total_block_num)
            trial_num = 1;
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
                    if (clockCurrent > clockStart + 3)
                        strength = 0;
                    end
                    c = clock;
                    clockCurrent = c(4)*3600+c(5)*60+c(6);
                    force = GetForce();
                    elapsed_time = clockCurrent - clockStart;
                    force_display = force;
                    data_box = [roundn(strength,-5) roundn(force_display,-5) roundn(Error,-5) roundn(elapsed_time, -5)];
                    disp(data_box);
                    for i = 1:(length(data_box))
                        data_string = [data_string uniform_data(data_box(i))];
                    end
                    write(u2,data_string,"string","LocalHost",4000);
                    data_string = [];
                    force_array = [force_array force];
                    force_target_array = [force_target_array strength];
                end
                force_name = strcat('trial',num2str(trial_index));
                gripforce.(force_name) = force_array;
                gripforce_target.(force_name) = force_target_array;
                force_target_array = [];
                force_array = [];
                trial_num = trial_num+1;
                trial_index = trial_index+1;
                disp(['end of trial']);
            end
            block_num = block_num + 1;
            disp(['Block number: ', num2str(block_num)]);
            pause(5);% time for ready count down in AppDesigner
        end
        gripForceMaintain.gripforce = gripforce;
        gripForceMaintain.gripforcetarget = gripforce_target;
        save ('gripForceMaintain.mat','gripForceMaintain');
        
    case 9 % save all the data in workspace
        hi5Targer_fullAssisted.WristPos = hi5Target_FA_wristPos;
        hi5Targer_fullAssisted.TatgetPos = hi5Target_FA_targetPos;
        save('hi5Targer_fullAssisted.mat','hi5Target_fullAssisted');
        hi5Targer_semiAssisted.WristPos = hi5Target_SA_wristPos;
        hi5Targer_semiAssisted.TatgetPos = hi5Target_SA_targetPos;
        save('hi5Targer_semiAssisted.mat','hi5Target_semiAssisted');        
        hi5Targer_zeroAssisted.WristPos = hi5Target_ZA_wristPos;
        hi5Targer_zeroAssisted.TatgetPos = hi5Target_ZA_targetPos;
        save('hi5Targer_zeroAssisted.mat','hi5Target_zeroAssisted'); 
        
        

end
end
