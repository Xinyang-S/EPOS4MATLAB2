clear u1 u2 u3 u4
u4 = udpport("LocalPort",1000,'TimeOut',60);
u2 = udpport("LocalPort",3000); % open udp for FES pw from simulink, clear port if error
force_array = [];
force_target_array = [];
encoder_array = [];
velocity_array = [];
current_array = [];
data_string = [];
Error_array = [];

gripMaintain = {};
gripForce = {};
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

%for hi5 tracking traj = sinPos; for grip sensor traj = newton
%for hi5 position traj = position;
% flush(u4)
% disp(1)
flush(u4)
app_data = read(u4,10,'string');
app_data_string = split(app_data,'');
disp(app_data)
exp_num = str2num(app_data_string(2));
%trial_num = str2num(cell2mat(strcat(app_data_string(3),app_data_string(4),app_data_string(5))));
total_trial_num = str2num(cell2mat(strcat(app_data_string(6),app_data_string(7),app_data_string(8))));
trial_length = str2num(cell2mat(strcat(app_data_string(9),app_data_string(10),app_data_string(11))));

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
        if trial_num ~= 0
            while trial_num ~= 0
                [exp_num, trial_num, traj] = read(u4,1,'string');
                hi5_SA_traj = [hi5_SA_traj traj];
                encoder = Motor1.ActualPosition;
                encoder_array = [encoder_array encoder];
            end
        elseif trial_num == 0
            semi_assisted_name = strcat('trial',num2str(trial_num));
            hi5Target_SA_wristPos.(semi_assisted_name) = encoder_array;
            hi5Target_SA_targetPos.(semi_assisted_name) = hi5_SA_traj;
            hi5_SA_traj = [];
            encoder_array = [];
        end
    case 3 %hi5 zero-assisted target tracking
        if trial_num ~= 0
            while trial_num ~= 0
                [exp_num, trial_num, traj] = read(u4,1,'string');
                hi5_ZA_traj = [hi5_ZA_traj traj];
                encoder = Motor1.ActualPosition;
                encoder_array = [encoder_array encoder];
            end
        elseif trial_num == 0
            zero_assisted_name = strcat('trial',num2str(trial_num));
            hi5Target_ZA_wristPos.(zero_assisted_name) = encoder_array;
            hi5Target_ZA_targetPos.(zero_assisted_name) = hi5_ZA_traj;
            hi5_ZA_traj = [];
            encoder_array = [];
        end
    case 4 %hi5 position track
        
    case 5 %torque stablization
    case 6 %grip tracking
        disp('Task: grip tracking')
        %numTrials = 1; %max number of trials 
        trial_num = 1;
        while(trial_num <= total_trial_num)
            c = clock;
            clockStart = c(4)*3600+c(5)*60+c(6);
            clockCurrent = clockStart;
            while (clockCurrent < clockStart + trial_length)
                c = clock;
                clockCurrent = c(4)*3600+c(5)*60+c(6);
                sinTime = clockCurrent - clockStart;
                force_target = 2*18.51*(sin(sinTime*pi/1.547)*sin(sinTime*pi/2.875));
                blueY= 6.8*(force_target)+335;
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
                if clock_counter == 1
                    time_starttrial = clock;
                end
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
            disp('endoftrial');
            pause(5);% time for ready count down in AppDesigner
        end
        griptrack.gripforce = gripforce;
        griptrack.gripforcetarget = gripforce_target;
        save ('griptrack.mat','griptrack');
        
    case 7 %grip force maintain
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
        
        
        save('gripforce.mat','gripforce');

end
end
