clear u1 u2 u3 u4
u4 = udpport("LocalPort",1000);
u2 = udpport("LocalPort",3000); % open udp for FES pw from simulink, clear port if error
force_array = [];
encoder_array = [];
velocity_array = [];
current_array = [];

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
Motor1 = Epos4(0,0);
Motor1.ClearErrorState;
Motor1.DisableNode;
Motor1.SetOperationMode( OperationModes.CurrentMode );
Motor1.EnableNode;
Motor1.ClearErrorState;

while(1)
%target tracking trials 
velocity = Motor1.ActualVelocity;
velocity_array = [velocity_array velocity];
current = Motor1.ActualCurrent;
current_array = [current_array current];

%for hi5 tracking traj = sinPos; for grip sensor traj = newton
%for hi5 position traj = position;
while (true)
    flush(u4)
    read(u4,1,'double')
end


switch exp_num
    case '1'%hi5 full-assisted target tracking
        if trail_num ~= 0
            while trail_num ~= 0
                [exp_num, trial_num, traj] = read(u4,1,'double');
                hi5_FA_traj = [hi5_FA_traj traj];
                encoder = Motor1.ActualPosition;
                encoder_array = [encoder_array encoder];
            end
        elseif trail_num == 0
            full_assisted_name = strcat('trail',num2str(trail_num));
            hi5Target_FA_wristPos.(full_assisted_name) = encoder_array;
            hi5Target_FA_targetPos.(full_assisted_name) = hi5_FA_traj;
            hi5_FA_traj = [];
            encoder_array = [];
        end
    case '2'%hi5 semi-assisted target tracking
        if trail_num ~= 0
            while trail_num ~= 0
                [exp_num, trial_num, traj] = read(u4,1,'double');
                hi5_SA_traj = [hi5_SA_traj traj];
                encoder = Motor1.ActualPosition;
                encoder_array = [encoder_array encoder];
            end
        elseif trail_num == 0
            semi_assisted_name = strcat('trail',num2str(trail_num));
            hi5Target_SA_wristPos.(semi_assisted_name) = encoder_array;
            hi5Target_SA_targetPos.(semi_assisted_name) = hi5_SA_traj;
            hi5_SA_traj = [];
            encoder_array = [];
        end
    case '3'%hi5 zero-assisted target tracking
        if trail_num ~= 0
            while trail_num ~= 0
                [exp_num, trial_num, traj] = read(u4,1,'double');
                hi5_ZA_traj = [hi5_ZA_traj traj];
                encoder = Motor1.ActualPosition;
                encoder_array = [encoder_array encoder];
            end
        elseif trail_num == 0
            zero_assisted_name = strcat('trail',num2str(trail_num));
            hi5Target_ZA_wristPos.(zero_assisted_name) = encoder_array;
            hi5Target_ZA_targetPos.(zero_assisted_name) = hi5_ZA_traj;
            hi5_ZA_traj = [];
            encoder_array = [];
        end        
    case '4'%hi5 position track
        
    case '5'%torque stablization
    case '6'% grip tracking
        if trial_num ~= 0
            force = Getforce();
            if (~isempty(force))
                write(u2,[force encoder],"double","LocalHost",4000);
            end
            force_array = [force_array ; force];
        elseif trial_num == 0
            force_name = strcat('trial',num2str(trial_num));
            gripforce.(force_name) = force_array;
            force_array = [];
        end
    case '7'%grip force maintain
    case '9'% save all the data in workspace
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
