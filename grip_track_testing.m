% Change for individual recorder host
clear all; 
recorderip = '127.0.0.1';
eegall = [];
% Establish connection to BrainVision Recorder Software 32Bit RDA-Port
% (use 51234 to connect with 16Bit Port)
con = pnet('tcpconnect', recorderip, 51244);

% Check established connection and display a message
stat = pnet(con,'status');
if stat > 0
    disp('connection established');
end


% --- Main reading loop ---
header_size = 24;

%eeg init
 eeg_data = [];
 
props = [];

%%%%%%%%%%%%%%%%%%%%%
u_force = udpport('LocalPort',1250);
total_trial_num = 1;
        trial_length = 10;
        disp('Task: grip tracking')
        force_array = [];
        force_target_array = [];
        Error_array = [];
        %data_string = '';
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
            counter = 0;
            while (clockCurrent < clockStart + trial_length)
               counter = counter +1;
                k = 1;

                [eeg_current, props] = get_eeg(recorderip, con, stat, header_size, props);
                eeg_data = [eeg_data eeg_current];
%                 disp('eeg');
                while k <= 10
                    c = clock;
                    clockCurrent = c(4)*3600+c(5)*60+c(6);
                    elapsed_time(k) = clockCurrent - clockStart;
                    k = k+1;
                end
                force_target =2.*18.51.*(sin(elapsed_time.*pi/1.547).*sin(elapsed_time.*pi/2.875));
                flush(u_force)
                force = read(u_force,10,'single');
%                 ErrorSample = sqrt((force_target-force).^2);
%                 Error_array = [Error_array ErrorSample];
%                 Error = mean(Error_array);
                data_box = [roundn(force_target,-5) roundn(force,-5) roundn(Error,-5) roundn(elapsed_time, -5)];
                elapsed_time = [];
                
%                 write(u2,data_box,"double","LocalHost",4000);
                force_array = [force_array force];
                force_target_array = [force_target_array force_target];
                force_target = [];
                if size(eeg_data,2)~=size(force_array,2)
                    length_array = min(size(force_array,2),size(eeg_data,2));
                    eeg_data=eeg_data(:,1:length_array);
                    force_array = force_array(:,1:length_array);
                end
               
            end
            trial_num = trial_num+1;
            disp('end of trial');
            pause(5);% time for ready count down in AppDesigner
            
        end
        gripTrack.gripForce = gripforce;
        gripTrack.gripForceTarget = gripforce_target;
        gripTrack.gripforce_error = gripforce_error;
        gripTrack.eeg=eeg_data;
        save ('gripTrack.mat','gripTrack');