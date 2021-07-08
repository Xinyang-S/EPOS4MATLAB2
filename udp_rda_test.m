clear u1 u2 u3 u4 u_force ur ur_rda
% fclose(uw_rda);
port_rda = 6666; 
uw_rda = udp('LocalHost', port_rda+2,'timeout',100);
% ur_rda = udpport('LocalPort', port_rda+1);

fopen(uw_rda);

% while 1
%     toc
%     tic
%     dataR_rda = int8(read(ur_rda, 264, 'int8'));
%     
%     eeg_data_vector = typecast(dataR_rda, 'single');
%     eeg_data = [eeg_data_vector(1:33)' ,eeg_data_vector(34:66)']
% end
EEGData = zeros(1,66);
tic
while 1
    EEGData_vector = [EEGData(1,1:33), EEGData(1,34:66)];
    toc
    tic
    dataW =  typecast(single(EEGData_vector), 'int8');
    fwrite(uw_rda, dataW, 'int8');
    pause(0.015)
end

fclose(uw_rda);