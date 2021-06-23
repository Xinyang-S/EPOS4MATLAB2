clear u1 
%echoudp('on',1250);
u1 = udp("LocalPort",1250); % open udp for FES pw from simulink, clear port if error
fopen(u1);
while(true)
    %flush(u1);
    tic
    fread(u1,1)  % ensure buffer is multiple of number of electrodes used 
    toc
end
echoudp('off')
fclose(u1)