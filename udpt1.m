clear u1 u2 u3 u4
u3 = udpport("LocalPort",2000);
u1 = udpport("LocalPort",4000); % open udp for FES pw from simulink, clear port if error
f2 = [];
f1 = 0;
write(u3, strcat(num2str(6), '0','0',num2str(1)), 'string','LocalHost', 1000)
        
while(true)
    
%     write(u3,'asdfasdf',"string","LocalHost",1000);
%     f1 = read(u1,1,"double");  % ensure buffer is multiple of number of electrodes used 
%     f2 = [f2 f1];
script_data = read(u1,24,"string")  % ensure buffer is multiple of number of electrodes used 
script_data = split(script_data,'');
f_targ = str2num(strcat(script_data(2),script_data(3),...
    script_data(4),script_data(5),script_data(6),script_data(7),script_data(8),script_data(9))) %force target
f = str2num(strcat(script_data(10),script_data(11),script_data(12),...
    script_data(13),script_data(14),script_data(15),script_data(16),...
    script_data(17))) %force of the human
error = str2num(strcat(script_data(18),script_data(19),script_data(20),...
    script_data(21),script_data(22),script_data(23),script_data(24),...
    script_data(25)))%error between target force and human force
end

    