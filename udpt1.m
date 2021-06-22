clear u1 u2 u3 u4
u3 = udpport("LocalPort",2000);
u1 = udpport("LocalPort",4000); % open udp for FES pw from simulink, clear port if error
f2 = [];
f1 = 0;
%write(u3, strcat(num2str(6), '0','0',num2str(1),'0','0','2'), 'string','LocalHost', 1000)
        
while(true)
    
%     write(u3,'asdfasdf',"string","LocalHost",1000);
%     f1 = read(u1,1,"double");  % ensure buffer is multiple of number of electrodes used 
%     f2 = [f2 f1];
        flush(u1);
        script_data = read(u1,1,"string");  % ensure buffer is multiple of number of electrodes used 
        script_data = split(script_data,'!');
        app.Label.Text = 'split';
        data_box = str2double(script_data(strlength(script_data)>0));
        app.Label.Text = num2str(data_box(10));
        f_targ = data_box(1:10);
        app.Label.Text = num2str(f_targ(10));
        f = data_box(11:20);
        Error = data_box(21);
        Elapsed_Time = data_box(22:31);
        app.Label.Text = num2str(Elapsed_Time(10));
end

    