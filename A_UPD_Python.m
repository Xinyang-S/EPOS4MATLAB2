
clear u
%u4 = udpport("LocalPort",1000,'TimeOut',100);
u = udpport('LocalPort',1249);
%fopen(u);

while(1)
    flush(u)
    %app_data = read(u4,10,'string');
    A = read(u,7,'uint8')
    %A = uint8(A)
    %A = swapbytes(typecast(uint8([65 227 216 168]),'single'))
end

%fclose(u)