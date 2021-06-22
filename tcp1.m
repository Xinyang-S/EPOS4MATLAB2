

t = tcpclient("144.212.130.17",80,"Timeout",20)

data = sin(1:64);
%plot(data);
while 1
    tic
    write(t,data,"double")
    toc
end