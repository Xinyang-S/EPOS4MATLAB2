clear u1
u1 = udpport("LocalPort",1263);
while(true)
  tic
  script_data = read(u1,1,"single")  % ensure buffer is multiple of number of electrodes used 
  toc
end

    