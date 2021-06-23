clear u1
u1 = udpport("LocalPort",3000);
while(true)
  flush(u1);
  script_data = int8(read(u1,4,"int8"));  % ensure buffer is multiple of number of electrodes used 
  clock3 = typecast(script_data,'single')
end

    