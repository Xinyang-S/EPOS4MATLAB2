function cmd = generate_command(electroderef, amplitude, pulsewidth,testname,velecnumber)
    if nargin<5
        velecnumber = 5;
    end
    cmd="";
    cathodeslist = zeros(1,16);
    amplitudelist = zeros(1,16);
    pulsewidthlist = zeros(1,16);
    for i = 1:length(electroderef)
        if electroderef(i) >16
            fprintf('Electrode reference not recognized')
        end   
        if pulsewidth(i)>400
            fprintf('Please select a pulse width between 100 and 400 us')
        end 
        if amplitude(i)>20
            fprintf('Amplitude selected is too high, Change the settings if you really want to proceed')
        end   
        
        cathodeslist(electroderef(i))= 1;
        amplitudelist(electroderef(i))= amplitude(i);
        pulsewidthlist(electroderef(i))= pulsewidth(i);
    end

    if electroderef(i) == 2
            fprintf('Electrode number 2 => change the anode settings')
            activeanode = "*anode 32768";
            
    else
        activeanode = "*anode 2";
    end
    activatecathodes = strcat("*cathodes 1="+num2str(cathodeslist(1)) +",2="+ num2str(cathodeslist(2))+",3="+ num2str(cathodeslist(3))+",4="+ num2str(cathodeslist(4))+",5="+ num2str(cathodeslist(5))+",6="+num2str(cathodeslist(6)) +",7="+ num2str(cathodeslist(7))+",8="+ num2str(cathodeslist(8))+",9="+num2str(cathodeslist(9))+",10="+num2str(cathodeslist(10))+",11="+ num2str(cathodeslist(11))+",12="+ num2str(cathodeslist(12))+",13="+num2str(cathodeslist(13))+",14="+ num2str(cathodeslist(14))+",15="+num2str(cathodeslist(15)) +",16="+num2str(cathodeslist(16))+", " );        
    cathodesamplitudes = ("*amp 1="+num2str(amplitudelist(1)) +",2="+ num2str(amplitudelist(2))+",3="+ num2str(amplitudelist(3))+",4="+ num2str(amplitudelist(4))+",5="+ num2str(amplitudelist(5))+",6="+num2str(amplitudelist(6)) +",7="+ num2str(amplitudelist(7))+",8="+ num2str(amplitudelist(8))+",9="+num2str(amplitudelist(9))+",10="+num2str(amplitudelist(10))+",11="+ num2str(amplitudelist(11))+",12="+ num2str(amplitudelist(12))+",13="+num2str(amplitudelist(13))+",14="+ num2str(amplitudelist(14))+",15="+num2str(amplitudelist(15)) +",16="+num2str(amplitudelist(16))+", " );
    cathodespulsewidth = ("*width 1="+num2str(pulsewidthlist(1)) +",2="+ num2str(pulsewidthlist(2))+",3="+ num2str(pulsewidthlist(3))+",4="+ num2str(pulsewidthlist(4))+",5="+ num2str(pulsewidthlist(5))+",6="+num2str(pulsewidthlist(6)) +",7="+ num2str(pulsewidthlist(7))+",8="+ num2str(pulsewidthlist(8))+",9="+num2str(pulsewidthlist(9))+",10="+num2str(pulsewidthlist(10))+",11="+ num2str(pulsewidthlist(11))+",12="+ num2str(pulsewidthlist(12))+",13="+num2str(pulsewidthlist(13))+",14="+ num2str(pulsewidthlist(14))+",15="+num2str(pulsewidthlist(15)) +",16="+num2str(pulsewidthlist(16))+", " );
    cmd = ("velec "+num2str(velecnumber)+" *name "+testname+" *elec 1 "+activatecathodes+cathodesamplitudes+cathodespulsewidth+activeanode+" *selected 1 *sync 0"); %\r\n")
 
end