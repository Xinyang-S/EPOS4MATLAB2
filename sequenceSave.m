function [] = sequenceSave(expNum)
switch expNum
    case 1
        expName='FullAssist';
    case 2
        expName='SemiAssist';
    case 3
        expName ='ZeroAssist';
    case 4
        expName='PosTrack';
    case 5
        expName='TorqueStab';
    case 6
        expName='GripTrack';
    case 7
        expName='GripMaintain';
    case 10
        expName='GripCalibration';
    otherwise
        expName='unknownName';
end

load('username.mat');
if isfile('aaaSequence.mat')
     load('aaaSequence.mat')
     
else
     sequence={};
     save('aaaSequence.mat','sequence')
end

if ~isfield(sequence,Username)
    sequence.(Username)=[];
end
date=strcat('d',string(datetime('now','TimeZone','local','Format','uuuu_MM_dd')));
time=strcat('t',string(datetime('now','TimeZone','local','Format','HH_mm_ss')));
expDes=strcat('expNum_',num2str(expNum),'_',expName);
sequence.(Username).(date).(time)=expDes;

save('aaaSequence.mat','sequence')
clear Username
clear sequence
end