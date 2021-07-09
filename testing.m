load('aaaSequence.mat')
load('username.mat')
date=strcat('d',string(datetime('now','TimeZone','local','Format','uuuu_MM_dd')));
trialsSequence=sequence.(Username).(date);
