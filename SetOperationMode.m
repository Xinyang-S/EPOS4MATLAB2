function res = SetOperationMode(handle, nodeid, OpMode)
% This function set the operation mode for EPOS 4 idetifyed with 'nodeid'
% Call it as,
%
% res = SetOperationMode(handle, nodeid, OpMode)
%
% Where:
%
% res is a number, 0 (zero) if it works, a negative number if not
%
% handle is the handle returned by OpenCommunication
%
% nodeid is the id number for EPOS 4 controller
%
% OpMOde is a number greater than 0 and lower than 7.
%    1   set node to Homming Mode.
%    2   set node to Current Mode.
%    3   set node to Velocity Mode.
%    4   set node to Position Mode.
%    5   set node to Profile Velocity Mode.
%    6   set node to Profile Position Mode.
% 
% 
% Operation 
% mode Description
% 1 Profile Position Mode (PPM)
% 3 Profile Velocity Mode (PVM)
% 6 Homing Mode (HMM)
% 8 Cyclic Synchronous Position Mode (CSP)
% 9 Cyclic Synchronous Velocity Mode (CSV)
% 10 Cyclic Synchronous Torque Mode (CST)

% Copyright E. Yime, 2015.
% Colombia
%