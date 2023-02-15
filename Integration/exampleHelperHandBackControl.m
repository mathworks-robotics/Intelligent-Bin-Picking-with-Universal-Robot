function exampleHelperHandBackControl() 
%This function is for internal use only and may be removed in the future.
% Hand Back the control to PolyScope  

%   Copyright 2023 The MathWorks, Inc.

client = rossvcclient('/ur_hardware_interface/hand_back_control');  
call(client);  
pause(0.5); 
end 
