%% This script contains the parameters used in the project modules' perception, motion planning, and integration.
% This script runs at the time of project initialization. Change this
% parameters accordingly if you are using any different setup.

%   Copyright 2023 The MathWorks, Inc.

%% Perception parameters

% YOLOv4 parameters
yoyloScoreTh = 0.8;% YOLO score threshold. if 0, all detection are accepted
gridDownsample = 0.001;% Measured point downsample gridsize
PlanrThickness = 0.02;
normalvector = [0 0 1];
maxPlaneTilt = 5; % in degrees


%% Motion planning parameters

% home position
homePosition = deg2rad([-15+180 -126 113 -80 -91 76]);

% Bin and part dimensions
binLength = 0.38; % Along X axis
binWidth = 0.57; % Along Y axis
binHeight = 0.11;
binCenterPosition = [0.48 0 -0.09+0.11/2];
binRotation = 0;

% cuboid object dimensions
partheight = 0.0508;
partWidth = 0.0508;


% Transformation matrix: From camera to the world reference frame
rotationFromRobotToCam = eul2rotm([pi/2, pi, pi],"ZYZ");
posDepthCam = [0.54 -0.018 0.63];


% Set max acceleration and velocity limits for motion planning with
% hardware
maxqd = pi/2; %rad / s
maxqdd = deg2rad(100); % rad / s2

vellimits = repmat([-maxqd; maxqd],1,6);
accellimits  = repmat([-maxqdd; maxqdd],1,6);


% Position threshold for IK
poseThr = 1e-3;


%% Open introductory script
open("SmartBinPickingWithUR5eStartScript.mlx");
