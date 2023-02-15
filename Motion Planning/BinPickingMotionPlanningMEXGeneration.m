%% Generate Code for Manipulator Motion Planning for Bin Picking Application in Perceived Environment 
% 
% Overview
% In this script, you will generate a MEX function using C/C++ code generation 
% for the motion planning purpose of the bin picking application. The motion planning 
% algorithm is given in this example < add rbt sim example> is used to create 
% a MEX function. 
% 
% Generating a MEX function using C/C++ code generation helps to reduce the 
% computation time and hence reduce the pick and place cycle time.
% 
% For more information on MEX function creation for the manipulatorRRT algorithm-based 
% planner, you can refer to <https://in.mathworks.com/help/robotics/ug/generate-code-for-manipulator-motion-planning-in-perceived-environment.html 
% this> example.
% 
% Also, refer to <https://in.mathworks.com/help/coder/gs/generating-mex-functions-from-matlab-code-at-the-command-line.html 
% this> example to know more about how to generate MEX function to accelerate 
% your MATLAB program execution. 
%% Design Planner Entry Point Function for MEX generation
% The planner entry point function is designed from the algorithm used in this<attach 
% link to the RBT simulation example> example. This algorithm uses <https://in.mathworks.com/help/robotics/ref/manipulatorrrt.html 
% |manipulatorRRT|> to plan an approach, and pick and place trajectories in the 
% collision environment.
% 
% The function |exampleHelperBinPickingRRTMotionPlanner| is designed in a way 
% that it takes input as a goal configuration and a path type, and outputs the 
% computed path. Detailed information about the inputs and outputs is given below.
% 
% Note that bin and part dimensions have been added in the same function |exampleHelperBinPickingRRTMotionPlanner| 
% and used to create a collision environment for the planner. *If you are using 
% any different bin or parts then change those parameters accordingly.* 
% 
% 
% 
% *Inputs:*
%% 
% # |partID|: Index of the part for pick and place from the detected object's 
% |partGT| array
% # |partGT|: Computed object location and pose using perception pipeline
% # |startConfig:| Cobot start configuration in joint space from which trajectory 
% needs to start
% # |endConfig|: Cobot end configuration in joint space upto which trajectory 
% needs to end
% # |pathType|: The path type, either 'place' or 'approach'
%% 
% *Outputs:*
% 
% As an output, this function provides three types of the path as described 
% below.
%% 
% # |planPath|: Computed path or waypoints using
% # |shortPath|: Computed shortened path with trimmed edges
% # |interpolatePath|: Computed interpolated states along the path
%% Setup Planner Parameters for Reference Input for Code Generation
% Loading dummy parameters to set as input for code generation command. As mentioned 
% in the previous section, this function requires |partID,partGT,startConfig,endConfig| 
% and |pathType| as input arguments.

%% create PartGT array
load("GroundTruth.mat");
PartGTLoaded = partGT;
numberOfParts = size(PartGTLoaded,1);
partGT = zeros(numberOfParts,4);

for i=1:numberOfParts
    partGT(i,1:2) = PartGTLoaded(i,1:2);
    partGT(i,3) = 0.01;
    partGT(i,4) = PartGTLoaded(i,3);
end

%% Specify part index to approach or place
partID = 2;

%% Specify start and end configurations as 1x6 array
startConfig = deg2rad([-15 -126 113 -80 -91 76]);
endConfig = [-0.1922 -1.4101 1.8234 -1.9841 -1.5708 1.3786];

%% Specify the path type for which the trajectory needs to generate
pathType = 'approach';
%% Generate Code for Planning
% To generate a MEX function, exampleHelperBinPickingRRTMotionPlanner|_mex|, 
% from the MATLAB function |exampleHelperBinPickingRRTMotionPlanner.m| , run this 
% command:
%%
% 
%   codegen("exampleHelperBinPickingRRTMotionPlanner", ...
%       "-args",{partID,coder.typeof(partGT,[inf 4],[1 0]),startConfig,endConfig,coder.typeof(pathType,[1 inf],[0 1])})
%
%% 
% The |variable_dims| argument of <docid:fixedpoint_ref#bsu_35x |coder.typeof|> 
% specifies that the input 2D array |partGT| of the entry-point planning function 
% can have an unbounded and variable-sized second dimension.    
% 
% We will use this MEX function in the integrated script of the bin-picking 
% workflow.
% 
% 

% Output from MATLAB function
[planPath, shortPath, interpolatePath] = exampleHelperBinPickingRRTMotionPlanner(partID,partGT,startConfig,endConfig,pathType);
%% 
% 
% 
% Run this command to use the generated MEX function for the computation of 
% the paths for specified inputs,
%%
% 
%   % Output from MEX function
%   [planPath, shortPath, interpolatePath] = exampleHelperBinPickingRRTMotionPlanner_mex(partID,partGT,startConfig,endConfig,pathType);
%
%% 
% 
%% Supporting Functions
% Planner Entry Point Function
% The function given below is an entry point function for the motion planning 
% logic for bin picking application. Change the bin dimension according to your 
% environment in this function before generating a code.
%%
% 
%   function [planPath, shortPath, interpolatePath]  =  exampleHelperBinPickingRRTMotionPlanner(partID,partGT,startConfig,endConfig,pathType) %#codegen
%   %This function is for internal use only and may be removed in the future.
%   %exampleHelperBinPickingRRTMotionPlanner is a motion planner function in a collision environment
%   % for Universal Robots UR5e a robot
%   %   [PLANPATH, SHORTPATH, INTERPOLATEPATH]  =  exampleHelperBinPickingRRTMotionPlanner(PARTID,PARTGT,STARTCONFIG,ENDCONFIG,PATHTYPE)
%   %
%   %   OUTPUT:
%   %   Outputs a collision-free geometric plan and interpolated path,
%   %
%   %   INPUT:
%   %   PARTID is the index of the object to pick.
%   %   PARTGT is the array representing the part CenterPoint and its orientation
%   %   with respect to the vertical Z-axis.
%   %   START and GOAL are the start and goal joint configurations of the
%   %   robot, respectively, and are specified as a row vector.
%   %   PATHTYPE is the type of the path as 'approach' and 'place' in a char array.
%   %
%   %   All the dimensions are in meters.
%   
%   %Copyright 2023 The MathWorks, Inc.
%   
%   % Import Universal Robots UR5e as a rigid body tree model
%   rbt = importrobot('universalUR5e.urdf','MaxNumBodies',22,'DataFormat','row');
%   
%   % Attach Robotiq Epick suction gripper body as a fixed joint
%   % at the end-effector of the robot
%   ur5e = exampleHelperAddGripper(rbt);
%   tformZYX = eul2tform([0 0 0]);
%   setFixedTransform(ur5e.Base.Children{1,2}.Joint,tformZYX);
%   
%   % Bin dimensions in meters (Used to create a collision environment
%   binLength = 0.395; % Along X axis
%   binWidth = 0.585; % Along Y axis
%   binHeight = 0.11;
%   binCenterPosition = [0.48 0 -0.038];
%   binRotation = 0;
%   
%   % Radius to switch planner to optimize computations
%   planRadius1 = 0.18;
%   planRadius2 = 0.21;
%   
%   % Set planner parameters according to the planer radius for the approach
%   % and place trajectory
%   if string(pathType) == "approach"
%       % Distance between the part center point and bin center point
%       dist = pdist([partGT(partID,1:2);binCenterPosition(1:2)]);
%   
%       % Set MaxConnectionDistance and ValidationDistance properties of the
%       % planner according to the plan radius
%       if dist < planRadius1
%           MaxConnectionDistance = 0.05;
%           ValidationDistance = MaxConnectionDistance/4;
%       elseif dist >  planRadius2
%           MaxConnectionDistance = 0.15;
%           ValidationDistance = MaxConnectionDistance/2;
%       else
%           MaxConnectionDistance = 0.1;
%           ValidationDistance = MaxConnectionDistance/3;
%       end
%   else
%       MaxConnectionDistance = 0.1;
%       ValidationDistance = 0.1;
%   end
%   
%   % Modify the robot by adding a part as a collision box with the end-effector
%   % for place type of trajectory to avoid part collision with the environment
%   % while placing it at the destination.
%   if string(pathType) == "place"
%       % Find end effector transform
%       endEffectorTransform = getTransform(ur5e,startConfig,ur5e.BodyNames{end});
%       eulerTransform = tform2eul(endEffectorTransform);
%   
%       % Modify the robot rigid body tree by adding part at the end-effector
%       ur5e = exampleHelperAttachPartAtEndEffector(ur5e,-deg2rad(partGT(partID,4))+eulerTransform(1)+pi);
%   end
%   
%   %% Start Planning
%   numOfParts = size(partGT,1); % Number of parts
%   
%   % Create a collision environment for the planner
%   env = exampleHelperGenerateCollisionEnviornment(binLength, binWidth, binHeight, binCenterPosition, binRotation,numOfParts,partGT);
%   
%   index = false(1,length(env));
%   index(7+partID) = true;
%   
%   env(index) = [];
%   
%   % Create and set up the planner from the rigid body tree and
%   % environment.
%   planner = manipulatorRRT(ur5e,env);
%   planner.MaxConnectionDistance = MaxConnectionDistance;
%   planner.ValidationDistance = ValidationDistance;
%   planner.SkippedSelfCollisions="parent";
%   
%   % Compute the planned path for the given start and end configuration
%   % using the planner
%   path = plan(planner,startConfig,endConfig);
%   
%   % Compute short and interpolated path form the computed path
%   dist = pdist([partGT(partID,1:2);binCenterPosition(1:2)]);
%   if dist < planRadius1
%       shortFlag = false;
%   else
%       shortFlag = true;
%   end
%   
%   if shortFlag
%       numIteration = 20;
%       shortPath = shorten(planner,path,numIteration);
%       interpConfigurations = interpolate(planner,shortPath);
%   else
%       shortPath = path;
%       interpConfigurations = interpolate(planner,shortPath);
%   end
%   
%   planPath = path;
%   interpolatePath = interpConfigurations;
%   
%   %% Helper functions
%       function ur5e = exampleHelperAttachPartAtEndEffector(ur5e,rotation)
%           %This function is for internal use only and may be removed in the future.
%           % This helper function is used to attach a part body to the
%           % end-effector to update the rigid body tree for collision-free
%           % trajectory in the given environment
%   
%           %   Copyright 2021 The MathWorks, Inc.
%   
%           % Part Dimensions in meters
%           partwidth = 0.0508;
%           partheight = 0.0508;
%           partLength = 0.1016;
%   
%           % Computed transformation matrix for adding collision object
%           transformForCollision = eul2tform([rotation+pi/2 0 0]);
%           transformForCollision(:,4) = [0; 0; partheight/2-0.01; 1];
%   
%           % Attach collision box to the rigid body model
%           part = rigidBody('part','MaxNumCollisions',1);
%           box = [partLength partwidth partheight];
%           addCollision(part,'box',box,transformForCollision);
%   
%           % Computed transformation matrix for adding fixed joint for object
%           transformPart = eul2tform([0 0 0]);
%           transformPart(:,4) = [0; 0; 0.005; 1]; % To avoid self collision add 0.005m offset
%   
%           % Create a fixed joint and attach it to the robot end-effector body
%           partJoint = rigidBodyJoint('partJoint','fixed');
%           part.Joint = partJoint;
%           setFixedTransform(part.Joint, transformPart);
%           curEndEffectorBodyName = ur5e.BodyNames{end};
%           addBody(ur5e,part,curEndEffectorBodyName);
%       end
%   
%   
%       function env = exampleHelperGenerateCollisionEnviornment(binLength, binWidth, binHeight, binCenterPosition, binRotation,numberOfParts,partGT)
%           %This function is for internal use only and may be removed in the future.
%           % This helper function generates a collision environment array with
%           % the bin, parts, and the camera
%   
%           %   Copyright 2021 The MathWorks, Inc.
%   
%           % Add CollisionBox face 1 (Thickness 0.001 assumed)
%           b1 = collisionBox(binLength,0.001,binHeight);
%           b1.Pose(1:3,1:3) = eul2rotm([0 binRotation*pi/180 0]);
%           b1.Pose(1:3,4) = binCenterPosition+[0 binWidth/2 0];
%   
%           % Add CollisionBox face 2 (Thickness 0.001 assumed)
%           b2 = collisionBox(binLength,0.001,binHeight);
%           b2.Pose(1:3,1:3) = eul2rotm([0 binRotation*pi/180 0]);
%           b2.Pose(1:3,4) = binCenterPosition+[0 -binWidth/2 0];
%   
%           % Add CollisionBox face 3 (Thickness 0.001 assumed)
%           b3 = collisionBox(0.001,binWidth,binHeight);
%           b3.Pose(1:3,1:3) = eul2rotm([0 binRotation*pi/180 0]);
%           b3.Pose(1:3,4) = binCenterPosition + (eul2rotm([0 binRotation*pi/180 0])*[binLength/2;0;0])';
%   
%           % Add CollisionBox face 4 (Thickness 0.001 assumed)
%           b4 = collisionBox(0.001,binWidth,binHeight);
%           b4.Pose(1:3,1:3) = eul2rotm([0 binRotation*pi/180 0]);
%           b4.Pose(1:3,4) = binCenterPosition + (eul2rotm([0 binRotation*pi/180 0])*[-binLength/2;0;0])';
%   
%           % Add CollisionBox face 5 (Thickness 0.001 assumed)
%           b5 = collisionBox(binLength,binWidth,0.001);
%           b5.Pose(1:3,1:3) = eul2rotm([0 binRotation*pi/180 0]);
%           b5.Pose(1:3,4) = binCenterPosition + (eul2rotm([0 binRotation*pi/180 0])*[0;0;-binHeight/2])';
%   
%           % Add CollisionBox Place Table (Thickness 0.05 assumed)
%           table = collisionBox(0.5,0.9,0.05);
%           table.Pose(1:3,1:3) = eul2rotm([0 0 0]);
%           table.Pose(1:3,4) = [0 0.9 -0.09];
%   
%           % Add camera as a collision box
%           camera = collisionBox(0.04,0.1,0.04);
%           camera.Pose(1:3,1:3) = eul2rotm([0 0 0]);
%           camera.Pose(1:3,4) = [0.54 0 0.61];
%   
%           env = {b1,b2,b3,b4,b5,table,camera}; % Collision enviorment array
%   
%           % Define Part dimensions (cuboid)
%           partLength = 0.1016;
%           partWidth = 0.0508;
%           partheight = 0.0508;
%   
%           % Create collision from the given part ground truth of a box shape
%           % using collisionBox
%           for i=coder.unroll(1:20)
%               if i<=numberOfParts
%                   coder.inline('never');
%                   B = collisionBox(partLength,partWidth,partheight);
%                   B.Pose(1:3,4) = [partGT(i,1) partGT(i,2) (binCenterPosition(3) - (binHeight/2) + (partheight/2) + 0.002)]';
%                   B.Pose(1:3,1:3) = eul2rotm([deg2rad(partGT(i,4)+90) 0 0]);
%                   env{end+1} = B;
%               end
%   
%           end
%       end
%   
%   end
%   
%
%% 
% 
% 
% 
% 
%