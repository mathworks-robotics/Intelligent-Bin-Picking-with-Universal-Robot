function [planPath, shortPath, interpolatePath]  =  exampleHelperBinPickingRRTMotionPlanner(partID,partGT,startConfig,endConfig,pathType) %#codegen
%This function is for internal use only and may be removed in the future.
%exampleHelperBinPickingRRTMotionPlanner is a motion planner function in a collision environment
% for Universal Robots UR5e a robot
%   [PLANPATH, SHORTPATH, INTERPOLATEPATH]  =  exampleHelperBinPickingRRTMotionPlanner(PARTID,PARTGT,STARTCONFIG,ENDCONFIG,PATHTYPE)
%
%   OUTPUT:
%   Outputs a collision-free geometric plan and interpolated path,
%
%   INPUT:
%   PARTID is the index of the object to pick.
%   PARTGT is the array representing the part CenterPoint and its orientation
%   with respect to the vertical Z-axis.
%   START and GOAL are the start and goal joint configurations of the
%   robot, respectively, and are specified as a row vector.
%   PATHTYPE is the type of the path as 'approach' and 'place' in a char array.
%
%   All the dimensions are in meters.

%Copyright 2023 The MathWorks, Inc.

% Import Universal Robots UR5e as a rigid body tree model
rbt = importrobot('universalUR5e.urdf','MaxNumBodies',22,'DataFormat','row');

% Attach Robotiq Epick suction gripper body as a fixed joint
% at the end-effector of the robot
ur5e = exampleHelperAddGripper(rbt);
tformZYX = eul2tform([0 0 0]);
setFixedTransform(ur5e.Base.Children{1,2}.Joint,tformZYX);

% Bin dimensions in meters (Used to create a collision environment
binLength = 0.395; % Along X axis
binWidth = 0.585; % Along Y axis
binHeight = 0.11;
binCenterPosition = [0.48 0 -0.038];
binRotation = 0;

% Radius to switch planner to optimize computations
planRadius1 = 0.18;
planRadius2 = 0.21;

% Set planner parameters according to the planer radius for the approach
% and place trajectory
if string(pathType) == "approach"
    % Distance between the part center point and bin center point
    dist = pdist([partGT(partID,1:2);binCenterPosition(1:2)]);

    % Set MaxConnectionDistance and ValidationDistance properties of the
    % planner according to the plan radius
    if dist < planRadius1
        MaxConnectionDistance = 0.05;
        ValidationDistance = MaxConnectionDistance/4;
    elseif dist >  planRadius2
        MaxConnectionDistance = 0.15;
        ValidationDistance = MaxConnectionDistance/2;
    else
        MaxConnectionDistance = 0.1;
        ValidationDistance = MaxConnectionDistance/3;
    end
else
    MaxConnectionDistance = 0.1;
    ValidationDistance = 0.1;
end

% Modify the robot by adding a part as a collision box with the end-effector
% for place type of trajectory to avoid part collision with the environment
% while placing it at the destination.
if string(pathType) == "place"
    % Find end effector transform
    endEffectorTransform = getTransform(ur5e,startConfig,ur5e.BodyNames{end});
    eulerTransform = tform2eul(endEffectorTransform);

    % Modify the robot rigid body tree by adding part at the end-effector
    ur5e = exampleHelperAttachPartAtEndEffector(ur5e,-deg2rad(partGT(partID,4))+eulerTransform(1)+pi);
end

%% Start Planning
numOfParts = size(partGT,1); % Number of parts

% Create a collision environment for the planner
env = exampleHelperGenerateCollisionEnviornment(binLength, binWidth, binHeight, binCenterPosition, binRotation,numOfParts,partGT);

index = false(1,length(env));
index(7+partID) = true;

env(index) = [];

% Create and set up the planner from the rigid body tree and
% environment.
planner = manipulatorRRT(ur5e,env);
planner.MaxConnectionDistance = MaxConnectionDistance;
planner.ValidationDistance = ValidationDistance;
planner.SkippedSelfCollisions="parent";

% Compute the planned path for the given start and end configuration
% using the planner
path = plan(planner,startConfig,endConfig);

% Compute short and interpolated path form the computed path
dist = pdist([partGT(partID,1:2);binCenterPosition(1:2)]);
if dist < planRadius1
    shortFlag = false;
else
    shortFlag = true;
end

if shortFlag
    numIteration = 20;
    shortPath = shorten(planner,path,numIteration);
    interpConfigurations = interpolate(planner,shortPath);
else
    shortPath = path;
    interpConfigurations = interpolate(planner,shortPath);
end

planPath = path;
interpolatePath = interpConfigurations;

%% Helper functions
    function ur5e = exampleHelperAttachPartAtEndEffector(ur5e,rotation)
        %This function is for internal use only and may be removed in the future.
        % This helper function is used to attach a part body to the
        % end-effector to update the rigid body tree for collision-free
        % trajectory in the given environment


        % Part Dimensions in meters
        partwidth = 0.0508;
        partheight = 0.0508;
        partLength = 0.1016;

        % Computed transformation matrix for adding collision object
        transformForCollision = eul2tform([rotation+pi/2 0 0]);
        transformForCollision(:,4) = [0; 0; partheight/2-0.01; 1];

        % Attach collision box to the rigid body model
        part = rigidBody('part','MaxNumCollisions',1);
        box = [partLength partwidth partheight];
        addCollision(part,'box',box,transformForCollision);

        % Computed transformation matrix for adding fixed joint for object
        transformPart = eul2tform([0 0 0]);
        transformPart(:,4) = [0; 0; 0.005; 1]; % To avoid self collision add 0.005m offset

        % Create a fixed joint and attach it to the robot end-effector body
        partJoint = rigidBodyJoint('partJoint','fixed');
        part.Joint = partJoint;
        setFixedTransform(part.Joint, transformPart);
        curEndEffectorBodyName = ur5e.BodyNames{end};
        addBody(ur5e,part,curEndEffectorBodyName);
    end


    function env = exampleHelperGenerateCollisionEnviornment(binLength, binWidth, binHeight, binCenterPosition, binRotation,numberOfParts,partGT)
        %This function is for internal use only and may be removed in the future.
        % This helper function generates a collision environment array with
        % the bin, parts, and the camera


        % Add CollisionBox face 1 (Thickness 0.001 assumed)
        b1 = collisionBox(binLength,0.001,binHeight);
        b1.Pose(1:3,1:3) = eul2rotm([0 binRotation*pi/180 0]);
        b1.Pose(1:3,4) = binCenterPosition+[0 binWidth/2 0];

        % Add CollisionBox face 2 (Thickness 0.001 assumed)
        b2 = collisionBox(binLength,0.001,binHeight);
        b2.Pose(1:3,1:3) = eul2rotm([0 binRotation*pi/180 0]);
        b2.Pose(1:3,4) = binCenterPosition+[0 -binWidth/2 0];

        % Add CollisionBox face 3 (Thickness 0.001 assumed)
        b3 = collisionBox(0.001,binWidth,binHeight);
        b3.Pose(1:3,1:3) = eul2rotm([0 binRotation*pi/180 0]);
        b3.Pose(1:3,4) = binCenterPosition + (eul2rotm([0 binRotation*pi/180 0])*[binLength/2;0;0])';

        % Add CollisionBox face 4 (Thickness 0.001 assumed)
        b4 = collisionBox(0.001,binWidth,binHeight);
        b4.Pose(1:3,1:3) = eul2rotm([0 binRotation*pi/180 0]);
        b4.Pose(1:3,4) = binCenterPosition + (eul2rotm([0 binRotation*pi/180 0])*[-binLength/2;0;0])';

        % Add CollisionBox face 5 (Thickness 0.001 assumed)
        b5 = collisionBox(binLength,binWidth,0.001);
        b5.Pose(1:3,1:3) = eul2rotm([0 binRotation*pi/180 0]);
        b5.Pose(1:3,4) = binCenterPosition + (eul2rotm([0 binRotation*pi/180 0])*[0;0;-binHeight/2])';

        % Add CollisionBox Place Table (Thickness 0.05 assumed)
        table = collisionBox(0.5,0.9,0.05);
        table.Pose(1:3,1:3) = eul2rotm([0 0 0]);
        table.Pose(1:3,4) = [0 0.9 -0.09];

        % Add camera as a collision box
        camera = collisionBox(0.04,0.1,0.04);
        camera.Pose(1:3,1:3) = eul2rotm([0 0 0]);
        camera.Pose(1:3,4) = [0.54 0 0.61];

        env = {b1,b2,b3,b4,b5,table,camera}; % Collision enviorment array

        % Define Part dimensions (cuboid)
        partLength = 0.1016;
        partWidth = 0.0508;
        partheight = 0.0508;

        % Create collision from the given part ground truth of a box shape
        % using collisionBox
        for i=coder.unroll(1:20)
            if i<=numberOfParts
                coder.inline('never');
                B = collisionBox(partLength,partWidth,partheight);
                B.Pose(1:3,4) = [partGT(i,1) partGT(i,2) (binCenterPosition(3) - (binHeight/2) + (partheight/2) + 0.002)]';
                B.Pose(1:3,1:3) = eul2rotm([deg2rad(partGT(i,4)+90) 0 0]);
                env{end+1} = B;
            end

        end
    end

end



