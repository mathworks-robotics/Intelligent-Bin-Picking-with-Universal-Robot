function [bboxes,scores,labels] = detectObjectsYoloNet(trainedYoloNet,rgbImage)
%This function is for internal use only and may be removed in the future.

%DETECTOBJECTSYOLONET detects objects in the given image using the trained YOLO object detector
%Inputs:
%   trainedYoloNet - yolov4 object detector trained for detecting the
%                   specific object (1X1 yolov4ObjectDetector)
%   rgbImage - Input RGB image (MXNX3)
%Outputs:
%   bboxes -   P-by-4 matrix defining P bounding boxes. Each row of bboxes
%              contains a four-element vector, [x, y, width, height]. This
%              vector specifies the upper-left corner and size of a bounding
%              box in pixels 
%   scores -  confidence scores for each bounding box (PX1)
%   labels - labels assigned to the bounding boxes (PX1)

%   Copyright 2023 The MathWorks, Inc.

    networkInputSize = trainedYoloNet.InputSize;
    originalInputSize = size(rgbImage);

% Input image resizing before passing it to the YOLO detector. Its
    % recommended to keep this resize operation same as the resize
    % operation used while training the network
    resizeFactor = networkInputSize(1)/originalInputSize(2);
    offset = round((networkInputSize(1) - originalInputSize(1)*resizeFactor)/2);
    Tout =[ resizeFactor 0 0; 0 resizeFactor 0; 0 offset 1];
    tform = affine2d(Tout);
    I = imwarp(rgbImage,tform,...
        'OutputView',imref2d([networkInputSize(2) networkInputSize(1)]),...
        'SmoothEdges',true,...
        'FillValues',127);
    % Detect the objects
    [bboxes,scores,labels] = detect(trainedYoloNet,I);
    
    % Transform the bounding coordinates to the original dimension of the
    % input image
    bboxes(:,2) = bboxes(:,2) - offset;
    topLeft = bboxes(:,1:2) - 1;
    topLeft = topLeft.*(size(rgbImage,2)./networkInputSize(2));
    bboxes = [topLeft + 1, bboxes(:,3:4).*(size(rgbImage,2)./networkInputSize(2))];
    
% expand bounding boxes by a few pixels if required
    bboxes(:,1:2) = bboxes(:,1:2)-5;
    bboxes(:,3:4) = bboxes(:,3:4)+5;
end
