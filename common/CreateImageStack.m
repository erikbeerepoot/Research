%             _____ _____  _      
%      /\    / ____|  __ \| |     
%     /  \  | (___ | |__) | |     
%    / /\ \  \___ \|  _  /| |     
%   / ____ \ ____) | | \ \| |____ 
%  /_/    \_\_____/|_|  \_\______|
%  Autonomous Space Robotics Lab
% 
% Description:
% Creates an empty image stack
%
% Author:
% Erik E. Beerepoot
%
% Structure (as of Sept. 12, 2012:
%
%          azimImg: [360x480 double]
%          elevImg: [360x480 double]
%       encHorzImg: [360x480 double]
%       encVertImg: [360x480 double]
%     intense16Img: [360x480 double]
%      intense8Img: [360x480 double]
%          maskImg: [360x480 double]
%         rangeImg: [360x480 double]
%          timeImg: [360x480 double]
%        timestamp: 259.9890
%         udpIDImg: [360x480 double]
%       udpPxIDImg: [360x480 double]
%
function [imagestack] = CreateImageStack(width,height)
    imagestack.azimImg = zeros(width,height);
    imagestack.elevImg = zeros(width,height);
    imagestack.encHorzImg = zeros(width,height);
    imagestack.encVertImg = zeros(width,height);
    imagestack.intense16Img = zeros(width,height);
    imagestack.intense8Img = zeros(width,height);
    imagestack.maskImg = 255*ones(width,height);
    imagestack.rangeImg = zeros(width,height);
    imagestack.timeImg = zeros(width,height);
    imagestack.timestamp = 0;
    imagestack.udpIDImg = zeros(width,height);
    imagestack.udpPxIDImg = zeros(width,height);
end