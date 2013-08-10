%       ___           ___           ___           ___ 
%      /\  \         /\  \         /\  \         /\__\
%     /::\  \       /::\  \       /::\  \       /:/  /
%    /:/\:\  \     /:/\ \  \     /:/\:\  \     /:/  / 
%   /::\~\:\  \   _\:\~\ \  \   /::\~\:\  \   /:/  /  
%  /:/\:\ \:\__\ /\ \:\ \ \__\ /:/\:\ \:\__\ /:/__/   
%  \/__\:\/:/  / \:\ \:\ \/__/ \/_|::\/:/  / \:\  \   
%       \::/  /   \:\ \:\__\      |:|::/  /   \:\  \  
%       /:/  /     \:\/:/  /      |:|\/__/     \:\  \ 
%      /:/  /       \::/  /       |:|  |        \:\__\
%      \/__/         \/__/         \|__|         \/__/
%  |---------- Autonomous Space Robotics Lab ---------|    
function [idx] = FindReferenceFrame(referenceImagestackDir,T_ref_in,T_in)
    %Lazy initialization
    %if(~exist('referenceTransformationMatrices'))
        referenceTransformationMatrices = FindReferenceTransforms(referenceImagestackDir,T_ref_in)               
    %end
    
    % Match T_in against T_ref_in
    
end