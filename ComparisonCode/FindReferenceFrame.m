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
%  /name    FindReferenceFrame
%  /brief   Matches T_in against T_ref_in and returns index of the closest
%           match.
function [minIdx] = FindReferenceFrame(T_ref_in,T_in)    
    %Keep track of weighted minima
    weightedMinimum = inf;
    minIdx = 0;
    R_weight = 6;
    Rho_weight = 1;
    
    %Input transformation matrix
    R = T_in(1:3,1:3);
    rho = T_in(:,4);
    
    %Loop over reference transformation matrix
    for T_ref_idx = 1 : size(T_ref_in,3)        
        R_ref = T_ref_in(1:3,1:3,T_ref_idx);
        rho_ref = T_ref_in(:,4,T_ref_idx);
        
        diff = R_weight * sum(abs(R_ref(:) - R(:))) + Rho_weight * sum(abs(rho_ref - rho));
        if(diff < weightedMinimum)
            weightedMinimum = diff;
            minIdx = T_ref_idx;
        end
    end
end