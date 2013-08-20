%             _____ _____  _      
%      /\    / ____|  __ \| |     
%     /  \  | (___ | |__) | |     
%    / /\ \  \___ \|  _  /| |     
%   / ____ \ ____) | | \ \| |____ 
%  /_/    \_\_____/|_|  \_\______|
%  Autonomous Space Robotics Lab
%
% Description:
% Matches timestamps in two vectors, and returns the index. The assumption
% is that the start of each vector is synchronized.
%
% Author:
% Erik E. Beerepoot 
%
function [Tindex] = MatchTimestamps(stampTime,minStampTime, rosT)
    % Compute the relative timestamps
    truncatedTimestamps = rosT(rosT > 0);
    stamps = truncatedTimestamps - min(truncatedTimestamps);
    scanStamp = stampTime - minStampTime;
    
    z = abs(stamps-scanStamp);
    Tindex = find(min(z)==z);
    
    %Return -1 if no index found
    if(isempty(Tindex))
        Tindex = -1;
    end
end

function [tol] = ComputeTimestampTolerance(t)
    
    %Compute mean difference in timestamps
    tdiff = 0;
    for index = 3 : size(t,1)
        tdiff = tdiff + (t(index) - t(index-1));
    end
    
    mean = tdiff / size(t,1);
    
    %Now that we have the mean, compute the standard deviation
    sqdiff = 0;
    tdiff = 0;
    for index = 3 : size(t,1)
         tdiff = (t(index) - t(index-1));
         sqdiff = sqdiff + (tdiff - mean)^2;
    end
    variance = sqdiff / size(t,1);
    std = sqrt(variance);
    
    tol = 3*std; 
end