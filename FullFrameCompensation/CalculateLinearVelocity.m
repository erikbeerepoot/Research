%      ___           ___           ___           ___
%     /\  \         /\  \         /\  \         /\__\
%    /::\  \       /::\  \       /::\  \       /:/  /
%   /:/\:\  \     /:/\ \  \     /:/\:\  \     /:/  /
%  /::\~\:\  \   _\:\~\ \  \   /::\~\:\  \   /:/  /
% /:/\:\ \:\__\ /\ \:\ \ \__\ /:/\:\ \:\__\ /:/__/
% \/__\:\/:/  / \:\ \:\ \/__/ \/_|::\/:/  / \:\  \
%      \::/  /   \:\ \:\__\      |:|::/  /   \:\  \
%      /:/  /     \:\/:/  /      |:|\/__/     \:\  \
%     /:/  /       \::/  /       |:|  |        \:\__\
%     \/__/         \/__/         \|__|         \/__/
% -----------------------------------------------------
% ----------> Autonomous Space Robotics Lab <----------
% -----------------------------------------------------
% @name     CalculateLinearVelocity
% @brief    Calculates the velocity between two times using a linear
%           approximation.
function [v_out] =  CalculateLinearVelocity(T,t_in,dirIndex,offsets,t1,t2,minStamp,debug)

persistent lastDirIndex;
if(isempty(lastDirIndex)  || (dirIndex~=lastDirIndex))
    lastDirIndex = dirIndex;
    figure(999); clf;
end
    

v_out = zeros(6,1);

index = 1;
K_NUM_TRIALS = 150;
K_INLIER_THRESH = 0.1;

t = t_in(dirIndex,:);
t = t(t>0);

%% Transform index 1 %%%
tIdx1 = offsets(dirIndex) + MatchTimestamps(t1,minStamp,t) ;

%Check if we have actually found a matching transform index
if(tIdx1==-1)
    disp('No matching scan found!')
end 



%%% Transform index 2 %%%
% tIdx2 = offsets(dirIndex) + MatchTimestamps(t2,minStamp,t) ;
% %Check if we have actually found a matching transform index
% if(tIdx1==-1)
%     disp('No matching scan found!')  
% end 
tIdx2 = tIdx1 + 50;

%Using all transforms, calculate robot velocity
v = [];
for idx = tIdx1+1: (tIdx2)
    T1 = reshape(T((1 + (dirIndex-1)*4) : (4 + (dirIndex-1)*4),1:4,idx),4,4);
    T2 = reshape(T((1 + (dirIndex-1)*4) : (4 + (dirIndex-1)*4),1:4,idx-1),4,4);
    
%     if(all(all(T1)==0) || all(all(T2)==0)) 
%         continue;
%     end
    
    T_rel = T2 / T1;
    euler = fromTEuler(T_rel);
    
    %diff = t(idx+1) - t(idx);
    diff = t(idx) - t(idx-1);
    if(diff > 0)
        v(:,idx-offsets(dirIndex)) = euler / diff;
    else 
        v(:,idx-offsets(dirIndex)) = zeros(6,1);
    end
end








if(size(v,2)==0)
    return;
end
%Reject the atrocious outliers
v = v(:,~any(abs(v) > 1));

%smooth signal
v = filter(ones(1,5),1,v)

for velIdx = 1 : size(v_out,1)
    res_error = inf;
    m = 0;
    b = 0;
    m_save = 0;
    b_save = 0;
    inliers_save = [];
    inliers = [];
    
    % Apply RANSAC 
    % Fit lines using LS error metric
    for idx = 1 : K_NUM_TRIALS    
        if(debug==1)
            figure(1); clf; hold on;
            plot(find(any(v>0)),v(velIdx,any(v>0)),'b');
            %plot(v(velIdx,:),'b','LineWidth',1);    
        end

        %1. Randomly generate 2 indices
        idx1 = (tIdx1-offsets(dirIndex)) + (round(rand*(size(v(:,any(v>0)),2)-1)));
        idx2 = (tIdx1-offsets(dirIndex)) + (round(rand*(size(v(:,any(v>0)),2)-1)));

        %2. Get points for those indices    
        v1 = v(:,idx1);
        v2 = v(:,idx2);

        %3. Fit line (generate hypothesis)
        m = (v2(velIdx) - v1(velIdx)) /  (idx2 - idx1);
        b = v1(velIdx) - m*idx1;

        if(debug==1)
            x = find(any(v>0));
            y = m*x + b;

            plot(idx1,v1(velIdx),'r.','LineWidth',3);
            plot(idx2,v2(velIdx),'r.','LineWidth',3);
            plot(x,y,'g','LineWidth',0.5);
            pause(0.05);
        end

        %4. Get consensus set
        inliers = [];
        for i = 1 : size(v,2)
            %a. Calculate residual
            r = v(velIdx,i) - (m*i+b);

            %b. Check if inlier
            if( abs(r) < K_INLIER_THRESH)
                inliers = [inliers, v(velIdx,i)];
            end
        end
    
        if(size(inliers,2) > 0)
            %5. Compute error for consensus set
            sum_r = 0;
            for i = 1 : size(inliers,2)
                sum_r = sum_r + (inliers(i) - (m*i+b));
            end

            %6. Check if this model fits the data better
            if(abs(sum_r) < abs(res_error) && (m~=NaN) && (b~=NaN))
                %a. save error for next iteration
                res_error = abs(sum_r);

                %b. Save model params and inlier set
                m_save = m;
                b_save = b;
                inliers_save = inliers;
            end    
        end
    end

    if(debug==1)
        if(velIdx==6)
            labels = {'Vx','Vy','Vz','Wx','Wy','Wz'};
            hold off;

            %Plot final solution
            figure(1); clf; hold on;
            x = find(any(v>0));
            y = m_save*x + b_save;
            plot(find(any(v>0)),v(velIdx,any(v>0)),'b');
            %plot(1:size(v,2),v(velIdx,:),'b');
            plot(x,y,'g','LineWidth',1);  
            legend(labels{velIdx},'Linear fit to Wz');
            hold off;
        end
    end
    
    if(size(inliers,2) > 0)
        v_out(velIdx) = mean(inliers);
    else
        v_out(velIdx) = mean(v(velIdx,:));
    end
end

%if(debug==1)
    figure(999); hold on;
    plot(find(any(v>0)),v(6,any(v>0)),'b');
   
    x = find(any(v>0));
    y = m_save*x + b_save;
    plot(x,y,'r','LineWidth',2);   
    hold off;
%end

v_out(6) = mean(y);



end
