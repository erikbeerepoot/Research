%             _____ _____  _      
%      /\    / ____|  __ \| |     
%     /  \  | (___ | |__) | |     
%    / /\ \  \___ \|  _  /| |     
%   / ____ \ ____) | | \ \| |____ 
%  /_/    \_\_____/|_|  \_\______|
%  Autonomous Space Robotics Lab
%s
% Description:
% Loads groundtruth data from ASRL archives
%
% Author:
% Erik E. Beerepoot
%
% Usage:
%   1. Set asrlMATLABPaths to the path where you have checked out the ASRL
%      matlab source code,
%   2. Set dataDirs to the path where the vicon bag is located.
%   3. Set outputDirs to the path where the pre-processed vicon data should
%      end up.
%   4. Set options:
%           {plotting = 1}      - Plot the Vicon data
%   5. Run script: i.e., T = ParseVicon('file list')
%   6. Outputs are:
%           T_vicon_v           - Transformation matrices from vicon
%           t                   - Timestamps
%
function [T_vicon_v, t, sizes,names] = ParseViconGroundTruth(bagDir, fileList,plotting,useDifferentAxes)    
    %Allow the user to choose different directories for ASRL matlab code
         asrlMATLABPaths{1} = '/Users/erik/Dropbox/ASRL/asrl-code/matlab-bin/plotting';
         asrlMATLABPaths{2} = '/Users/erik/Dropbox/ASRL/asrl-code/matlab-bin/utilities';
         asrlMATLABPaths{3} = '/Users/erik/Dropbox/ASRL/asrl-code/matlab-bin/rotations';
    %     asrlMATLABPaths{4} = '/home/asrl/Dropbox/ASRL/asrl-code/matlab-bin/plotting';
    %     asrlMATLABPaths{5} = '/home/asrl/Dropbox/ASRL/asrl-code/matlab-bin/utilities';
    %     asrlMATLABPaths{6} = '/home/asrl/Dropbox/ASRL/asrl-code/matlab-bin/rotations';
    asrlMATLABPaths{4} = '/mnt/data/Dropbox/ASRL/asrl-code/matlab-bin/utilities';
    asrlMATLABPaths{5} = '/mnt/data/Dropbox/ASRL/asrl-code/matlab-bin/rotations';
    asrlMATLABPaths{6} = '/mnt/data/Dropbox/ASRL/asrl-code/matlab-bin/plotting';
    addpath(asrlMATLABPaths{:});
    
    %Check if empty
    if(size(fileList,1) < 1)
        disp('Output directory empty!');
        return;
    end

    for fileIndex = 1 : size(fileList,1)
     
        %Load the next archive containing processed vicon data
        ground_truth = loadAsrlMatArchive([bagDir fileList(fileIndex).name]);
        names{fileIndex} = fileList(fileIndex).name;
        sizes(fileIndex) = size(ground_truth.timestamps,1);
        
        %Get the size of the vicon data, and get n 4x4 T matrices
        n = size(ground_truth.T_vicon_v,1)/4;
        
        %Grab ground truth matrices
        A =  reshape(ground_truth.T_vicon_v',4,4,n);
        
        %Reorder matrix colums
        for idx = 1 : size(A,3)
            A(:,:,idx) = A(:,:,idx)';
        end
        
        
        if(fileIndex==1)
            T_vicon_v = A;
            if(useDifferentAxes==1)
                T_vicon_v(1,:,:) = A(3,:,:);
                T_vicon_v(3,:,:) = A(1,:,:);
                A= T_vicon_v;
            else 
                %T_vicon_v(2,:,:) = A(3,:,:);
                %T_vicon_v(3,:,:) = A(2,:,:);
                A= T_vicon_v;
            end
            
            
            t = ground_truth.timestamps';
        elseif(fileIndex > 1)
            if(useDifferentAxes==1)
                B=A;
                B(1,:,:) = A(3,:,:);
                B(3,:,:) = A(1,:,:);
                A = B;
            else
                B=A;
                %T_vicon_v(2,:,:) = A(3,:,:);
                %T_vicon_v(3,:,:) = A(2,:,:);
                A = B;
            end
            
            T_vicon_v = ConcatenateTransformationMatrix(T_vicon_v,A);
            t = ConcatenateTimeVector(t,ground_truth.timestamps');
        end
        
        
        %Plot vicon data if required  
        if(plotting)
            %Set up plotting parameters
            figure(1); clf;
            hold on; axis equal; grid on;
            xlabel('x'); ylabel('y'); zlabel('z')
            %plotCoordinateFrame(eye(3), [0,0,0]' ,5); 
            plotCoordinateFrame(rph2c([pi/2,-pi/2,pi]),[0,0,0]');
            title([fileList(fileIndex).name]);
            
            %Plot each frame
            for i = 1:10:n
                T_t = A(:,:,i);
                plotCoordinateFrame(T_t(1:3,1:3),T_t(1:3,4),2); 
            end
        end
    end