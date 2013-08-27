%             _____ _____  _      
%      /\    / ____|  __ \| |     
%     /  \  | (___ | |__) | |     
%    / /\ \  \___ \|  _  /| |     
%   / ____ \ ____) | | \ \| |____ 
%  /_/    \_\_____/|_|  \_\______|
%  Autonomous Space Robotics Lab
% 
% Description:
% Rudimentary motion compensation on an image
%
% Author:
% Erik E. Beerepoot
%

function [v_out] = CompensateImagesPiecewiseVelocity(dataPath,outputDir,T,t_in,t_img,offsets,dirIndex)
    %Some configuration options
    GUI = 0;
    T_offset = [eye(3),[0.08;0.23;-0.1]; [0 0 0 1]];
    
    %Post processing
    kernel = [0 1 0; 1 0 1; 0 1 0] / 4;
    kMovementThreshold = 0.025;
    kEnvelopeRange = 0.5;
    
    %Add ASRL some source code to PATH
    addpath('/Users/erik/Dropbox/ASRL/asrl-code/matlab-bin/rotations/');
    addpath('/Users/erik/Dropbox/ASRL/asrl-code/matlab-bin/homogeneous/');
    addpath('/Users/erik/Dropbox/ASRL/asrl-code/matlab-bin/utilities/');
    
    % Allow multiple defaults to be retained
    if (nargin > 0)
        dataDir = dataPath;
    elseif (nargin == 0)
        dataDirs{1} = '/home/asrl/Dataset/12-May-12/ImageStacks/';
        dataSelection = 1;
        
        % Set default location
        dataDir = dataDirs{dataSelection};
    end
    
    % ************************************************ 
    % ************* Load image stacks ****************
    % ************************************************ 
    
    % In the ImageStacks folder, we find a folder for each trial 
    full_dirlist = dir(dataDir);
    
    % Exclude '.', '..', and mac resource forks.
    index = 1;
    for dirIdx = 1: length(full_dirlist)
        if(full_dirlist(dirIdx).name(1)~='.' && full_dirlist(dirIdx).isdir==1)
            dirlist(index) = full_dirlist(dirIdx);
            index = index + 1;
        end
    end
    
    %Loop over the directories (skip "." and "..")  
    endDirIndex = length(dirlist);
    if(endDirIndex < 1)
        disp('No files found in given data directory');
    end
    
    filelist = dir([dataDir dirlist(dirIndex).name '/0001/' '*.asa']);

    %Loop over the files
    endFileIndex = length(filelist);
    
    %Get timestamp of first scan
    s = loadAsrlMatArchive([dataDir dirlist(dirIndex).name '/0001/' filelist(1).name]);
    minStamp  = s.timestamp;
    
    scanDir = 0;
    velSave = [];
    
    
%     figure(3); hold on;
%     plot(range,x,'r');
%     plot(range,y,'b');
%     hold off;

%     for index = 1:(endFileIndex-1)
%         current_scan = loadAsrlMatArchive([dataDir dirlist(dirIndex).name '/0001/' filelist(index).name]);
%         next_scan    = loadAsrlMatArchive([dataDir dirlist(dirIndex).name '/0001/' filelist(index+1).name]);
%                 
%         %Calculate velocity between these frames
%         t1 = current_scan.timestamp;
%         t2 = next_scan.timestamp;
%         vel = CalculateLinearVelocity(T,t_in,dirIndex,offsets,t1,t2,minStamp,0);
%         
%         %DEBUG zero velocity for non-rotating axes
%         vel(1:5) = 0;
%         velSave(:,index) = vel;
%     end
%     
%     %Determine start of motion
%     truncatedVel = velSave(:,abs(velSave(6,:)) > kMovementThreshold);
%     truncatedVelMean = mean(truncatedVel,2)
%     
%     %Reject values that are not close to the steady state value (to
%     %calculate "true" mean
%     aboveLowerThreshold = repmat((1-kEnvelopeRange)*truncatedVelMean,1,size(velSave,2)) >= velSave
%     belowUpperThreshold = repmat((1+kEnvelopeRange)*truncatedVelMean,1,size(velSave,2)) <= velSave
%     motionStart = find(diff(all(aboveLowerThreshold & belowUpperThreshold)))
%     truncatedVel = velSave(:,(all(aboveLowerThreshold & belowUpperThreshold)));
%     
%     %Fit least squares line to velocity data
%     p = polyfit(1:size(truncatedVel,2),truncatedVel(6,:),0);
%     v_out = p(1);
%     
%     %Plot velocity on figure
%     figure(999); hold on;
%     plot(1:(50*size(velSave,2)),p(1),'g.','LineWidth',2);
    
    %HACK, set velocity
%     velSave(6,motionStart:size(velSave,2)) = p(1);
    
    velSave = zeros(6,endFileIndex-1)
    velSave(6,:) = -0.5;

    for index = 1:(endFileIndex-1)
        current_scan = loadAsrlMatArchive([dataDir dirlist(dirIndex).name '/0001/' filelist(index).name]);
        next_scan    = loadAsrlMatArchive([dataDir dirlist(dirIndex).name '/0001/' filelist(index+1).name]);
        
        %Keep track of scan direction
        scanDir = ~scanDir;
        
        %Create empty image stack
        compensated_scan = CreateImageStack(size(current_scan.intense8Img,1),size(current_scan.intense8Img,2));
        compensated_scan.elevImg = current_scan.elevImg;
        compensated_scan.azimImg = current_scan.azimImg;
        compensated_scan.maskImg  = current_scan.maskImg;
        compensated_scan.rangeImg = current_scan.rangeImg;
        
        compensated_scan.timeImg = current_scan.timeImg - min(min(current_scan.timeImg(current_scan.timeImg > 0)));
        compensated_scan.timestamp = current_scan.timestamp;
        
        %Calculate velocity between these frames
        t1 = current_scan.timestamp;
        t2 = next_scan.timestamp;
        %vel = CalculateLinearVelocity(T,t_in,dirIndex,offsets,t1,t2,minStamp,0);
        %velSave(:,index) = vel;
        %vel(1:5) = 0;
        vel = velSave(:,index);
        vel = [0;0;0;0;0;0.25];
        
        %Data fix: "skewed elevation image"
        %It seems that the elevation image varies in both the u and v
        %directions. We expect variation in v direction only.
        current_scan.elevImg = repmat(current_scan.elevImg(:,1),1,size(current_scan.elevImg,2))
         
        %Calculate min & max azimuth and elevation
       minEl = min(min(current_scan.elevImg));
       maxEl = max(max(current_scan.elevImg));
       minAz = min(min(current_scan.azimImg));
       maxAz = max(max(current_scan.azimImg));
        
%         minEl = inf;
%         maxEl = -inf;
%         minAz = inf;
%         maxAz = -inf;
%         for row = 1 : size(current_scan.intense8Img,1)
%             for col = 1 : size(current_scan.intense8Img,2)
%                 if(current_scan.maskImg(row,col)==0)
%                     continue;
%                 end
% 
%                 %a. Grab az,el,range
%                 az = current_scan.azimImg(row,col); 
%                 el = current_scan.elevImg(row,col);
%                 r  = current_scan.rangeImg(row,col);
% 
%                 %b. Convert to xyz     
%                 xyz = azElRange2xyz([az;el;r],'rad');
% 
%                 %Create transformation matrix
%                 t = compensated_scan.timeImg(row,col);
%                 R = axisAngle2r(vel(4:6)*t,1);
%                 rho = vel(1:3)*t;
%                 H = hgToTransform(R,rho);
% 
%                 %c. Compensate xyz
%                 xyz_h = H*[xyz;1];
%                 xyz = [xyz_h(1)/xyz_h(4);xyz_h(2)/xyz_h(4);xyz_h(3)/xyz_h(4)];
% 
%                 %d. Calculate (az,el,range)
%                 r_new = sqrt(xyz(1)^2 + xyz(2)^2 + xyz(3)^2);
%                 az_new = atan2(xyz(2),xyz(1));
%                 el_new = atan(xyz(3)/sqrt(xyz(1)^2 + xyz(2)^2));
%                 
%                 if(az_new < minAz)
%                     minAz = az_new;
%                 end
%                 
%                 if(az_new > maxAz)
%                     maxAz = az_new;
%                 end
%                 
%                 if(el_new < minEl)
%                     minEl = el_new;
%                 end
%                 
%                 if(el_new > maxEl)
%                     maxEl = el_new;
%                 end
%             end
%         end
%         minEl
%         maxEl
%         minAz
%         maxAz
        
        min(min(current_scan.elevImg)) %minEl
        max(max(current_scan.elevImg)) %maxEl
        min(min(current_scan.azimImg)) %minAz
        max(max(current_scan.azimImg)) %maxAz
        
        %Compensate pixel-by-pixel
        pixelTime = t_img / (size(current_scan.intense8Img,1)*size(current_scan.intense8Img,2));
        if(scanDir==0)
            for row = 1 : size(current_scan.intense8Img,1)
                for col = 1 : size(current_scan.intense8Img,2)
                    if(current_scan.maskImg(row,col)==0)
                        continue;
                    end
                    
                    %a. Grab az,el,range
                    az = current_scan.azimImg(row,col); 
                    el = current_scan.elevImg(row,col);
                    r  = current_scan.rangeImg(row,col);

                    %b. Convert to xyz     
                    xyz = azElRange2xyz([az;el;r],'rad');
    
                    %Create transformation matrix
                    t = compensated_scan.timeImg(row,col);
                    R = axisAngle2r(vel(4:6)*t,1);
                    rho = vel(1:3)*t;
                    H = hgToTransform(R,rho);

                    %c. Compensate xyz
                    xyz_h = H*[xyz;1];
                    xyz = [xyz_h(1)/xyz_h(4);xyz_h(2)/xyz_h(4);xyz_h(3)/xyz_h(4)];

                    %d. Calculate (az,el,range)
                    r_new = sqrt(xyz(1)^2 + xyz(2)^2 + xyz(3)^2);
                    az_new = atan2(xyz(2),xyz(1));
                    el_new = atan(xyz(3)/sqrt(xyz(1)^2 + xyz(2)^2));

                    [u,v] = AzElRangeToUVMinMax(az_new,el_new,r_new,minEl,maxEl,minAz,maxAz,size(current_scan.intense8Img,1),size(current_scan.intense8Img,2));                    

                    if(u<1 || u>size(compensated_scan.intense8Img,2) ||  v<1 || v>size(compensated_scan.intense8Img,1))
                        %nothing
                        if( u>size(compensated_scan.intense8Img,2) )
                            disp('exceeded height of image')
                            u
                        end
                        
                        if( v>size(compensated_scan.intense8Img,1) )
                            disp('exceeded width of image')
                            v
                        end
                    else
                        %Write value to new image stack
                        compensated_scan.intense8Img(v,u) = current_scan.intense8Img(row,col);
                        compensated_scan.maskImg(v,u) = 1;
                        
                        %Not compensated
                        %compensated_scan.intense16Img(v,u) = current_scan.intense16Img(row,col);
                        %compensated_scan.elevImg(row,col) = el_new;
                        %compensated_scan.azimImg(row,col) = az_new;
                        %compensated_scan.rangeImg(row,col) = r_new;
                    end 
                end
            end
        else 
            for row = size(current_scan.intense8Img,1) : -1 : 1
                for col = 1: size(current_scan.intense8Img,2)
                    if(current_scan.maskImg(row,col)==0)
                        continue;
                    end
                    
                    %a. Grab az,el,range
                    az = current_scan.azimImg(row,col); 
                    el = current_scan.elevImg(row,col);
                    r  = current_scan.rangeImg(row,col);

                    %b. Convert to xyz     
                    xyz = azElRange2xyz([az;el;r],'rad');

                    %Create transformation matrix
                    t = compensated_scan.timeImg(row,col);
                    R = axisAngle2r(vel(4:6)*t,1);
                    rho = vel(1:3)*t;
                    H = hgToTransform(R,rho);

                    %c. Compensate xyz
                    xyz_h = H*[xyz;1];
                    xyz = [xyz_h(1)/xyz_h(4);xyz_h(2)/xyz_h(4);xyz_h(3)/xyz_h(4)];

                    %d. Calculate (az,el,range)
                    r_new = sqrt(xyz(1)^2 + xyz(2)^2 + xyz(3)^2);
                    az_new = atan2(xyz(2),xyz(1));
                    el_new = atan(xyz(3)/sqrt(xyz(1)^2 + xyz(2)^2));

                    compensated_scan.intense8Img(row,col) = current_scan.intense8Img(row,col);
                    compensated_scan.maskImg(row,col) = current_scan.maskImg(row,col);
                    compensated_scan.elevImg(row,col) = el_new;
                    compensated_scan.azimImg(row,col) = az_new;
                    compensated_scan.rangeImg(row,col) = r_new;
                    
                    [u,v] = AzElRangeToUVMinMax(az_new,el_new,r_new,minEl,maxEl,minAz,maxAz,size(current_scan.intense8Img,1),size(current_scan.intense8Img,2));                    
                    
                    rows = 360;
                    cols = 480;
                    mel = double((rows)/(abs(double(maxEl-minEl))));
                    maz = double((cols)/(abs(double(maxAz-minAz))));

                    %Calculate the (u,v) coordinate
                    u = floor(double(cols - (maz)*((az-minAz))));
                    v = floor(double(rows - (mel)*(el-minEl)));
                    
                    
                    if(u<1 || u>size(compensated_scan.intense8Img,2) ||  v<1 || v>size(compensated_scan.intense8Img,1))
                        %nothing
                    else
                        %Write value to new image stack
                        compensated_scan.intense8Img(v,u) = current_scan.intense8Img(row,col);
                        compensated_scan.maskImg(v,u) = current_scan.maskImg(row,col);
                        
                        %Not compensated
                        %compensated_scan.intense16Img(v,u) = current_scan.intense16Img(row,col);
                        %compensated_scan.elevImg(row,col) = el_new;
                        %compensated_scan.azimImg(row,col) = az_new;
                        %compensated_scan.rangeImg(row,col) = r_new;
                    end 
                end
            end
        end

        %Linear interpolation to fill gaps    
%         for col = 3 : size(current_scan.intense8Img,2) - 2
%             for row = 3 : size(current_scan.intense8Img,1) - 2
%                 if(compensated_scan.intense8Img(row,col)==0)
%                     compensated_scan.intense8Img(row,col) = 0.25*(compensated_scan.intense8Img(row-2,col) + compensated_scan.intense8Img(row-1,col) + compensated_scan.intense8Img(row+1,col) + compensated_scan.intense8Img(row+2,col)); 
%                     compensated_scan.intense16Img(row,col) = 0.25*(compensated_scan.intense16Img(row-2,col) + compensated_scan.intense16Img(row-1,col) + compensated_scan.intense16Img(row+1,col) + compensated_scan.intense16Img(row+2,col)); 
%                     %compensated_scan.elevImg(row,col) = 0.25*(compensated_scan.elevImg(row-2,col) + compensated_scan.elevImg(row-1,col) + compensated_scan.elevImg(row+1,col) + compensated_scan.elevImg(row+2,col)); 
%                     %compensated_scan.azimImg(row,col) = 0.25*(compensated_scan.azimImg(row-2,col) + compensated_scan.azimImg(row-1,col) + compensated_scan.azimImg(row+1,col) + compensated_scan.azimImg(row+2,col)); 
%                     %compensated_scan.maskImg(row,col) = 0.25*(compensated_scan.maskImg(row-2,col) + compensated_scan.maskImg(row-1,col) + compensated_scan.maskImg(row+1,col) + compensated_scan.maskImg(row+2,col)); 
%                     %compensated_scan.rangeImg(row,col) = 0.25*(compensated_scan.rangeImg(row-2,col) + compensated_scan.rangeImg(row-1,col) + compensated_scan.rangeImg(row+1,col) + compensated_scan.rangeImg(row+2,col)); 
%                 end
%             end
%         end

        fprintf('\n');
        
        %Plot result
        figure(2); clf;
        subplot(2,1,1);
        imshow(current_scan.intense8Img / 256);
        subplot(2,1,2);
        imshow(compensated_scan.intense8Img / 256);
            
        %Create output dir if required
        [o] = dir([outputDir dirlist(dirIndex).name '/0001/']);
        if(size(o,1)==0)
            mkdir([outputDir dirlist(dirIndex).name '/0001/']);
        end
        imwrite(compensated_scan.intense8Img/256,[outputDir dirlist(dirIndex).name '/0001/' filelist(index).name '.comp.png'],'png');
        saveAsrlMatArchive([outputDir dirlist(dirIndex).name '/0001/' filelist(index).name '.comp.asa'],compensated_scan);
    end    
end