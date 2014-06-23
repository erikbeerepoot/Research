
    %Processed rotational speed
    rotSpeed = [v(1),mean([v(2),v(3)]),mean([v(4),v(5)]),v(6),v(7),mean([v(8),v(9)]),v(10)]
    
    %%%%%%%%%%%%%%%%% SURF %%%%%%%%%%%%%%%%%%%
    %Compute mean matching scores
    meanCompScore = (sum(surfCompScore') ./ sum(surfCompScore'>0));
    meanDistortScore = (sum(surfDistortScore') ./ sum(surfDistortScore'>0));
    meanTrackLength = (sum(compTrackLength') ./ sum(compTrackLength'>0));
    meanDistortTrackLength = (sum(distortTrackLength') ./ sum(distortTrackLength'>0));
    
    for idx = 1 : size(surfCompScore,1)
        stdCompScore(idx) = std(surfCompScore(idx,surfCompScore(idx,:) > 0))
        stdDistortScore(idx) = std(surfDistortScore(idx,surfDistortScore(idx,:) > 0))
        stdTrackLength(idx) = std(compTrackLength(idx,compTrackLength(idx,:) > 0))
        stdDistortTrackLength(idx) = std(distortTrackLength(idx,distortTrackLength(idx,:) > 0))
    end
    
    processedCompScore = [meanCompScore(1), mean([meanCompScore(2) meanCompScore(3)]), mean([meanCompScore(4) meanCompScore(5)]), meanCompScore(6),meanCompScore(7), mean([meanCompScore(8),meanCompScore(9)]),meanCompScore(10)]
    processedCompScoreStd = [stdCompScore(1), mean([stdCompScore(2) stdCompScore(3)]), mean([stdCompScore(4) stdCompScore(5)]), stdCompScore(6),stdCompScore(7), mean([stdCompScore(8),stdCompScore(9)]),stdCompScore(10)]
    processedDistortScore = [meanDistortScore(1), mean([meanDistortScore(2),meanDistortScore(3)]), mean([meanDistortScore(4),meanDistortScore(5)]), meanDistortScore(6),meanDistortScore(7), mean([meanDistortScore(8),meanDistortScore(9)]),meanDistortScore(10)]
    processedDistortScoreStd = [stdDistortScore(1), mean([stdDistortScore(2),stdDistortScore(3)]), mean([stdDistortScore(4),stdDistortScore(5)]), stdDistortScore(6),stdDistortScore(7), mean([stdDistortScore(8),stdDistortScore(9)]),stdDistortScore(10)]
    
    processedCompLength = [meanTrackLength(1), mean([meanTrackLength(2), meanTrackLength(3)]), mean([meanTrackLength(4), meanTrackLength(5)]), meanTrackLength(6),meanTrackLength(7), mean([meanTrackLength(8),meanTrackLength(9)]),meanTrackLength(10)]
    processedDistortLength = [meanDistortTrackLength(1), mean([meanDistortTrackLength(2),meanDistortTrackLength(3)]), mean([meanDistortTrackLength(4),meanDistortTrackLength(5)]), meanDistortTrackLength(6),meanDistortTrackLength(7), mean([meanDistortTrackLength(8),meanDistortTrackLength(9)]),meanDistortTrackLength(10)]
    processedCompLengthStd = [stdTrackLength(1), mean([stdTrackLength(2), stdTrackLength(3)]), mean([stdTrackLength(4), stdTrackLength(5)]), stdTrackLength(6),stdTrackLength(7), mean([stdTrackLength(8),stdTrackLength(9)]),stdTrackLength(10)]
    processedDistortLengthStd = [stdDistortTrackLength(1), mean([stdDistortTrackLength(2),stdDistortTrackLength(3)]), mean([stdDistortTrackLength(4),meanDistortTrackLength(5)]), stdDistortTrackLength(6),stdDistortTrackLength(7), mean([stdDistortTrackLength(8),stdDistortTrackLength(9)]),stdDistortTrackLength(10)]
    
    %Plot matching score
    figure(1); clf; hold on;
    plot(abs(rotSpeed),processedCompScore,'b-x','LineWidth',2);
    plot(abs(rotSpeed),processedDistortScore,'r-x','LineWidth',2)
    legend('Compensated score','Distorted score');
    xlabel('Rotational speed (rad/s)');
    ylabel('Normalized matching score');
    
    %Plot errorbars
    errorbar(abs(rotSpeed),processedCompScore,processedCompScoreStd,'b');
    errorbar(abs(rotSpeed),processedDistortScore,processedDistortScoreStd,'r');
    
    
    %Plot track length
    figure(2); clf; hold on;
    plot(abs(rotSpeed),processedCompLength,'b-x','LineWidth',2);
    plot(abs(rotSpeed),processedDistortLength,'r-x','LineWidth',2)
    legend('Compensated score','Distorted score');
    xlabel('Rotational speed (rad/s)');
    ylabel('Track length (pixels)');
    
    %Plot errorbars
    errorbar(abs(rotSpeed),processedCompLength,processedCompLengthStd,'b');
    errorbar(abs(rotSpeed),processedDistortLength,processedDistortLengthStd,'r');
    
%     figure(3); clf; hold on;
%     plot(surfCompScore(1,:),'b');
%     plot(surfDistortScore(1,:),'r');
%     hold off;
%     
%     figure(4); clf; hold on;
%     plot(surfCompScore(2,surfCompScore(2,:)>0),'b');
%     plot(surfDistortScore(2,surfDistortScore(2,:)>0),'r');
%     hold off;
%     
%     figure(5); clf; hold on;
%     plot(surfCompScore(4,surfCompScore(4,:)>0),'b');
%     plot(surfDistortScore(4,surfDistortScore(4,:)>0),'r');
%     hold off;
%     
%     figure(6); clf; hold on;
%     plot(surfCompScore(6,surfCompScore(6,:)>0),'b');
%     plot(surfDistortScore(6,surfDistortScore(6,:)>0),'r');
%     hold off;

%%%% BIMODAL TRACK LENGTH MODEL
for idx = 1 : size(surfCompScore,1)
        bimodalCompTrackLength(idx,1) = sum(compTrackLength(idx,1:2:size(compTrackLength,2)) / sum(compTrackLength(idx,1:2:size(compTrackLength,2)) > 0));
        bimodalCompTrackLength(idx,2) = sum(compTrackLength(idx,2:2:size(compTrackLength,2)) / sum(compTrackLength(idx,2:2:size(compTrackLength,2)) > 0));
        bimodalDistortTrackLength(idx,1) = sum(distortTrackLength(idx,1:2:size(distortTrackLength,2)) / sum(distortTrackLength(idx,1:2:size(distortTrackLength,2)) > 0));
        bimodalDistortTrackLength(idx,2) = sum(distortTrackLength(idx,2:2:size(distortTrackLength,2)) / sum(distortTrackLength(idx,2:2:size(distortTrackLength,2)) > 0));
end

bimodalCompTrackLength2(:,1) = [bimodalCompTrackLength(1,1), mean([bimodalCompTrackLength(2,1),bimodalCompTrackLength(3,1)]),mean([bimodalCompTrackLength(4,1),bimodalCompTrackLength(5,1)]),bimodalCompTrackLength(6,1),bimodalCompTrackLength(7,1),mean([bimodalCompTrackLength(8,1),bimodalCompTrackLength(9,1)]),bimodalCompTrackLength(10,2)]';
bimodalCompTrackLength2(:,2) = [bimodalCompTrackLength(1,2), mean([bimodalCompTrackLength(2,2),bimodalCompTrackLength(3,2)]),mean([bimodalCompTrackLength(4,2),bimodalCompTrackLength(5,2)]),bimodalCompTrackLength(6,2),bimodalCompTrackLength(7,2),mean([bimodalCompTrackLength(8,2),bimodalCompTrackLength(9,2)]),bimodalCompTrackLength(10,1)]';
bimodalDistortTrackLength2(:,1) = [bimodalDistortTrackLength(1,2), mean([bimodalDistortTrackLength(2,1),bimodalDistortTrackLength(3,1)]),mean([bimodalDistortTrackLength(4,2),bimodalDistortTrackLength(5,2)]),bimodalDistortTrackLength(6,1),bimodalDistortTrackLength(7,1),mean([bimodalDistortTrackLength(8,2),bimodalDistortTrackLength(9,2)]),bimodalDistortTrackLength(10,2)]';
bimodalDistortTrackLength2(:,2) = [bimodalDistortTrackLength(1,1), mean([bimodalDistortTrackLength(2,2),bimodalDistortTrackLength(3,2)]),mean([bimodalDistortTrackLength(4,2),bimodalDistortTrackLength(5,1)]),bimodalDistortTrackLength(6,2),bimodalDistortTrackLength(7,2),mean([bimodalDistortTrackLength(8,1),bimodalDistortTrackLength(9,1)]),bimodalDistortTrackLength(10,1)]';

figure; clf; hold on;
plot(abs(rotSpeed),bimodalCompTrackLength2(:,1),'bx','LineWidth',1)
plot(abs(rotSpeed),bimodalCompTrackLength2(:,2),'bx','LineWidth',1)
plot(abs(rotSpeed),bimodalDistortTrackLength2(:,1),'rx','LineWidth',1);
plot(abs(rotSpeed),bimodalDistortTrackLength2(:,2),'rx','LineWidth',1);


coeffs = polyfit(abs(rotSpeed),bimodalCompTrackLength2(:,1)',1)
coeffs2 = polyfit(abs(rotSpeed),bimodalCompTrackLength2(:,2)',1)
coeffs3 = polyfit(abs(rotSpeed),bimodalDistortTrackLength2(:,1)',1)
coeffs4 = polyfit(abs(rotSpeed),bimodalDistortTrackLength2(:,2)',1)

vals=polyval(coeffs,abs(rotSpeed))
vals2=polyval(coeffs2,abs(rotSpeed))
vals3=polyval(coeffs3,abs(rotSpeed))
vals4=polyval(coeffs4,abs(rotSpeed))

plot(abs(rotSpeed),vals,'b-','LineWidth',2);
plot(abs(rotSpeed),vals2,'b-','LineWidth',2);
plot(abs(rotSpeed),vals3,'r-','LineWidth',2);
plot(abs(rotSpeed),vals4,'r-','LineWidth',2);

hold off