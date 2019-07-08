clear all
clc
close all
m=[109.796448655349 49.3566595579915 -2.6144626049236 3.18437226327890;4.76644293633854 44.9476795201901 -100.305282252619 -0.194987162038942;0.00951986980857828 0.114388808629263 -0.0080358074460270 -0.00180341422458125];
k=1;
ptCloudObj={};
clds = [
1574
1582
1633
1638
1643
1657
1684
1694
1704
1718
1730
1737
1744
1750
1768
1776
1783
1788
1795
1800]
imgs = [1607,1615,1667,1672,1678,1692,1719,1730,1740,1754,1766,1773,1781,1787,1805,1813,1820,1826,1833,1838];
for i=1:size(clds,1)
ptCloudObj{i}  = pcread("video5/lidar"+clds(i)+".pcd");
imgs = [imgs round(clds(i)+(clds(i)*0.021))];
end


bbs = read_bbs('bbs.txt');

inv_m=pinv(m);
centers = {};
factor =10;
%for cl=1:size(ptCloudObj,2)
for cl=1:20
    centers{cl} = [];
     figure();
     pcshow(ptCloudObj{cl})
      view(0,90);
    xlim([-40 40]);
    ylim([0 80]);
for i=1:size(bbs{cl},1)
 
   [bb3d,center] = Get_Lidar_BB(bbs{cl}(i,:),inv_m,ptCloudObj{cl})
  
    centers{cl} = [centers{cl}; center(1),center(2) ]
    hold on
    scatter3(centers{cl}(i,1),centers{cl}(i,2),0);
    if(size(centers,2)>1)
        %endpoint = [centers{cl-1}(i,1)+factor*(centers{cl}(i,1)-centers{cl-1}(i,1))
            %centers{cl-1}(i,2)+factor*(centers{cl}(i,2)-centers{cl-1}(i,2))];
        %plot3([centers{cl}(i,1),endpoint(1)],[centers{cl}(i,2),endpoint(2)],[0 0],'LineWidth',1,'color','Black');
        dp = [[centers{cl}(i,:) 0] - [centers{cl-1}(i,:) ,0]];
        quiver3([centers{cl}(i,1)],[centers{cl}(i,2)],[0],dp(1),dp(2),dp(3),4,'LineWidth',2);
    end
hold on
    plot3([bb3d(1,1),bb3d(2,1)],[bb3d(1,2),bb3d(2,2)],[0 0],'LineWidth',1,'color','Black');
hold on
%plot3([maxx maxx],[mind maxd], [0 0]);
plot3([bb3d(2,1),bb3d(3,1)],[bb3d(2,2),bb3d(3,2)],[0 0],'LineWidth',1,'color','Black');
hold on
%plot3([minx maxx],[mind mind], [0 0]);
plot3([bb3d(3,1),bb3d(4,1)],[bb3d(3,2),bb3d(4,2)],[0 0],'LineWidth',1,'color','Black');
hold on
%plot3([minx minx],[mind maxd], [0 0]);
plot3([bb3d(4,1),bb3d(1,1)],[bb3d(4,2),bb3d(1,2)],[0 0],'LineWidth',1,'color','Black');

end
hold off
F(cl) = getframe(gcf) ;
    drawnow
end

 writerObj = VideoWriter('short_lidar.avi');
  writerObj.FrameRate = 2;
  % set the seconds per image
% open the video writer
open(writerObj);
% write the frames to the video
for i=1:length(F)
    % convert the image to a frame
    frame = F(i) ;    
    writeVideo(writerObj, frame);
end
% close the writer object
close(writerObj);

