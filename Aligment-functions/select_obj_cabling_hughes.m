function [ret_pts, cat_pts] = select_obj_cabling_hughes(cloud_input, rails_input)
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Keep only the objective area
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    rails_pca   =   pca(rails_input);
    rails       =   rails_input * rails_pca;
    centre_pt   =   mean(rails);
    rails       =   rails - centre_pt;
    cloud       =   align_cloud(cloud_input,rails_pca); 
    cloud       =   translate_cloud(cloud,centre_pt); 
    

    
    %%
%     figure; pcshow(cabling_pts, sign(pt_to_plane_d))
%     createPlane(mean(rails), [0 1 0]);
%     drawPlane3d(ans)
%     hold on; pcshow(cloud.Location(cat_pts, :), 'r')
%     createPlane(mean(cloud.Location(cat_pts, :)), [0 1 0]);
%     drawPlane3d(ans)
%     hold on; pcshow(cloud.Location(ret_pts, :), 'g')
    %%
    r_cloud    =   raster2DANM(cloud, 0.1); 
    
    imshow(r_cloud.nPoints_image);
%     %%
    I       =   r_cloud.nPoints_image;
    I(I>0)  =   1;
%     cc      =   bwconncomp(I,8);
%     
%     cluster_idx = cell(numel(cc.PixelIdxList),1);
%     for i = 1:numel(cc.PixelIdxList)
%         aux_id      =   ismember(r_cloud.indices, cc.PixelIdxList{i});
%         aux_id_2    =   cell2mat(r_cloud.parent_idx(aux_id)); %ids en cloud
%         cluster_idx{i}     =   (aux_id_2);
%         
%         bin = I(cluster_idx{i});  % Binary test image
%         R   = rand(1,1);  % Value in range [0, 1]
%         G   = rand(1,1);
%         B   = rand(1,1);
%         RGB = cat(3, bin * R, bin * G, bin * B);
%         hold on;
%         imshow(RGB);
%         
%     end
%% se = strel('rectangle',[15 50]);
% BW = imdilate(I,se);

%Opening to merge the rails
se = strel('square',10);
BW = imclose(I,se);
imshow(BW);


%% Find the edges in the image using the edge function.
% BW1 = edge(BW,'Canny');
% imshow(BW);
% 
% % 
% % 
% BW2 = edge(BW,'Prewitt');
% % 
% imshowpair(BW1,BW2,'montage')
% 
% Compute the Hough transform of the binary image returned by edge.
[H,theta,rho]   =   hough(BW);

% figure; imshow(imadjust(rescale(H)),[],'XData',theta,'YData',rho,...
% 'InitialMagnification','fit'); xlabel('$\theta$ (deg)'); ylabel('$\rho$')
% axis on; axis normal; hold on; colormap(gca,hot)


%% Find the peaks in the Hough transform matrix, H, using the houghpeaks function.

P = houghpeaks(H,20,'threshold',ceil(0.3*max(H(:))));

% Find lines in the image using the houghlines function. 
lines = houghlines(BW,theta,rho,P,'FillGap',1,'MinLength',20);

figure, imshow(I), hold on
for k = 1:length(lines)
   xy = [lines(k).point1; lines(k).point2];
   plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');

   % Plot beginnings and ends of lines
   plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
   plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');
end 
    %
    %% 
%      1. Hacer hughes 
%      2. quedarme con la línea horz de la catenaria como la que está entre
%           las líneas de los raíles
%      3. Ecoger de las restantes, la más cercana a la catenaria y que pase
%           por un poste