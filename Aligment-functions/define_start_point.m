function start_pt = define_start_point(nTips)


    condition = ones(length(nTips)+1,1);

    filter = [];
    while sum(filter) == 0
        condition   =   ones(1, length(condition)-1);
        filter_1    =   (strfind(nTips(:,1)', condition));    
        filter_2    =   (strfind(nTips(:,2)', condition)); 
        filter      =   ismember(filter_1, filter_2);
    end
    
        filter      =   find(filter); 
        start_pt    =   filter_1(filter(1)); 
        start_pt    =   start_pt(1) + ceil(length(condition) / 2); 
    
%     try
%         % Find a section with 2 tips and where the surrounding also have 2 tips
%         filter_1    =   (strfind(nTips(2:end-2,1)', [2 1 1 1 1 1 1]));    
%         filter_2    =   (strfind(nTips(2:end-2,2)', [2 1 1 1 1 1 1])); 
%         filter      =   ismember(filter_1, filter_2);
%         filter      =   find(filter); 
%         start_pt    =   filter_1(filter(1)); 
%         start_pt    =   start_pt(1) + 2; 
%     catch
%         try
%             % Find a section with 2 tips and where the surrounding also have 2 tips
%             filter_1    =   (strfind(nTips(2:end-2,1)', [1 1 1]));    
%             filter_2    =   (strfind(nTips(2:end-2,2)', [1 1 1])); 
%             filter      =   ismember(filter_1, filter_2);
%             % start_pt     =   find(filter); 
%             start_pt     =   filter_1(filter);
%             start_pt     =   start_pt(1) + 1; 
% 
%         catch
%             try
%                 % Find a section with 2 tips and where the surrounding also have 2 tips
%                 filter_1    =   (strfind(nTips(2:end-2,1)', [1 1]));    
%                 filter_2    =   (strfind(nTips(2:end-2,2)', [1 1])); 
%                 filter      =   ismember(filter_1, filter_2);
%                 % start_pt     =   find(filter); 
%                 start_pt     =   filter_1(filter);
%                 start_pt     =   start_pt(1) + 1; 
%             catch
%                 % Find a section with 2 tips and where the surrounding also have 2 tips
%                 filter_1    =   (strfind(nTips(2:end-2,1)', 1));    
%                 filter_2    =   (strfind(nTips(2:end-2,2)', 1)); 
%                 filter      =   ismember(filter_1, filter_2);
%                 % start_pt     =   find(filter); 
%                 start_pt     =   filter_1(filter);
%                 start_pt     =   start_pt(1) + 1; 
%             end
%         end
%     end
%     start_pt = start_pt + 1;


%%%%%%%%%%%%%%%%%%% 

%     one_peak_pts_1  =   (nTips(:,1) == 1);
%     one_peak_pts_2  =   (nTips(:,2) == 1);
%     
%     no_peak_pts_1   =   [find(~one_peak_pts_1);length(one_peak_pts_1)];
%     no_peak_pts_2   =   [find(~one_peak_pts_2);length(one_peak_pts_2)];
%     
%     length_ones_1   =   length(one_peak_pts_1);
%     length_ones_2   =   length(one_peak_pts_2);
%     
%     min_pt_1(1)  =   0;
%     if ~isempty(no_peak_pts_1)
%         for i = 1:length(no_peak_pts_1)
%             length_i  =   no_peak_pts_1(i) - min_pt_1(i);
%             if length_i ~=0 && length_i < length_ones_1
%                 length_ones_1   =   length_i;
%             end
%             min_pt_1(i+1)  =    no_peak_pts_1(i);
%         end
%     end
% 
%     
%     min_pt_2(1)  =   0;
%     if ~isempty(no_peak_pts_2)
%         for i = 1:length(no_peak_pts_2)
%             length_i  =   no_peak_pts_2(i) - min_pt_2(i);
%             if length_i ~=0 
%                 length_i_2(i)   =   length_i;
%             end
%             min_pt_2(i+1)  =    no_peak_pts_2(i);
%         end
%     end
%     
%     start_i_1   =   max(


end