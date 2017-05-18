function [line_d] = interp_data(data,condition,xd)
    line=data(:,condition*2-1:condition*2);
    line(~any(~isnan(line), 2),:)=[];
    
    min_index=line(1,1);
    max_index=line(length(line),1);
    avg_value=mean(line(:,2));
        
    line_d=interp1(line(:,1),line(:,2),xd,'spline');
    
    for i=1:length(xd)
        if xd(i)<min_index || xd(i)>max_index || avg_value*line_d(i)<0
            line_d(i)=0;
        end
    end