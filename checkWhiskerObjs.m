function [whiskerPosition_median,isWhisker,fX,fY,tX,tY] = checkWhiskerObjs(iList,p,measurements)

whiskerAngles = zeros(1,length(iList));
isWhisker = ones(1,length(iList));
fX = zeros(1,length(iList));
fY = zeros(1,length(iList));
tX = zeros(1,length(iList));
tY = zeros(1,length(iList));

for j = 1:length(iList)
    t = iList(j);
    
    %Find the 'follicle' point, which is the closest point on the
    %whisker to the face edge
    fX(j) = measurements(t).follicle_x;
    fY(j) = measurements(t).follicle_y;
    tX(j) = measurements(t).tip_x;
    tY(j) = measurements(t).tip_y;
    whiskerAngles(j) =  p.faceAngle + abs(measurements(t).angle) + 90;
    
    %Check each traced object and determine if potential whisker or
    %not; returns '0' if not whisker and '1' if it is
    whiskerInfo = struct('fx',fX(j),'fy',fY(j),'tx',tX(j),'ty',tY(j),'angle',whiskerAngles(j));
        
    isWhisker(j) = checkTrace(whiskerInfo,p);
end
%Just return the median angle per frame to keep
whiskerPosition_median = median(whiskerAngles(isWhisker == 1));
end