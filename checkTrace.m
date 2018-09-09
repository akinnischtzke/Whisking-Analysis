function var = checkTrace(w,p)

%INPUT:
% fx & fy: x and y positions for follicle point of traced object
% tx & ty: x and y positions for tip point of traced object
% xThresh1,yThresh1,xThresh2,yThresh2: ROI boundaries
% faceEdgeX & faceEdgeY: line for the edge of the face
% minFollicleDistance: distance from the follicle point on the object to closest point on the face
% faceAngle: angle of the mouse's face
% whiskAngle: angle of the traced object

%OUTPUT:
%var = set to 1 (is whisker object) or 0 (not whisker object). Default is
%set to 1, but value is changed to 0 if any of the following criteria are
%not true

var = 1;

% %Checks that at least 2/3 of the traced object is within the chosen
% %quadrant for whiskers.
% if (sum(xp < xThresh2) + sum(xp > xThresh1)) >= (length(xp)/2)
%     var = 0;
% elseif (sum(yp > yThresh2) + sum(yp < yThresh1)) >= length(yp)/2
%     var = 0;
% end

if p.xThresh1 > p.xThresh2
    dir = 'right';
else
    dir = 'left';
end

%Determine which way the face is oriented
if p.yThresh1 > p.yThresh2
    dir2 = 'up';
else
    dir2 = 'down';
end


%Checks if follicle point is within ROI
if isequal(dir2,'down')
    if isequal(dir,'right')
        if (w.fx > p.xThresh1) || (w.fy > p.yThresh2) || (w.fy < p.yThresh1)
            var = 0;
        end
    else
        if (w.fx < p.xThresh1) || (w.fy > p.yThresh2)
            var = 0;
        end
    end
else
    if isequal(dir,'right')
        if (w.fx > p.xThresh1) || (w.fy > p.yThresh1)
            var = 0;
        end
    else
        if (w.fx < p.xThresh1) || (w.fy > p.yThresh1)
            var = 0;
        end
    end
end

%Sets a minimum length that object has to exceed to be considered a whisker
wlength = sqrt((w.tx - w.fx)^2 + (w.ty - w.fy)^2); %Length of current traced object
if abs(wlength) <= p.minLength
    var = 0;
end

%Sets a threshold such that the 'follicle' of the traced object has to be x
%distance from the face edge
minFollicleDistance = findFollicle_b(w.fx,w.fy,p.faceEdgeX,p.faceEdgeY);

if minFollicleDistance > p.minDistance
    var = 0;
end

%Checks if the angle is greater than 50 degrees from the median angle on
%the last frame. Helps (but doesn't totally eliminate) objects that aren't
%whiskers but in the whisker field
if ~isempty(p.medAngleLast)
    if abs(w.angle - p.medAngleLast) > 70
        var = 0;
    end
end
    
end