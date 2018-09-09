function [indList,b] = getInds(f,b,measurements)
if isempty(b)
    b=1;
    while measurements(b).fid < f
        b = b+1;
    end
    b=b+1;
end
first = b;
while measurements(b).fid == f && (b < length(measurements))
    b = b+1;
end
last = b-1;
indList = first:last;
end