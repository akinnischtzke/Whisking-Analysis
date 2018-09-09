function plotOutput(im,p,isWhisker,fx,fy,tx,ty,frame,framesPlot,h)

if frame < framesPlot
    %cla(h)
    %axes(h)
    
    image(h,im), hold(h,'on')
    title(h,sprintf('%s','Frame ',num2str(frame)))
    w = p.xThresh1 - p.xThresh2;
    he = p.yThresh2 - p.yThresh1;
    rectangle(h,'Position',[p.xThresh2 p.yThresh1 w he],'EdgeColor','b')
    line(h,[p.faceEdgeX(1),p.faceEdgeX(end)],[p.faceEdgeY(1) p.faceEdgeY(end)],'Color','y')
    
    line(h,[fx(isWhisker == 1)' tx(isWhisker == 1)']',[fy(isWhisker == 1)' ty(isWhisker == 1)']','Color','g') %Kept whisker objects
    %plot(h,[fx(isWhisker == 1)' tx(isWhisker == 1)']',[fy(isWhisker == 1)' ty(isWhisker == 1)']','.b','MarkerSize',20)
    line(h,[fx(isWhisker == 0)' tx(isWhisker == 0)']',[fy(isWhisker == 0)' ty(isWhisker == 0)']','Color','r') %Rejected whisker objects
    
    drawnow
    hold(h,'off')
end
end