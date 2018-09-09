
function p = whiskingAnalysis_objMotion(vidobj)

plotFig = 'n';
nFrames = vidobj.Duration*vidobj.FrameRate;
vidobj.CurrentTime = 0;

a = 1;

currFrame = rgb2gray(readFrame(vidobj));
nextFrame = rgb2gray(readFrame(vidobj));
d = nextFrame-currFrame;
dFrame = d;

if plotFig == 'y'; figure; end
while hasFrame(vidobj)
    
    a=a+1;
    percCounter(a,1,nFrames)
    
    currFrame = nextFrame;
    nextFrame = rgb2gray(readFrame(vidobj));
    temp = (nextFrame-currFrame);
    d = temp;
    
    d = reshape(d,[numel(d),1]);
    thresh = 15;%quantile(d,0.98)
    %d(d<thresh) = 0;
    %d = imbinarize(d);
    d2 = reshape(d,[size(currFrame,1),size(currFrame,2)]);
    p(a) = sum(d ~= 0);
    
    if plotFig == 'y'
        subplot(2,1,1),image(temp),set(gca,'YDir','reverse'), title(a), axis square, axis off
        subplot(2,1,2),imshow(d2),axis square, set(gca,'YDir','reverse'), title('binary image')
        drawnow
    end
end
p(1) = p(2);
figure;plot(p);axis tight
ylabel('sum moving pixels')
xlabel('frame #')
