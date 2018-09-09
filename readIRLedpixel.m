
function signal = readIRLedpixel(position,filename)

%Loading all the necessary files
if isequal(filename(end-3:end),'.mat')
    vidfile = sprintf('%s',filename(1:end-4),'.mp4');
else
    vidfile = sprintf('%s',filename,'.mp4');
end
disp('Loading video file...')
vidobj = VideoReader(vidfile); %Load the video file
n = vidobj.NumberOfFrames;
%signal = zeros(1,n);

whiskMov = zeros(vidobj.Height,vidobj.Width,3,'uint8');

percDone = 0;
if vidobj.FrameRate > 125
    ds = 1;%round(vidobj.FrameRate/125);
else
    ds = 1;
end
a=1;
disp('Getting IR LED pixel value...')
disp(sprintf('%s',num2str(percDone),'% done...'))
for i = 1:ds:n
    percDone = floor(100*(i/n));
    percDoneLast = floor(100*((i-ds)/n));
    if isequal(percDone,percDoneLast) == 0
        disp(sprintf('%s',num2str(percDone),'% done...'))
    end
    
    whiskMov = read(vidobj,i);
    signal(a) = mean(whiskMov(position(2),position(1),:));%whiskMov(position(2),position(1),1); %
    a=a+1;
%     figure(1)
%     image(whiskMov)
%     hold on
%     plot(position(2),position(1),'.g')
%     hold off
end

IRledSignal = signal;

figure
plot(IRledSignal)

if isequal(filename(end-3:end),'.mat')
    f = filename;
else
    f = [filename '.mat'];
end

if exist(f,'file')
    save(f,'IRledSignal','-append')
else
    save(f,'IRledSignal')
end

end