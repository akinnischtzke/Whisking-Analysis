function traceAnalysis_new(filename,parameters,h,vidobj,measurements) %Use filename

% December 1, 2017 AK: This is the most recent version of program to
% analyze the output from 'whiski' to calculate whisking from mouse whisker video. Before running
% this, run the analyzeWhiskersSetup program to obtain the appropriate
% parameters.

%Input arguments:
%Filename - enter the filename (with or without the extension) for the whisker
%video. All variables will get saved in a .mat file with the same name

%parameters: should have already run the "analyzeWhiskersSetup" which will
%generate a .mat file containing a structure with necessary parameters. Load
%this file and pass the parameters structure as an input. 

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global nFrames faceEdgeX faceEdgeY faceAngle xThresh1 yThresh1 xThresh2 yThresh2 framesPlot

%Load necessary files & videos
switch nargin
    case 2
        h = figure;
        loadFiles()
    case 3
        loadFiles()
end
%Load necessary variables
%loadFiles()
%loadParams(parameters)

%Initialize variables
nFrames = floor(vidobj.Duration*vidobj.FrameRate);
vidobj.CurrentTime = 0;%1/vidobj.FrameRate; %reset video back to first frame
ind=1;
frame = 0;
plotFrames = 0; %Not going to plot any output, since you can do that using the setupGUI

whiskerPosition_median = zeros(1,nFrames);
IRledSignal = zeros(1,nFrames);


%Remove any filename extension if exists & check for mat file
if isequal(filename(end-3),'.'), filename = filename(1:end-4);end
if ~exist([filename '.mat'],'file'), error('Need to run "analyzeWhiskersSetup" first to get parameter values'), end

%Analyze whisker tracking information by frame
while hasFrame(vidobj)
    
    frame = frame+1; percCounter(frame,1,nFrames) %keeps track of progress
    
    %Get the current frame
    im = readFrame(vidobj);
    
    %Get all traced objects in the current frame
    [indList,ind] = getInds(frame-1,ind,measurements);
    
    %Goes through each object in this frame for analysis; returns median
    %angle per frame & coordinates of objects for plotting
    [whiskerPosition_median(frame),isWhisker,fX,fY,tX,tY] = checkWhiskerObjs(indList,parameters,measurements); 
     
    %Plot frame output (at least for first few hundred frames)
    plotOutput(im,parameters,isWhisker,fX,fY,tX,tY,frame,plotFrames,h)
    
    %Save the mean value of the frame to track IR led flashes
    IRledSignal(frame) = readIRLedpixel(im);
    
end

[whiskingVarSR,whiskingPower,whiskingSmPower] = whiskerFinalAnalysis(whiskerPosition_median);
plotFinalOutput(whiskerPosition_median,whiskingVarSR,[whiskingPower;whiskingSmPower])

disp('Saving .mat file...')
save(filename,'IRledSignal','whiskerPosition_median','whiskingVarSR','whiskingPower','whiskingSmPower','-append')

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function loadParams(p)
        xThresh1 = p.xThresh1;
        xThresh2 = p.xThresh2;
        yThresh1 = p.yThresh1;
        yThresh2 = p.yThresh2;
        faceEdgeX = p.faceEdgeX;
        faceEdgeY = p.faceEdgeY;
        faceAngle = p.faceAngle;
        framesPlot = p.framesPlot;
        
        nFrames = vidobj.Duration*vidobj.FrameRate;
    end

    function loadFiles()
        %Loading video file
        disp('Loading video file...')
        vidobj = VideoReader([filename '.mp4']); %Load the video file
        %Load measurements file
        measurements = LoadMeasurements([filename '.measurements']);

    end

    function signal = readIRLedpixel(im2)
        signal = mean(im2(:));
    end

    function [wVarSR,wPower,wSmPower] = whiskerFinalAnalysis(whiskerPosition_median)
        
        w = fillmissing(whiskerPosition_median,'linear'); %interpolate any NaNs
        
        %Calculate the variance within a sliding window
        wVarSR = zeros(size(w));
        binSize = 20;
        for i = 1:length(w) - binSize
            wVarSR(i) = var(w(i:i+binSize));
        end
        wVarSR = sqrt(wVarSR);
        
        %Filter the angle data and compute hilbert transform to get the
        %whisking 'power'
        filtXdata = filterData(w,[0.01 25],125);
        [~,wPower,wSmPower] = hilbertT_plot(filtXdata);
    end

    function plotFinalOutput(w1,w2,w3)
        figure(1)
        ax(1) = subplot(3,1,1);
        plot(w1,'-b')
        axis tight
        title('Median whisker angle per frame (deg)')
        
        subplot(3,1,2)
        plot(w2,'-r')
        axis tight
        title('whisking std dev across frames')
        
        subplot(3,1,3)
        plot(w3(1,:),'-g'), hold on, plot(w3(2,:),'-m')
        axis tight
        title('whisking power')
    end

end


