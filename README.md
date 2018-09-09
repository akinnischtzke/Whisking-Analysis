# Whisking-Analysis
MATLAB code for tracking mouse whisker videos

This repository contains MATLAB code for analysis of mouse whisking videos. This analysis can be used for any videos where the animal is stationary, and will work for freely whisking or videos where whiskers are coming into contact with objects.

This code is the final step of the analysis pipeline. The entire pipeline right now is 1) collect the videos (using Sony PS3eye camera), 2) process through whiski (https://wiki.janelia.org/wiki/display/MyersLab/Whisker+Tracking) to 'trace' putative whisker objects in each frame, then 3) run this GUI to set specific parameters to filter out traced objects that are not whiskers. This program will also save variables that can be used for further analysis, such as the median whisker angle per frame or whisking power per frame.

Next stages after this progam will be to combine this whisking time-series data with the other information collected during the experiment, such as possibly neural data or other behavioral data.

To run:

- Run the GUI interface (analyzeWhiskersSetup_gui_v2.m), with the inferface it will guide you through each step. 
- The main analysis script is 'traceAnalysis_new.m', check this if interested to understand how the analysis is working.



