# Whisking-Analysis
MATLAB code for tracking mouse whisker videos

This repository contains MATLAB code for analysis mouse whisking videos. This analysis can be used for any videos where the animal is stationary, and will for for freely whisking or videos where objects are made on objects.

This code is the final step of the analysis pipeline. The entire pipeline right now is 1) collect the videos (using Sony PS3eye camera), 2) process through whiski (https://wiki.janelia.org/wiki/display/MyersLab/Whisker+Tracking) to 'trace' putative whisker objects in each frame, then 3) run this GUI to set specific parameters to filter out traced objects that are not whiskers. This program will also save variables that will be used for further analysis, such as the median whisker angle per frame or whisking power per frame.

Next stages after this progam will be to combine this whisking time-series data with the other information collected during the experiment, such as possibly neural data or other behavioral data.



