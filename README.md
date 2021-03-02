# Chessboard calibration software

This code features functions that allows you to load a full-frame 1280x800 calibration image like this:





and extract its features to create a linearization grid.


The **align** fucntion prepares the image for the features extraction:

1. Cutting off the black regions above and beyond the proper chessboard image
2. Extracting the so called "crosshair" and other _geometric_ features from the image. 
3. Write the image to be processed by the features extraction routine.

