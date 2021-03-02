# Chessboard calibration software

This code features functions that allows you to load a raw full-frame 1280x800 calibration image like this:

![Raw calib img](/images/raw_right.bmp)



and extract its features to create a linearization grid.


## Align routine

The functions grouped in **align** prepares the image for the features extraction that will be operated by the **calib** routine

1. Cutting off the black regions above and beyond the proper chessboard image






3. Extracting the so called "crosshair" and other _geometric_ features from the image. 







5. Write the image to be processed by the features extraction routine.

