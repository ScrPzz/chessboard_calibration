# Chessboard calibration software

This code features functions that allows you to load a raw full-frame 1280x800 calibration image like this:

![](/images/right_tilted.bmp)



and extract its features to create a linearization grid.
The images i'm using for this tutorial are manually tilted to enhance the property of the code to manage with images taken with not optimally positioned imagers.


## Align routine

The functions grouped in **align** prepares the image for the features extraction that will be operated by the **calib** routine

1. Cutting off the black regions above and beyond the proper chessboard image



![](/images/right_tilted_cropped.BMP)





3. Extracting the so called "crosshair" and other _geometric_ features from the image. 







5. Write the image to be processed by the features extraction routine.

