# Chessboard calibration software

This code features functions that allows you to load a raw full-frame 1280x800 calibration image like this:

![](/images/right_tilted.bmp)



and extract its features to create a linearization grid.
The images i'm using for this tutorial are manually tilted to enhance the property of the code to manage with images taken with not optimally positioned imagers.


## Align routine

The functions grouped in **align** prepares the image for the features extraction that will be operated by the **calib** routine


1. Extracting the so called "crosshair" and other _geometric_ features from the image. 


![](/images/RIGHT_screenshot_02.03.2021.png)


functions are: **extract_horizontal_black_line** and **extract_vertical_black_line**.


2. Cutting off the black regions above and beyond the proper chessboard image:


![](/images/right_tilted_cropped.BMP)


for my personal purpose the cut image has to be 1000x300, but you can easily change that dimensions on **adaptive_cut** function.



3. Write the image to be processed by the features extraction routine.




## Calib routine


That routine is dedicated to corners position extraction. The main function is GoodFeaturesToTrack from the OpenCV library, not much to say about the fantastic function. 

Corners extracted with GoodFeaturesToTrack are then filtered with some functions i wrote.

1. **dust_filter**: Filter corners not on the edges of the image with a control on the medium color of pixels contained on the (n,n) block centered on the corner.


 2. **baricentroids_filter**: this function construct 2 rectangles centered on the corner position, one for each direction (N-S, E-W). Those areas are used to check other corners positions. No corner is allowed to live on those areas.

3. **top_border_filter**: Filter corners closer that a distance (maxdistance) to the top white line. Euclidean norm used.


Then the routine works on the reconstructed position of corners usind "deltas" used early to construct the green line on the second point of align routine. That is useful to assign every corner to a square's raw.


That's the final positions:





![](/images/Final_corner_positions_screenshot.png)



Top-right there's a corner that has not been filterd, i know. I'm still working on the code and i'll update the results.


