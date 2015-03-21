# Introduction #

The software includes a demo simulation in the form of a sequence of pre-recorded images, but also if you have a webcam and a couple of pieces of paper you can run the system live.  To run the system from a webcam first of all you'll need to make a calibration target, which isn't at all complicated.  The target is used in order to detect some initial features at a known distance from the camera, so that it has some idea of the scale of things.  Without this initial information the system has no idea what the real size of objects are - they could be small and close or large and far away.

Take an A4 sheet of paper and fold it in half.  Take one half of the folded sheet and colour it black (I used a thick black marker pen which I usually use for writing on CDs), then stick it onto the centre of another piece of A4.  Your target is now ready.

Run the software, then from the Video menu select Run Camera.  You will need to select which camera you are going to be using from the drop down list.  I happen to be using an old Quickcam Express webcam.  It's about the cheapest nastiest webcam you can obtain, giving a low quality 320x240 image from lossy USB1.1 compression.  If MonoSLAM works with this, it will work with anything.  After clicking Ok the camera will begin running, then from the Tools menu select Calibration.  You'll see something like the following:

![http://sluggish.uni.cc/monoSLAM/monoSLAM_03.jpg](http://sluggish.uni.cc/monoSLAM/monoSLAM_03.jpg)

Position your camera looking directly at the target and measure the distance between them.  This should be anything up to about a metre away.  Enter values for the distance to the target and the field of vision of the camera.  Almost all webcams have a 40 degrees horizontal field of vision, from the far left to the far right of the image.  Press ENTER after changing each value.  If your camera's default resolution is 320x240 you shouldn't need to alter the centre of distortion values.  Now with the camera pointing at the target alter the lens distortion value until the four crosshairs are aligned with the corners of the black square.  When that's complete press Done and the system is ready for action.  This procedure only needs to be carried out once, or whenever you first use a different camera.

Now it's time to see what this system can do.  Point the camera at the target and align the crosshairs (or blobs) with each corner of the black square.  When the system detects that each corner is correctly aligned for a minimum duration of one second then it will automatically begin mapping, with a "tracking" message appearing at the bottom of the screen.  If for any reason it does not begin tracking you can force it to track by clicking the begin tracking button.  Move the camera slowly and smoothly and you should notice that detected features are reliably tracked.

![http://sluggish.uni.cc/monoSLAM/monoSLAM_04.jpg](http://sluggish.uni.cc/monoSLAM/monoSLAM_04.jpg)

The performance of MonoSLAM is limited mainly by processor speed and the frame rate of the camera.  The higher the frame rate the better.  Less processor resources are used if you have a high frame rate, because the areas within which it searches for features become increasingly small and well focussed.  This version also does not presently include the patch warping feature from Davison's more recent work, which makes the feature tracking much more reliable, but it at least shows the principle of the approach.

![http://sluggish.uni.cc/monoSLAM/monoSLAM_02.jpg](http://sluggish.uni.cc/monoSLAM/monoSLAM_02.jpg)