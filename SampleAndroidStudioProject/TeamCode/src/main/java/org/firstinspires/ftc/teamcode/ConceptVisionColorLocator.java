/*
 * Copyright (c) 2024 Phil Malone
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.ftc6448.simulator.PlatformSupport;
import org.opencv.core.RotatedRect;

import java.util.List;
import java.util.Locale;
import java.util.stream.Collectors;
import java.util.stream.Stream;

/*
 * This OpMode illustrates how to use a video source (camera) to locate specifically colored regions
 *
 * Unlike a "color sensor" which determines the color of an object in the field of view, this "color locator"
 * will search the Region Of Interest (ROI) in a camera image, and find any "blobs" of color that match the requested color range.
 * These blobs can be further filtered and sorted to find the one most likely to be the item the user is looking for.
 *
 * To perform this function, a VisionPortal runs a ColorBlobLocatorProcessor process.
 *   The ColorBlobLocatorProcessor process is created first, and then the VisionPortal is built to use this process.
 *   The ColorBlobLocatorProcessor analyses the ROI and locates pixels that match the ColorRange to form a "mask".
 *   The matching pixels are then collected into contiguous "blobs" of pixels.  The outer boundaries of these blobs are called its "contour".
 *   For each blob, the process then creates the smallest possible rectangle "boxFit" that will fully encase the contour.
 *   The user can then call getBlobs() to retrieve the list of Blobs, where each Blob contains the contour and the boxFit data.
 *   Note: The default sort order for Blobs is ContourArea, in descending order, so the biggest contours are listed first.
 *
 * To aid the user, a colored boxFit rectangle is drawn on the camera preview to show the location of each Blob
 * The original Blob contour can also be added to the preview.  This is helpful when configuring the ColorBlobLocatorProcessor parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

//@Disabled
@TeleOp(name = "Concept: Vision Color-Locator", group = "Concept")
public class ConceptVisionColorLocator extends LinearOpMode
{
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    /**
     * The variables to store our instances of the ColorBlobLocator processors.
     */
    private ColorBlobLocatorProcessor redColorLocator;
    private ColorBlobLocatorProcessor blueColorLocator;
    private ColorBlobLocatorProcessor yellowColorLocator;
    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    protected final ElapsedTime runtime = new ElapsedTime();
    protected int cycleCount;
    protected double fps;

    @Override
    public void runOpMode()
    {
        initColorLocators();

        telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();
        runtime.reset();

        //use PlatformSupport method from SimulatorSupport.jar to get the time
        //on robot, this maps to System.currentTimeMillis()
        //in simulator, it maps to elapsed simulator time
        long startTime = PlatformSupport.getCurrentTimeMillis();
        int frameCount = 0;
        boolean lastX = false;
        int capFrameCount = 0;
        long capReqTime = 0;

        // WARNING:  To be able to view the stream preview on the Driver Station, run this code in INIT mode - remove waitForStart above and uncomment below:
        while (opModeIsActive()) // || opModeInInit()
        {
            // Read the current list
            List<ColorBlobLocatorProcessor.Blob> redBlobs = redColorLocator.getBlobs();
            List<ColorBlobLocatorProcessor.Blob> blueBlobs = blueColorLocator.getBlobs();
            List<ColorBlobLocatorProcessor.Blob> yellowBlobs = yellowColorLocator.getBlobs();

            /*
             * The list of Blobs can be filtered to remove unwanted Blobs.
             *   Note:  All contours will be still displayed on the Stream Preview, but only those that satisfy the filter
             *          conditions will remain in the current list of "blobs".  Multiple filters may be used.
             *
             * Use any of the following filters.
             *
             * ColorBlobLocatorProcessor.Util.filterByArea(minArea, maxArea, blobs);
             *   A Blob's area is the number of pixels contained within the Contour.  Filter out any that are too big or small.
             *   Start with a large range and then refine the range based on the likely size of the desired object in the viewfinder.
             *
             * ColorBlobLocatorProcessor.Util.filterByDensity(minDensity, maxDensity, blobs);
             *   A blob's density is an indication of how "full" the contour is.
             *   If you put a rubber band around the contour you would get the "Convex Hull" of the contour.
             *   The density is the ratio of Contour-area to Convex Hull-area.
             *
             * ColorBlobLocatorProcessor.Util.filterByAspectRatio(minAspect, maxAspect, blobs);
             *   A blob's Aspect ratio is the ratio of boxFit long side to short side.
             *   A perfect Square has an aspect ratio of 1.  All others are > 1
             */
            ColorBlobLocatorProcessor.Util.filterByArea(50, 500000, redBlobs);  // filter out very small blobs.
            ColorBlobLocatorProcessor.Util.filterByArea(50, 500000, blueBlobs);  // filter out very small blobs.
            ColorBlobLocatorProcessor.Util.filterByArea(50, 500000, yellowBlobs);  // filter out very small blobs.

            /*
             * The list of Blobs can be sorted using the same Blob attributes as listed above.
             * No more than one sort call should be made.  Sorting can use ascending or descending order.
             *     ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);      // Default
             *     ColorBlobLocatorProcessor.Util.sortByDensity(SortOrder.DESCENDING, blobs);
             *     ColorBlobLocatorProcessor.Util.sortByAspectRatio(SortOrder.DESCENDING, blobs);
             */

            cycleCount++;
            frameCount++;

            long curTime=PlatformSupport.getCurrentTimeMillis();
            if (frameCount >=10) {
                fps=(frameCount/((curTime-startTime)/1000.0));
                startTime=curTime;
                frameCount=0;
            }

            boolean x = gamepad1.x;
            if (x && !lastX)
            {
                visionPortal.saveNextFrameRaw(String.format(Locale.US, "CameraFrameCapture-%06d", capFrameCount++));
                capReqTime = System.currentTimeMillis();
            }
            lastX = x;
            telemetry.addLine(" > Press X (or Square) to capture a frame");
            telemetry.addData(" > Camera Status", visionPortal.getCameraState());
            if (capReqTime != 0) {
                telemetry.addLine("\nCaptured Frame!");
            }
            if (capReqTime != 0 && curTime - capReqTime > 1000) {
                capReqTime = 0;
            }

            //add telemetry.  to see telemetry in Webots, right click on your robot and select "Show Robot Window"
            telemetry.addData("FPS", "Loop speed - "+fps+" at elapsed time "+runtime.toString());

            telemetry.addLine(" Area Density Aspect  Center  Size  Angle");

            // Display the size (area) and center location for each Blob.
            for(ColorBlobLocatorProcessor.Blob b : redBlobs)
            {
                RotatedRect boxFit = b.getBoxFit();
                telemetry.addLine(String.format("Red: %5d  %4.2f   %5.2f  (%3d,%3d)  (%3d,%3d)  %.2f",
                          b.getContourArea(), b.getDensity(), b.getAspectRatio(),
                        (int) boxFit.center.x, (int) boxFit.center.y,
                        (int) boxFit.size.width, (int) boxFit.size.height, boxFit.angle));
            }
            for(ColorBlobLocatorProcessor.Blob b : blueBlobs)
            {
                RotatedRect boxFit = b.getBoxFit();
                telemetry.addLine(String.format("Blue: %5d  %4.2f   %5.2f  (%3d,%3d)  (%3d,%3d)  %.2f",
                        b.getContourArea(), b.getDensity(), b.getAspectRatio(),
                        (int) boxFit.center.x, (int) boxFit.center.y,
                        (int) boxFit.size.width, (int) boxFit.size.height, boxFit.angle));
            }
            for(ColorBlobLocatorProcessor.Blob b : yellowBlobs)
            {
                RotatedRect boxFit = b.getBoxFit();
                telemetry.addLine(String.format("Yellow: %5d  %4.2f   %5.2f  (%3d,%3d)  (%3d,%3d)  %.2f",
                        b.getContourArea(), b.getDensity(), b.getAspectRatio(),
                        (int) boxFit.center.x, (int) boxFit.center.y,
                        (int) boxFit.size.width, (int) boxFit.size.height, boxFit.angle));
            }

            telemetry.update();
//            sleep(50);

            //on the real robot, this method call does nothing
            //on the simulator, it forces the opmode to sync its loop to Webots simulator time steps
            PlatformSupport.waitForSimulatorTimeStep();
        }
    }

    /* Build a "Color Locator" vision processor based on the ColorBlobLocatorProcessor class.
     * - Specify the color range you are looking for.  You can use a predefined color, or create you own color range
     *     .setTargetColorRange(ColorRange.BLUE)                      // use a predefined color match
     *       Available predefined colors are: RED, BLUE YELLOW GREEN
     *     .setTargetColorRange(new ColorRange(ColorSpace.YCrCb,      // or define your own color match
     *                                           new Scalar( 32, 176,  0),
     *                                           new Scalar(255, 255, 132)))
     *
     * - Focus the color locator by defining a RegionOfInterest (ROI) which you want to search.
     *     This can be the entire frame, or a sub-region defined using:
     *     1) standard image coordinates or 2) a normalized +/- 1.0 coordinate system.
     *     Use one form of the ImageRegion class to define the ROI.
     *         ImageRegion.entireFrame()
     *         ImageRegion.asImageCoordinates(50, 50,  150, 150)  100x100 pixel square near the upper left corner
     *         ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5)  50% width/height square centered on screen
     *
     * - Define which contours are included.
     *     You can get ALL the contours, or you can skip any contours that are completely inside another contour.
     *        .setContourMode(ColorBlobLocatorProcessor.ContourMode.ALL_FLATTENED_HIERARCHY)  // return all contours
     *        .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)            // exclude contours inside other contours
     *        note: EXTERNAL_ONLY helps to avoid bright reflection spots from breaking up areas of solid color.
     *
     * - turn the display of contours ON or OFF.  Turning this on helps debugging but takes up valuable CPU time.
     *        .setDrawContours(true)
     *
     * - include any pre-processing of the image or mask before looking for Blobs.
     *     There are some extra processing you can include to improve the formation of blobs.  Using these features requires
     *     an understanding of how they may effect the final blobs.  The "pixels" argument sets the NxN kernel size.
     *        .setBlurSize(int pixels)    Blurring an image helps to provide a smooth color transition between objects, and smoother contours.
     *                                    The higher the number of pixels, the more blurred the image becomes.
     *                                    Note:  Even "pixels" values will be incremented to satisfy the "odd number" requirement.
     *                                    Blurring too much may hide smaller features.  A "pixels" size of 5 is good for a 320x240 image.
     *        .setErodeSize(int pixels)   Erosion removes floating pixels and thin lines so that only substantive objects remain.
     *                                    Erosion can grow holes inside regions, and also shrink objects.
     *                                    "pixels" in the range of 2-4 are suitable for low res images.
     *        .setDilateSize(int pixels)  Dilation makes objects more visible by filling in small holes, making lines appear thicker,
     *                                    and making filled shapes appear larger. Dilation is useful for joining broken parts of an
     *                                    object, such as when removing noise from an image.
     *                                    "pixels" in the range of 2-4 are suitable for low res images.
     */
    private void initColorLocators() {
        // Create the ColorBlobLocator processor.
        ColorBlobLocatorProcessor.Builder ColorBlobLocator = new ColorBlobLocatorProcessor.Builder()
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.entireFrame())
//                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5))  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5);                               // Smooth the transitions between different colors in image

        // use a predefined color match
        redColorLocator = ColorBlobLocator.setTargetColorRange(ColorRange.RED).build();
        blueColorLocator = ColorBlobLocator.setTargetColorRange(ColorRange.BLUE).build();
        yellowColorLocator = ColorBlobLocator.setTargetColorRange(ColorRange.YELLOW).build();

        /*
         * Build a vision portal to run the Color Locator process.
         *
         *  - Add the colorLocator process created above.
         *  - Set the desired video resolution.
         *      Since a high resolution will not improve this process, choose a lower resolution that is
         *      supported by your camera.  This will improve overall performance and reduce latency.
         *  - Choose your video source.  This may be
         *      .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))  .....   for a webcam
         *  or
         *      .setCamera(BuiltinCameraDirection.BACK)    ... for a Phone Camera
         */
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(960, 600));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the ColorBlobLocator processors.
        builder.addProcessor(redColorLocator);
        builder.addProcessor(blueColorLocator);
        builder.addProcessor(yellowColorLocator);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the ColorBlobLocator processors at any time.
        //visionPortal.setProcessorEnabled(redColorLocator, false);
        //visionPortal.setProcessorEnabled(blueColorLocator, false);
        //visionPortal.setProcessorEnabled(yellowColorLocator, false);
    }   // end method initColorLocators()

}
