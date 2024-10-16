// File:          CameraController.java
// Date:
// Description:
// Author:
// Modifications:

// You may need to add other webots classes such as
//  import com.cyberbotics.webots.controller.DistanceSensor;
//  import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Camera;

// Here is the main class of your controller.
// This class defines how to initialize and how to run your controller.
public class CameraController {

  // This is the main function of your controller.
  // It creates an instance of your Robot instance and
  // it uses its function(s).
  // Note that only one instance of Robot should be created in
  // a controller program.
  // The arguments of the main function can be specified by the
  // "controllerArgs" field of the Robot node
  public static void main(String[] args) {

    // create the Robot instance.
    Robot robot = new Robot();

    // get the time step of the current world.
    int timeStep = (int) Math.round(robot.getBasicTimeStep());

    // You should insert a getDevice-like function in order to get the
    // instance of a device of the robot. Something like:
    //  Motor motor = robot.getMotor("motorname");
    //  DistanceSensor ds = robot.getDistanceSensor("dsname");
    //  ds.enable(timeStep);
    Camera camera = robot.getCamera("Webcam 1");
    camera.enable(timeStep);
    System.out.printf("Sampling period: %d\n",camera.getSamplingPeriod());
    System.out.printf("Field of view: %.2f (%.2f..%.2f)\n",camera.getFov(),camera.getMinFov(),camera.getMaxFov());
    System.out.printf("Exposure: %f\n",camera.getExposure());
    System.out.printf("Focal length: %f\n",camera.getFocalLength());
    System.out.printf("Focal distance: %.2f (%.2f..%.2f)\n",camera.getFocalDistance(),camera.getMaxFocalDistance(),camera.getMinFocalDistance());
    int imageWidth = camera.getWidth();
    int imageHeight = camera.getHeight();
    System.out.printf("Image size: %d x %d\n",imageWidth,imageHeight);
    System.out.printf("Near: %f\n",camera.getNear());
    
/*
    for (int i=0; i < image.length; i++) {
      int pixel = image[i];
      int r = Camera.pixelGetRed(pixel);
      int g = Camera.pixelGetGreen(pixel);
      int b = Camera.pixelGetBlue(pixel);
      System.out.println("red=" + r + " green=" + g + " blue=" + b);
    }
*/
    // Main loop:
    // - perform simulation steps until Webots is stopping the controller
    while (robot.step(timeStep) != -1) {
      // Read the sensors:
      // Enter here functions to read sensor data, like:
      //  double val = ds.getValue();
      int[] image = camera.getImage();
      System.out.printf("Image size: %d\n",image.length);
      
//      int res = camera.saveImage("image1.png",100);
    
//      if (res<0) System.err.println("unable to save image");

      // Process sensor data here.

      // Enter here functions to send actuator commands, like:
      //  motor.setPosition(10.0);
    };

    // Enter here exit cleanup code.
  }
}
