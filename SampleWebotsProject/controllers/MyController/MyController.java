// File:          MyController.java
// Date:
// Description:
// Author:
// Modifications:

// You may need to add other webots classes such as
//  import com.cyberbotics.webots.controller.DistanceSensor;
//  import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.InertialUnit;

// Here is the main class of your controller.
// This class defines how to initialize and how to run your controller.
public class MyController {

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
    InertialUnit imu = robot.getInertialUnit("imu");
    imu.enable(timeStep);

    double[] angles = imu.getRollPitchYaw();
    System.out.printf("roll = %f, pitch = %f, yaw = %f\n",angles[0],angles[1],angles[2]);
    double[] values = imu.getQuaternion();
    System.out.printf("x = %f, y = %f, z = %f, w = %f\n",values[0],values[1],values[2],values[3]);

    // Main loop:
    // - perform simulation steps until Webots is stopping the controller
    while (robot.step(timeStep) != -1) {
      // Read the sensors:
      // Enter here functions to read sensor data, like:
      //  double val = ds.getValue();
      angles = imu.getRollPitchYaw();
      System.out.printf("roll = %f, pitch = %f, yaw = %f\n",angles[0],angles[1],angles[2]);
      values = imu.getQuaternion();
      System.out.printf("x = %f, y = %f, z = %f, w = %f\n",values[0],values[1],values[2],values[3]);

      // Process sensor data here.

      // Enter here functions to send actuator commands, like:
      //  motor.setPosition(10.0);
    };

    // Enter here exit cleanup code.
  }
}
