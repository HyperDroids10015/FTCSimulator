// File:          mygps.java
// Date: August 1, 2024
// Description: An example of a controller using gps device
// Author: Iskander Karibzhanov
// Modifications:

// You may need to add other webots classes such as
//  import com.cyberbotics.webots.controller.DistanceSensor;
//  import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.GPS;

public class mygps {

  public static void main(String[] args) {

    Robot robot = new Robot();

    int timeStep = (int) Math.round(robot.getBasicTimeStep());

    // You should insert a getDevice-like function in order to get the
    // instance of a device of the robot. Something like:
    //  Motor motor = robot.getMotor("motorname");
    //  DistanceSensor ds = robot.getDistanceSensor("dsname");
    //  ds.enable(timeStep);
    GPS gps = robot.getGPS("gps");
    gps.enable(timeStep);
    
    while (robot.step(timeStep) != -1) {
      // Read the sensors:
      // Enter here functions to read sensor data, like:
      //  double val = ds.getValue();
      double[] gps_values = gps.getValues();
      double[] speed_vector_values = gps.getSpeedVector();
      
      // Process sensor data here.
      System.out.printf("GPS position: %.3f %.3f %.3f\n", gps_values[0], gps_values[1], gps_values[2]);
      System.out.printf("GPS speed vector: %.3f %.3f %.3f\n", speed_vector_values[0], speed_vector_values[1], speed_vector_values[2]);
      
      // Enter here functions to send actuator commands, like:
      //  motor.setPosition(10.0);
    };

    // Enter here exit cleanup code.
  }
}
