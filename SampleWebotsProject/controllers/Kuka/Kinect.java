import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.RangeFinder;

public class Kinect {
  protected Camera color;
  protected RangeFinder range;

  public Kinect(Robot robot, int timeStep) {
    color = robot.getCamera("kinect color");
    color.enable(timeStep);
    range = robot.getRangeFinder("kinect range");
    range.enable(timeStep);
  }


}