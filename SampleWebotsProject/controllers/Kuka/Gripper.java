import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Motor;

public class Gripper {
  protected Motor fingers;
  private final double MIN_POS = 0.0;
  private final double MAX_POS = 0.025;
  private final double OFFSET_WHEN_LOCKED = 0.021;
  
  public Gripper(Robot robot) {
    fingers = robot.getMotor("finger::left");
    fingers.setVelocity(0.03);
  }
  
  public void grip() {
    fingers.setPosition(MIN_POS);
  }
  
  public void release() {
    fingers.setPosition(MAX_POS);
  }
  
  public void setGap(double gap) {
    double v = 0.5 * (gap - OFFSET_WHEN_LOCKED);
    if (v > MAX_POS) v = MAX_POS;
    else if (v < MIN_POS) v = MIN_POS;
    fingers.setPosition(v);
  }
}