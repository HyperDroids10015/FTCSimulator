import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.GPS;
import com.cyberbotics.webots.controller.Compass;

public class Base {
  private static final double SPEED = 4.0;
  public static final double MAX_SPEED = 0.3;
  private static final double SPEED_INCREMENT = 0.05;
  private static final double DISTANCE_TOLERANCE = 0.001;
  private static final double ANGLE_TOLERANCE = 0.001;
  
  // robot geometry
  private static final double WHEEL_RADIUS = 0.05;
  private static final double LX = 0.228;  // longitudinal distance from robot's COM to wheel [m].
  private static final double LY = 0.158;  // lateral distance from robot's COM to wheel [m].

  // stimulus coefficients
  private static final double K1 = 3.0;
  private static final double K2 = 1.0;
  private static final double K3 = 1.0;

  protected Motor[] wheels;
  private GPS gps;
  private Compass compass;

  private class GoTo {
    protected Vector2 vTarget;
    protected double alpha = 0.0;
    protected boolean reached = false;
    public GoTo(double x, double y, double alpha, boolean reached) {
      vTarget = new Vector2(x, y);
      this.alpha = alpha;
      this.reached = reached;
    }
  }

  private GoTo goTo;
  private double vx;
  private double vy;
  private double omega;
  
  public Base(Robot robot, int timeStep) {
    wheels = new Motor[] {robot.getMotor("wheel1"),robot.getMotor("wheel2"),robot.getMotor("wheel3"),robot.getMotor("wheel4")};
    reset();
    gps = robot.getGPS("gps");
    gps.enable(timeStep);
    compass = robot.getCompass("compass");
    compass.enable(timeStep);
    goTo = new GoTo(0.0, 0.0, 0.0, false);
  }
  
  public void reset() {
    double[] speeds = {0.0, 0.0, 0.0, 0.0};
    setWheelSpeedsHelper(speeds);
    vx = 0.0;
    vy = 0.0;
    omega = 0.0;
  }
  
  private void setWheelVelocity(Motor motor, double velocity) {
    motor.setPosition(Double.POSITIVE_INFINITY);
    motor.setVelocity(velocity);
  }
  
  private void setWheelSpeedsHelper(double[] speeds) {
    for (int i = 0; i < wheels.length; i++)
      setWheelVelocity(wheels[i], speeds[i]);
  }
  
  public void move(double vx, double vy, double omega) {
    double[] speeds = new double[4];
    speeds[0] = 1 / WHEEL_RADIUS * (vx + vy + (LX + LY) * omega);
    speeds[1] = 1 / WHEEL_RADIUS * (vx - vy - (LX + LY) * omega);
    speeds[2] = 1 / WHEEL_RADIUS * (vx - vy + (LX + LY) * omega);
    speeds[3] = 1 / WHEEL_RADIUS * (vx + vy - (LX + LY) * omega);
    setWheelSpeedsHelper(speeds);
//    System.out.printf("Wheel velocities (rad/s): %.2f %.2f %.2f %.2f\n", wheels[0].getVelocity(), wheels[1].getVelocity(), wheels[2].getVelocity(), wheels[3].getVelocity());
//    System.out.printf("Speeds: vx=%.2f[m/s] vy=%.2f[m/s] ω=%.2f[rad/s]\n", vx, vy, omega);
  }

  public void forwardsIncrement() {
    vx += SPEED_INCREMENT;
    if (vx > MAX_SPEED) vx = MAX_SPEED;
    move(vx, vy, omega);
  }

  public void backwardsIncrement() {
    vx -= SPEED_INCREMENT;
    if (vx < -MAX_SPEED) vx = -MAX_SPEED;
    move(vx, vy, omega);
  }

  public void strafeLeftIncrement() {
    vy += SPEED_INCREMENT;
    if (vy > MAX_SPEED) vy = MAX_SPEED;
    move(vx, vy, omega);
  }

  public void strafeRightIncrement() {
    vy -= SPEED_INCREMENT;
    if (vy < -MAX_SPEED) vy = -MAX_SPEED;
    move(vx, vy, omega);
  }

  public void turnLeftIncrement() {
    omega += SPEED_INCREMENT;
    if (omega > MAX_SPEED) omega = MAX_SPEED;
    move(vx, vy, omega);
  }

  public void turnRightIncrement() {
    omega -= SPEED_INCREMENT;
    if (omega < -MAX_SPEED) omega = -MAX_SPEED;
    move(vx, vy, omega);
  }
  
  public void setTarget(double x, double y, double alpha) {
    goTo.vTarget.setValues(x, y);
    goTo.alpha = alpha;
    goTo.reached = false;
  }
  
  public boolean reached() {
    return goTo.reached;
  }

  void run() {
    // get sensors
    double[] gpsRawValues = gps.getValues();
    double[] compassRawValues = compass.getValues();

    // compute 2d vectors
    Vector2 vGPS = new Vector2(gpsRawValues[0], gpsRawValues[1]);
    Vector2 vFront = new Vector2(compassRawValues[0], compassRawValues[1]);
    Vector2 vRight = new Vector2(-vFront.y, vFront.x);
    Vector2 vNorth = new Vector2(1.0, 0.0);

    // compute distance
    Vector2 vDir = goTo.vTarget.minus(vGPS);
    double distance = vDir.norm();

    // compute absolute angle & delta with the target angle
    double theta = vFront.angle(vNorth);
    double deltaAngle = theta - goTo.alpha;

    // compute the direction vector relatively to the robot coordinates
    // using a matrix of homogeneous coordinates
    Vector3 a = new Vector3(-vRight.x, vFront.x, 0.0);
    Vector3 b = new Vector3(vRight.y, -vFront.y, 0.0);
    Vector3 c = new Vector3(vRight.x * vGPS.x - vRight.y * vGPS.y, -vFront.x * vGPS.x + vFront.y * vGPS.y, 1.0);
    Matrix33 transform = new Matrix33(a, b, c);
    Vector3 vTargetTmp = new Vector3(goTo.vTarget.x, goTo.vTarget.y, 1.0);
    Vector3 vTargetRel = transform.multVector3(vTargetTmp);

    // compute the speeds
    double[] speeds = {0.0, 0.0, 0.0, 0.0};
    // -> first stimulus: delta_angle
    speeds[0] = -deltaAngle / Math.PI * K1;
    speeds[1] =  deltaAngle / Math.PI * K1;
    speeds[2] = -deltaAngle / Math.PI * K1;
    speeds[3] =  deltaAngle / Math.PI * K1;
    // -> second stimulus: x coord of the relative target vector
    speeds[0] += vTargetRel.x * K2;
    speeds[1] += vTargetRel.x * K2;
    speeds[2] += vTargetRel.x * K2;
    speeds[3] += vTargetRel.x * K2;
    // -> third stimulus: y coord of the relative target vector
    speeds[0] += -vTargetRel.y * K3;
    speeds[1] +=  vTargetRel.y * K3;
    speeds[2] +=  vTargetRel.y * K3;
    speeds[3] += -vTargetRel.y * K3;

    // apply the speeds
    for (int i = 0; i < 4; i++) {
      speeds[i] /= (K1 + K2 + K2);  // number of stimuli (-1 <= speeds <= 1)
      speeds[i] *= SPEED;           // map to speed (-SPEED <= speeds <= SPEED)
      // added an arbitrary factor increasing the convergence speed
      speeds[i] *= 5.0;
      if (speeds[i] > SPEED) speeds[i] = SPEED;
      else if (speeds[i] < -SPEED) speeds[i] = -SPEED;
    }
    setWheelSpeedsHelper(speeds);
    
    // check if the target is reached
    if (distance < DISTANCE_TOLERANCE && Math.abs(deltaAngle) < ANGLE_TOLERANCE)
      goTo.reached = true;
  }
  
}