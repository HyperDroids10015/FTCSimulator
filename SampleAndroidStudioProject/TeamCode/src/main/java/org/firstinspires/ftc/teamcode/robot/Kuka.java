package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Kuka {
  /* Declare OpMode members. */
  private LinearOpMode opmode;   // gain access to methods in the calling OpMode.

  // Define robot hardware objects  (Make them private so they can't be accessed externally)
  private Base base;
  private Arm arm;
  private Gripper gripper;
  //private WebcamName color;
  //private DistanceSensor range;

  // Define a constructor that allows the OpMode to pass a reference to itself.
  public Kuka(LinearOpMode opmode) {
    this.opmode = opmode;
  }

  /**
   * Initialize all the robot's hardware.
   * This method must be called ONCE when the OpMode is initialized.
   * <p>
   * All of the hardware devices are accessed via the hardware map, and initialized.
   */
  public void init() {
    // Initialize robot hardware objects (note: need to use reference to actual OpMode).
    base = new Base(opmode);
    arm = new Arm(opmode);
    gripper = new Gripper(opmode);
    //color = opmode.hardwareMap.get(WebcamName.class, "kinect color");
    //range = opmode.hardwareMap.get(DistanceSensor.class, "kinect range");
    opmode.telemetry.addData(">", "Hardware Initialized");
    opmode.telemetry.update();
  }

  public void setBasePosition(SparkFunOTOS.Pose2D pos) {
    base.setPosition(pos);
  }

  public void stopRobot() {
    base.stopMotors();
    arm.resetPosition();
  }

  public void driveRobot(double drive, double strafe, double turn) {
    base.driveRobot(drive, strafe, turn);
  }

  public double[] getArmPosition() {
    return arm.getPosition();
  }
  public void increaseArmHeight() {
    arm.increaseHeight();
  }

  public void decreaseArmHeight() {
    arm.decreaseHeight();
  }

  public void increaseArmOrientation() {
    arm.increaseOrientation();
  }

  public void decreaseArmOrientation() {
    arm.decreaseOrientation();
  }

  public void closeGripper() {
    gripper.grip();
  }

  public void openGripper() {
    gripper.release();
  }

  public SparkFunOTOS.Pose2D getPosition() {
    return base.getPosition();
  }

  public void setTarget(SparkFunOTOS.Pose2D target) {
    base.setTarget(target);
  }

  public void runToTarget() {
    base.runToTarget();
  }

  public boolean targetReached() {
    return base.targetReached();
  }
}
