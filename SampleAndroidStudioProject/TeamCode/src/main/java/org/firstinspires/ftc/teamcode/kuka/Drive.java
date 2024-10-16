package org.firstinspires.ftc.teamcode.kuka;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Drive {
  private final MecanumDrive drivebase;
  private SparkFunOTOS gps;

  // inputs into the drivebase, from human or auto
  private double forward; // Fwd/Rev driving power (Axial motion) (-1.0 to 1.0) +ve is forward
  private double strafe; // Right/Left strafing power (Lateral motion) (-1.0 to 1.0) +ve is right
  private double turn; // Right/Left turning power (Yaw motion) (-1.0 to 1.0) +ve is CW
  private SparkFunOTOS.Pose2D pose;
  private double heading;

  private static final double DISTANCE_TOLERANCE = 0.01; // in DISTANCE_UNITs to target
  private static final double ANGLE_TOLERANCE = 0.5; // in ANGLE_UNITs to target

  // robot geometry
//  private static final double WHEEL_RADIUS = 0.05; // in DISTANCE_UNITs
//  private static final double LX = 0.228;  // longitudinal distance from robot's COM to wheel [m].
//  private static final double LY = 0.158;  // lateral distance from robot's COM to wheel [m].
//  public static final double MAX_SPEED = 0.3; // maximum chassis speed in DISTANCE_UNITs per second

  // stimulus coefficients (must sum up to one)
  private static final double K_DRIVE = 8.0;
  private static final double K_STRAFE = 8.0;
  private static final double K_TURN = 8.0;

//  static final double gearRatio = (1+46.0/17) * (1+46.0/11) * (22.0/24);
//  static final double CPR = 28*gearRatio;
//  static final double RPM = 6000/gearRatio;

  public Drive(HardwareMap hwMap) {
    drivebase = new MecanumDrive(false,
            new Motor(hwMap, "frontLeftMotor", Motor.GoBILDA.RPM_312),//, CPR, RPM
            new Motor(hwMap, "frontRightMotor", Motor.GoBILDA.RPM_312),//, CPR, RPM
            new Motor(hwMap, "backLeftMotor", Motor.GoBILDA.RPM_312),//, CPR, RPM
            new Motor(hwMap, "backRightMotor", Motor.GoBILDA.RPM_312));//, CPR, RPM

    // configure odometry sensor
    gps = hwMap.get(SparkFunOTOS.class, "sensor_otos");
    gps.setLinearUnit(DistanceUnit.METER);
    gps.setAngularUnit(AngleUnit.DEGREES);
    gps.setOffset(new SparkFunOTOS.Pose2D(0, 0, 0));
    gps.setLinearScalar(1.0);
    gps.setAngularScalar(1.0);
    gps.calibrateImu();
    gps.resetTracking();
  }

  public void stop() {
    drivebase.stop();
  }

  public void setPosition(SparkFunOTOS.Pose2D pose) {
    gps.setPosition(pose);
    this.pose = pose;
  }

  public SparkFunOTOS.Pose2D getPosition() {
    return pose;
  }

  public void humanInputs(GamepadEx driver) {
    // Get the latest pose, which includes the x and y coordinates, plus the heading angle
    pose = gps.getPosition();
    heading = pose.h;

    // Run wheels in POV mode: use the Right stick to go forward & strafe, the Left stick to rotate left & right.
    forward = -driver.getRightY();
    strafe = driver.getRightX();
    turn = driver.getLeftX();
  }

  public void loop() {
    drivebase.driveFieldCentric(strafe, forward, turn, heading, false);
  }

  public boolean targetReached(SparkFunOTOS.Pose2D target) {
    // Get the latest pose, which includes the x and y coordinates, plus the heading angle
    pose = gps.getPosition();
    heading = pose.h;

    // compute distance and check if the target is reached
    double dx = target.x - pose.x;
    double dy = target.y - pose.y;
    double dh = target.h - pose.h;
    if (dh > 180) dh -= 360;
    if (dh < -180) dh += 360;

    // compute the direction vector relatively to the robot coordinates

    // compute the speeds stimuli as ratios of MAX_SPEED
    // -> first stimulus: x coord of the relative target vector
    forward = dy * K_DRIVE;
    // -> second stimulus: y coord of the relative target vector
    strafe = dx * K_STRAFE;
    // -> third stimulus: delta angle with the target heading angle
    turn = -dh/180 * K_TURN;

    return Math.hypot(dx, dy) < DISTANCE_TOLERANCE && Math.abs(dh) < ANGLE_TOLERANCE;
  }
}