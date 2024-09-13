package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.ftc6448.simulator.PlatformSupport;

public class Base {
  /* Declare OpMode members. */
  private LinearOpMode opmode;   // gain access to methods in the calling OpMode.

  private final DcMotor[] wheels;
  private SparkFunOTOS gps;
  private SparkFunOTOS.Pose2D target;
  private boolean targetReached;

  private static final DistanceUnit DISTANCE_UNIT = DistanceUnit.METER;
  private static final AngleUnit ANGLE_UNIT = AngleUnit.DEGREES;

  private static final double DISTANCE_TOLERANCE = 0.01; // in DISTANCE_UNITs to target
  private static final double ANGLE_TOLERANCE = 0.5; // in ANGLE_UNITs to target

  // robot geometry
//  private static final double WHEEL_RADIUS = 0.05; // in DISTANCE_UNITs
//  private static final double LX = 0.228;  // longitudinal distance from robot's COM to wheel [m].
//  private static final double LY = 0.158;  // lateral distance from robot's COM to wheel [m].
//  public static final double MAX_SPEED = 0.3; // maximum chassis speed in DISTANCE_UNITs per second

  // stimulus coefficients (must sum up to one)
  private static final double K_DRIVE = 32.0;
  private static final double K_STRAFE = 32.0;
  private static final double K_TURN = 32.0;

  public Base(LinearOpMode opmode) {
    this.opmode = opmode;
    wheels = new DcMotor[]{
            opmode.hardwareMap.get(DcMotor.class, "frontLeftMotor"),
            opmode.hardwareMap.get(DcMotor.class, "backLeftMotor"),
            opmode.hardwareMap.get(DcMotor.class, "frontRightMotor"),
            opmode.hardwareMap.get(DcMotor.class, "backRightMotor")
    };
    //use PlatformSupport method to see if you are running on the simulator
    if (!PlatformSupport.isSimulator()) {
      // On most drivetrains, you will need to reverse the left side for positive power to move forwards with most motors
      wheels[2].setDirection(DcMotor.Direction.REVERSE);
      wheels[3].setDirection(DcMotor.Direction.REVERSE);
    }
    for (DcMotor wheel : wheels) wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    stopMotors();
    configureGPS();
  }

  public void configureGPS() {
    gps = opmode.hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
    gps.setLinearUnit(DISTANCE_UNIT);
    gps.setAngularUnit(ANGLE_UNIT);
    SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
    gps.setOffset(offset);
    gps.setLinearScalar(1.0);
    gps.setAngularScalar(1.0);
    gps.calibrateImu();
    gps.resetTracking();
    targetReached = true;
  }

  public void setPosition(SparkFunOTOS.Pose2D pos) {
    gps.setPosition(pos);
  }

  public void stopMotors() {
    double[] power = {0.0, 0.0, 0.0, 0.0};
    setDrivePower(power);
  }

  /**
   * Pass the requested wheel motor powers to the appropriate hardware drive motors.
   *
   * @param power      Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
   */
  public void setDrivePower(double[] power) {
    for (int i = 0; i < wheels.length; i++)
      wheels[i].setPower(power[i]);
  }

  /**
   * Calculates the motor powers required to achieve the requested
   * robot motions: Drive (Axial motion), Strafe (Lateral motion) and Turn (Yaw motion).
   * Then sends these power levels to the motors.
   *
   * @param drive     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
   * @param strafe    Right/Left strafing power (-1.0 to 1.0) +ve is right
   * @param turn      Right/Left turning power (-1.0 to 1.0) +ve is CW
   */
  public void driveRobot(double drive, double strafe, double turn) {
    // Combine velocities for each axis-motion to determine each wheel's power.
    // Normalize the speeds so no wheel power exceeds 100%
    // This ensures that the robot maintains the desired motion.
    // Denominator is the largest motor power (absolute value) or 1
    // This ensures all the powers maintain the same ratio,
    // but only if at least one is out of the range [-1, 1]
    double max = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1);
    double[] power = new double[]{
              (drive + strafe + turn) / max,
              (drive - strafe + turn) / max,
              (drive - strafe - turn) / max,
              (drive + strafe - turn) / max
      };
    // Use existing function to drive all four wheels using calculated powers.
    setDrivePower(power);
  }

  public SparkFunOTOS.Pose2D getPosition() {
    return gps.getPosition();
  }

  public void setTarget(SparkFunOTOS.Pose2D target) {
    this.target = target;
    targetReached = false;
  }

  public void runToTarget() {
    // Get the latest position, which includes the x and y coordinates, plus the heading angle
    SparkFunOTOS.Pose2D pos = gps.getPosition();
    opmode.telemetry.addData("Robot Position", "x = %4.2f, y = %4.2f, h = %4.2f", pos.x, pos.y, pos.h);

    // compute distance and check if the target is reached
    double dx = target.x - pos.x;
    double dy = target.y - pos.y;
    double dh = target.h - pos.h;
    double distance = Math.sqrt(dx * dx + dy * dy);
    opmode.telemetry.addData("Distance to target", distance);
    opmode.telemetry.addData("Delta Heading to target", dh);
    opmode.telemetry.update();

    // compute the direction vector relatively to the robot coordinates
    double cosh = Math.cos(ANGLE_UNIT.toRadians(pos.h));
    double sinh = Math.sin(ANGLE_UNIT.toRadians(pos.h));

    // compute the speeds stimuli as ratios of MAX_SPEED
    // -> first stimulus: x coord of the relative target vector
    double drive = (-sinh * dx + cosh * dy) * K_DRIVE;
    // -> second stimulus: y coord of the relative target vector
    double strafe = (cosh * dx + sinh * dy) * K_STRAFE;
    // -> third stimulus: delta angle with the target heading angle
    double turn = -ANGLE_UNIT.toRadians(dh)/Math.PI * K_TURN;

    driveRobot(drive, strafe, turn);

    if (distance < DISTANCE_TOLERANCE && Math.abs(dh) < ANGLE_TOLERANCE) {
      stopMotors();
      targetReached = true;
    }
  }

  public boolean targetReached() {
    return targetReached;
  }
}