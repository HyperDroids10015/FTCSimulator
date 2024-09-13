package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Arm {
  /* Declare OpMode members. */
  private final LinearOpMode opmode;   // gain access to methods in the calling OpMode.

  protected DcMotor[] armElements;
  private Height currentHeight;
  private Orientation currentOrientation;
  private final static double TICKS_PER_RADIAN = 537.69 / (2 * Math.PI); // for 312 rpm gobilda motor

  private enum Orientation {
    ARM_BACK_RIGHT(2.949),
    ARM_RIGHT(Math.PI/2),
    ARM_FRONT_RIGHT(0.2),
    ARM_FRONT(0.0),
    ARM_FRONT_LEFT(-0.2),
    ARM_LEFT(-Math.PI/2),
    ARM_BACK_LEFT(-2.949);

    private final double arm0;

    Orientation(double arm0) {
      this.arm0 = arm0;
    }
    public Orientation next() {
      Orientation[] orientations = Orientation.values();
      // Prevents from going beyond index.
      return orientations[Math.min(this.ordinal() + 1, orientations.length - 1)];
    }

    public Orientation previous() {
      Orientation[] orientations = Orientation.values();
      return orientations[Math.max(this.ordinal() - 1, 0)];
    }
  }

  private enum Height {
    ARM_BACK_PLATE_LOW(0.92,0.42,1.78,0.0),
    ARM_BACK_PLATE_HIGH(0.678,0.682,1.74,0.0),
    ARM_RESET(1.57,-2.635,1.78,0.0),
    ARM_FRONT_CARDBOARD_BOX(0.0,-0.77,-1.21,0.0),
    ARM_HANOI_PREPARE(-0.4,-1.2,-Math.PI/2,Math.PI/2),
    ARM_FRONT_PLATE(-0.62,-0.98,-1.53,0.0),
    ARM_FRONT_FLOOR(-0.97,-1.55,-0.61,0.0);

    private final double arm1;
    private final double arm2;
    private final double arm3;
    private final double arm4;

    Height(double arm1, double arm2, double arm3, double arm4) {
      this.arm1 = arm1;
      this.arm2 = arm2;
      this.arm3 = arm3;
      this.arm4 = arm4;
    }

    public Height next() {
      Height[] heights = Height.values();
      // Prevents from going beyond index.
      return heights[Math.min(this.ordinal() + 1, heights.length - 1)];
    }

    public Height previous() {
        Height[] heights = Height.values();
        return heights[Math.max(this.ordinal() - 1, 0)];
    }
  }

  public Arm(LinearOpMode opmode) {
    this.opmode = opmode;
    armElements = new DcMotor[] {
            opmode.hardwareMap.get(DcMotor.class, "arm1"),
            opmode.hardwareMap.get(DcMotor.class, "arm2"),
            opmode.hardwareMap.get(DcMotor.class, "arm3"),
            opmode.hardwareMap.get(DcMotor.class, "arm4"),
            opmode.hardwareMap.get(DcMotor.class, "arm5")
    };
    for (DcMotor armElement : armElements) {
        armElement.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armElement.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armElement.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //armElement.setPower(1);
    }
    resetPosition();
  }

  public double[] getPosition() {
    return new double[]{
      armElements[0].getCurrentPosition()/TICKS_PER_RADIAN,
      armElements[1].getCurrentPosition()/TICKS_PER_RADIAN,
      armElements[2].getCurrentPosition()/TICKS_PER_RADIAN,
      armElements[3].getCurrentPosition()/TICKS_PER_RADIAN,
      armElements[4].getCurrentPosition()/TICKS_PER_RADIAN
    };
  }

  private void setMotorPosition(DcMotor motor, double position) {
    motor.setTargetPosition((int) (position*TICKS_PER_RADIAN));
  }

  private void setPosition(Orientation orientation, Height height) {
    double[] positions = new double[] {orientation.arm0, height.arm1, height.arm2, height.arm3, height.arm4};
//    System.out.print("arm:\t");
    for (int i = 0; i < armElements.length; i++) {
//      System.out.printf("%f\t",positions[i]);
      setMotorPosition(armElements[i], positions[i]);
    }
//    System.out.println();

    currentOrientation = orientation;
    currentHeight = height;
  }

  public void resetPosition() {
    setPosition(Orientation.ARM_FRONT, Height.ARM_RESET);
  }

  public void increaseHeight() {
    Height newHeight = currentHeight.next();
    // Prevents self-colliding poses.
    if (newHeight == Height.ARM_FRONT_FLOOR && (currentOrientation == Orientation.ARM_BACK_LEFT || currentOrientation == Orientation.ARM_BACK_RIGHT))
      newHeight = currentHeight;
    setPosition(currentOrientation, newHeight);
  }

  public void decreaseHeight() {
    Height newHeight = currentHeight.previous();
    setPosition(currentOrientation, newHeight);
  }

  public void increaseOrientation() {
    Orientation newOrientation = currentOrientation.next();
    // Prevents self-colliding poses.
    if (newOrientation == Orientation.ARM_BACK_LEFT && currentHeight == Height.ARM_FRONT_FLOOR)
      newOrientation = currentOrientation;
    setPosition(newOrientation, currentHeight);
  }

  public void decreaseOrientation() {
    Orientation newOrientation = currentOrientation.previous();
    // Prevents self-colliding poses.
    if (newOrientation == Orientation.ARM_BACK_RIGHT && currentHeight == Height.ARM_FRONT_FLOOR)
      newOrientation = currentOrientation;
    setPosition(newOrientation, currentHeight);
  }
}