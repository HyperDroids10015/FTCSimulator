package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Gripper {
  /* Declare OpMode members. */
  private final LinearOpMode opmode; // gain access to methods in the calling OpMode.

  protected DcMotor[] fingers;
  // to do: convert millimeters to ticks:
  private final static double TICKS_PER_RADIAN = 537.69 / (2 * Math.PI); // for 312 rpm gobilda motor
  private final int MIN_POS = 0;
  private final int MAX_POS = (int)(0.025*TICKS_PER_RADIAN);

  public Gripper(LinearOpMode opmode) {
    this.opmode = opmode;
    fingers = new DcMotor[]{
            opmode.hardwareMap.get(DcMotor.class, "finger::left"),
            opmode.hardwareMap.get(DcMotor.class, "finger::right")
    };
    for (DcMotor finger : fingers) {
      finger.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      finger.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      finger.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      //finger.setPower(1);
    }
    for (DcMotor finger : fingers) ((DcMotorEx)finger).setVelocity(1); // need DcMotorEx class
//    for (DcMotor finger : fingers) ((DcMotorEx)finger).setVelocity(0.03); // need DcMotorEx class
  }

  public void grip() {
    fingers[0].setTargetPosition(MIN_POS);
  }
  
  public void release() {
    fingers[0].setTargetPosition(MAX_POS);
  }
  
  public void setGap(int gap) {
      // to do: convert millimeters to ticks:
      int OFFSET_WHEN_LOCKED = 21;
      int v = (gap - OFFSET_WHEN_LOCKED) / 2;
    if (v > MAX_POS) v = MAX_POS;
    else if (v < MIN_POS) v = MIN_POS;
    fingers[0].setTargetPosition(v);
  }
}