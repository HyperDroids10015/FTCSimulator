package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Drive {
    // XXX FIXME: one copy of these (also in Auto)
    private static final AngleUnit ANGLE_UNIT = AngleUnit.DEGREES;
    private static final DistanceUnit DISTANCE_UNIT = DistanceUnit.METER;

    // medium-term I'd like to centralize telemetry handling in loop()
    // or another method (so we can do both "normal" telemetry, and
    // FTCDashboard stuff there, and turn it off easily if we want
    // too)
    Telemetry telemetry;

    // Create an instance of the sensor
    SparkFunOTOS otos;
    MecanumDrive drivebase = null;

    // inputs into the drivebase, from human or auto
    double forward = 0.0;
    double strafe = 0.0;
    double turn = 0.0;
//    double desired_heading = 0.0;
//    PIDController heading_control;

    // cached once per loop (do not set, read-only)
    public SparkFunOTOS.Pose2D current_position;

    // only pass in this _required_ by this class; if we give access
    // to the parent OpMode here, we have two-way dependencies and
    // lose track of what we "actually" need in these subsystem
    // classes

    public Drive(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        Motor motor_fl = new Motor(hardwareMap, "fl", Motor.GoBILDA.RPM_312);
        motor_fl.setInverted(true);
        Motor motor_fr = new Motor(hardwareMap, "fr", Motor.GoBILDA.RPM_312);
        Motor motor_bl = new Motor(hardwareMap, "bl", Motor.GoBILDA.RPM_312);
        motor_bl.setInverted(true);
        Motor motor_br = new Motor(hardwareMap, "br", Motor.GoBILDA.RPM_312);
        drivebase = new MecanumDrive(false, motor_fl,motor_fr,motor_bl,motor_br);
//        heading_control = new PIDController(0.033,0,0.0007);

        if (true) {
            motor_fl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
            motor_fr.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
            motor_bl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
            motor_br.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        } else {
            motor_fl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            motor_fr.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            motor_bl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            motor_br.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        }

        otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        configureOtos();
    }
    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        otos.setLinearUnit(DISTANCE_UNIT);
        otos.setAngularUnit(ANGLE_UNIT);

        // offset of the OTOS mount from center is 41.6mm according to
        // the CAD
        otos.setOffset(
            new SparkFunOTOS.Pose2D(0, 0.0416, 0)
        );

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971

        //otos.setAngularScalar(1.0);
        // average of 17.79 degrees off on 10 rotations, means: 0.995
        otos.setAngularScalar(0.995);

        // average of 0.9819 on manual push of 1m
        otos.setLinearScalar(1.018);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        otos.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        otos.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        otos.getVersionInfo(hwVersion, fwVersion);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }

    public void robotInputs(double strafe, double forward, double turn) {
        this.forward = forward;
        this.strafe = strafe;
        this.turn  = turn;
    }

    public void setPosition(SparkFunOTOS.Pose2D pose) {
        otos.setPosition(pose);
        current_position = pose;
    }

    // all interaction with gamepads should go through this method
    public void humanInputs(GamepadEx driver){
        // this method called ONCE per loop from teleop controller
        forward = -driver.getRightY();
        strafe = driver.getRightX();
        turn = driver.getLeftX();
/*
        desired_heading = desired_heading - 3*driver.getLeftX();
        if (desired_heading > 180) desired_heading -= 360;
        if (desired_heading < -180) desired_heading += 360;
        double h = desired_heading-current_position.h;
        if (h > 180) h -= 360;
        if (h < -180) h += 360;
        turn = heading_control.calculate(h);
*/
        // Reset the tracking if the user requests it
        if (driver.wasJustPressed(GamepadKeys.Button.Y)) {
            otos.resetTracking();
        }

        // Re-calibrate the IMU if the user requests it
        if (driver.wasJustPressed(GamepadKeys.Button.X)) {
            otos.calibrateImu();
        }
    }

    // called ONCE, before any driver or robot inputs
    public void read_sensors() {
        // Get the latest position, which includes the x and y coordinates, plus the
        // heading angle
        current_position = otos.getPosition();
    }

    public void loop() {
        // Log the position to the telemetry
        telemetry.addData("X coordinate cm", (current_position.x * 100.0));
        telemetry.addData("Y coordinate cm", (current_position.y * 100.0));
        telemetry.addData("Heading angle", current_position.h);
//        telemetry.addData("Desired heading", desired_heading);
//        telemetry.addData("turn", turn );

        // Update the telemetry on the driver station
        telemetry.update();

        // tell ftclib its inputs
        drivebase.driveFieldCentric(strafe, forward, turn, current_position.h, true);
    }

}
