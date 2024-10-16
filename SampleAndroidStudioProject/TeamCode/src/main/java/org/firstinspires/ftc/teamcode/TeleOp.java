package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

// put all FTCLib imports here
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.List;


// october 13, doing some OTOS calibration tests
//
// spin 10x on Adrian's "at home" tiles (not identical to production)
// -> start square to something
// -> use controller to spin 10x
// -> manually square up at the end
//
// test #1: end angle: 17.699  (spin ccw)
// test #2: end angle: -18.62  (spin cw)
// test #3: end angle: -16.9299 (spin cw)
// test #4: end angle: 17.869  (spin ccw)

//// (17.869 + 16.9299 + 18.62 + 17.699) / 4.0 = 17.779


// linear tests (1m push in one direction)
// test #1: manual push backwards: y= -1.012
// test #2: manual push backwards: y= -0.9818
// test #3: manual push backwards: y= -0.9872
// test #4: manual push backwards: y= -0.9857
// test #5: manual push backwards: y= -0.9686
// test #6: manual push backwards: y= -0.9561

/// (-1.012 + -0.9818 + -0.9872 + -0.9857 + -0.9686 + -0.9561) / 6.0 = -0.9819

public abstract class TeleOp extends OpMode   {
    public GamepadEx driver = null;
    public GamepadEx operator = null;
    public VoltageSensor battery;
    public Arm arm = null;
    public Drive drive = null;

    public enum Alliance {RED, BLUE};

    @Override
    public void init() {
        drive = new Drive(hardwareMap, telemetry);
//        arm = new Arm(hardwareMap);

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        battery = hardwareMap.voltageSensor.get("Control Hub");
    }

    @Override
    public void init_loop() {
        // runs while the robot is "on" but we haven't pressed "play"
        // yet
    }

    public abstract Alliance getAlliance();

    @Override
    public void start() {
        // this runs once, when "go" is pressed
    }

    @Override
    public void loop() {
        // read controls and sensors
        driver.readButtons();
        operator.readButtons();
        drive.read_sensors();
        // arm.read_sensors();

        // perform operations (figure out what inputs to various other
        // subsystems we want)
        drive.humanInputs(driver);
//        arm.humanInputs(operator, time);

        // let each subsystem perform its output operations
        drive.loop();
        // arm.loop();

        TelemetryPacket pack = new TelemetryPacket();
        pack.put("battery", battery.getVoltage());
        FtcDashboard.getInstance().sendTelemetryPacket(pack);
        // note, seems that "drawing stuff" commands have to go in their own packet
    }

}
