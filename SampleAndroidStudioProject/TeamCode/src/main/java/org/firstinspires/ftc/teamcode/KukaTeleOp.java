package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.kuka.Arm;
import org.firstinspires.ftc.teamcode.kuka.Drive;
import org.firstinspires.ftc.teamcode.kuka.Gripper;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

@TeleOp(name = "TeleOp", group = "Kuka")
@Disabled
public class KukaTeleOp extends OpMode {
    GamepadEx driver;
    //GamepadEx operator;

    // Define robot hardware objects  (Make them private so they can't be accessed externally)
    private Drive drive;
    private Arm arm;
    private Gripper gripper;
    //private WebcamName camera;
    //private DistanceSensor range;

    @Override public void init() {
        driver = new GamepadEx(gamepad1);
        //operator = new GamepadEx(gamepad2);

        // Initialize robot hardware objects (note: need to use reference to actual OpMode).
        drive = new Drive(hardwareMap);
        arm = new Arm(hardwareMap);
        gripper = new Gripper(hardwareMap);
//        camera = hardwareMap.get(WebcamName.class, "Webcam 1");
        telemetry.addData(">", "Hardware Initialized");
        telemetry.update();

        // set starting position
//        drive.setPosition(new SparkFunOTOS.Pose2D(0, -1.511, 0));
    }

    @Override public void loop() {
        // read controls and sensors
        driver.readButtons();
        //operator.readButtons();

        // perform operations (figure out what inputs to various other subsystems we want)
        drive.humanInputs(driver);
        arm.humanInputs(driver);//operator
        gripper.humanInputs(driver);//operator

        // Combine drive, strafe and turn for blended motion. Use RobotHardware class
        drive.loop();

        SparkFunOTOS.Pose2D drivePosition = drive.getPosition();;
        double[] armPosition = arm.getPosition();

        // Send telemetry messages to explain controls and show robot status
        // to see telemetry in Webots, right click on your robot and select "Show Robot Window"
        telemetry.addData("Drive/Strafe", "Left Stick")
                 .addData("Turn", "Right Stick")
                 .addData("Arm Up/Down", "Y & A Buttons")
                 .addData("Arm Left/Right", "X & B Buttons")
                 .addData("Gripper Open/Closed", "Left & Right Bumpers")
                 .addData("-", "-------")
                 .addData("Arm Position",  "%.4f %.4f %.4f %.4f %.4f",armPosition[0],armPosition[1],armPosition[2],armPosition[3],armPosition[4])
                 .addData("Robot Position", "x = %4.2f, y = %4.2f, h = %4.2f", drivePosition.x, drivePosition.y, drivePosition.h);
//                    .addData("Gripper Position",  "Offset = %.2f", handOffset)
        telemetry.update();
    }

    @Override public void stop() {
        drive.stop();
        arm.resetPosition();
    }
}
