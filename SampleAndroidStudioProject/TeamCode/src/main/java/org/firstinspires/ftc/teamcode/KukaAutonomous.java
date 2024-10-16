package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.kuka.Arm;
import org.firstinspires.ftc.teamcode.kuka.Drive;
import org.firstinspires.ftc.teamcode.kuka.Gripper;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.ftc6448.simulator.PlatformSupport;

@Autonomous(name = "Autonomous", group = "Kuka")
@Disabled
public class KukaAutonomous extends OpMode {

    // Define robot hardware objects  (Make them private so they can't be accessed externally)
    private Drive drive;
    private Arm arm;
    private Gripper gripper;
    private WebcamName camera;
    //private DistanceSensor range;

    private final ElapsedTime runtime = new ElapsedTime();

    private final SparkFunOTOS.Pose2D[] targets = {
            new SparkFunOTOS.Pose2D(0, -0.9, 0),
            new SparkFunOTOS.Pose2D(0.8, -1.4, -135),
            new SparkFunOTOS.Pose2D(0, -0.9, 0),
            new SparkFunOTOS.Pose2D(0.9, -1, -90),
            new SparkFunOTOS.Pose2D(0.9, 0, -90),
            new SparkFunOTOS.Pose2D(1.22, -0.33, -180),
            new SparkFunOTOS.Pose2D(1.22, -1.2, -180),
            new SparkFunOTOS.Pose2D(1.22, -0.2, -180),
            new SparkFunOTOS.Pose2D(1.48, -0.33, -180),
            new SparkFunOTOS.Pose2D(1.48, -1.2, -180),
            new SparkFunOTOS.Pose2D(1.48, -0.2, -180),
            new SparkFunOTOS.Pose2D(1.6, -0.33, -180),
            new SparkFunOTOS.Pose2D(1.6, -1.2, -180),
    };
    int currentTarget = 0;

    @Override public void init() {

        // Initialize robot hardware objects (note: need to use reference to actual OpMode).
        drive = new Drive(hardwareMap);
        arm = new Arm(hardwareMap);
        gripper = new Gripper(hardwareMap);
        camera = hardwareMap.get(WebcamName.class, "Webcam 1");
        telemetry.addData(">", "Hardware Initialized");
        telemetry.update();

    }

    @Override public void start() {
        // set starting position
        drive.setPosition(new SparkFunOTOS.Pose2D(0, -1.511, 0));
        runtime.reset();
    }

    @Override public void loop() {
        if (drive.targetReached(targets[currentTarget])) {
            drive.stop();
            currentTarget++;
            if (currentTarget == targets.length)
                currentTarget = 0;
        } else
            drive.loop();

        // Send telemetry messages to show robot status
        // to see telemetry in Webots, right click on your robot and select "Show Robot Window"
        telemetry.addData("Elapsed time", runtime.toString());
        telemetry.update();
    }

    @Override public void stop() {
        drive.stop();
        arm.resetPosition();
    }

}
