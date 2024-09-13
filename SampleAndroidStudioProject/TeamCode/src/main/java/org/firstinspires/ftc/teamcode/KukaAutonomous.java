package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Kuka;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ftc6448.simulator.PlatformSupport;

@Autonomous(name = "Autonomous", group = "Kuka")
public class KukaAutonomous extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "kuka." to access this class.
    Kuka kuka = new Kuka(this);

    protected final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        kuka.init();

        // set starting position
        kuka.setBasePosition(new SparkFunOTOS.Pose2D(-1.5, 0.3, -90));

        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        SparkFunOTOS.Pose2D[] targets = {
                new SparkFunOTOS.Pose2D(0.0, 1.2, 0.0),
//                new SparkFunOTOS.Pose2D(0.9, 0.3, 90.0),
//                new SparkFunOTOS.Pose2D(0.0, 0.3, 180.0),
//                new SparkFunOTOS.Pose2D(-1.5, 0.3, -90.0),
        };
        int currentTarget = 0;

        while (opModeIsActive()) {
            if (kuka.targetReached()) {
                if (currentTarget < targets.length) {
                    kuka.setTarget(targets[currentTarget]);
                    currentTarget++;
                } //else
//                    currentTarget = 0;
            } else {
                kuka.runToTarget();

                SparkFunOTOS.Pose2D pos = kuka.getPosition();

                // Send telemetry messages to show robot status
                // to see telemetry in Webots, right click on your robot and select "Show Robot Window"
                telemetry.addData("Robot Position", "x = %4.2f, y = %4.2f, h = %4.2f", pos.x, pos.y, pos.h);
                telemetry.addData("Elapsed time", runtime.toString());
                telemetry.update();
            }

            //on the real robot, this method call does nothing
            //on the simulator, it forces the opmode to sync its loop to Webots simulator time steps
            PlatformSupport.waitForSimulatorTimeStep();
        }
    }
}
