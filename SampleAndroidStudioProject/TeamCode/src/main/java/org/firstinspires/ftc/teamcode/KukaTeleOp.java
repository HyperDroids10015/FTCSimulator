package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Kuka;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.ftc6448.simulator.PlatformSupport;

@TeleOp(name = "TeleOp", group = "Kuka")
public class KukaTeleOp extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "kuka." to access this class.
    Kuka kuka = new Kuka(this);

    private final Gamepad currentGamepad1 = new Gamepad();
    private final Gamepad previousGamepad1 = new Gamepad();

    @Override
    public void runOpMode() {

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        kuka.init();

        // set starting position
        kuka.setBasePosition(new SparkFunOTOS.Pose2D(-1.5, 0.3, -90));

        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode use the Left stick to go forward & strafe, the Right stick to rotate left & right.
            double drive = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            //use PlatformSupport method to see if you are running on the simulator
            //if your motors on the real robot were different than the simulator, this could reverse the direction
            if (PlatformSupport.isSimulator())
                drive = -drive;

            // Combine drive, strafe and turn for blended motion. Use RobotHardware class
            kuka.driveRobot(drive, strafe, turn);

            // check to see if we need to change arm height or orientation.
            copy(gamepad1, currentGamepad1);

            if (currentGamepad1.y && !previousGamepad1.y)
                kuka.increaseArmHeight();
            else if (currentGamepad1.a && !previousGamepad1.a)
                kuka.decreaseArmHeight();

            if (currentGamepad1.x && !previousGamepad1.x)
                kuka.increaseArmOrientation();
            else if (currentGamepad1.b && !previousGamepad1.b)
                kuka.decreaseArmOrientation();

            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down)
                kuka.closeGripper();
            else if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up)
                kuka.openGripper();

            copy(currentGamepad1, previousGamepad1);

            SparkFunOTOS.Pose2D pos = kuka.getPosition();
            double[] arm = kuka.getArmPosition();

            // Send telemetry messages to explain controls and show robot status
            // to see telemetry in Webots, right click on your robot and select "Show Robot Window"
            telemetry.addData("Drive/Strafe", "Left Stick")
                     .addData("Turn", "Right Stick")
                     .addData("Arm Up/Down", "Y & A Buttons")
                     .addData("Arm Left/Right", "X & B Buttons")
                     .addData("Gripper Open/Closed", "Back/Start Buttons")
                     .addData("-", "-------")
                     .addData("Drive Power", "%.2f", drive)
                     .addData("Strafe Power", "%.2f", strafe)
                     .addData("Turn Power",  "%.2f", turn)
                     .addData("Arm Position",  "%.4f %.4f %.4f %.4f %.4f",arm[0],arm[1],arm[2],arm[3],arm[4])
//                    .addData("Gripper Position",  "Offset = %.2f", handOffset)
                    .addData("Robot Position", "x = %.2f, y = %.2f, h = %.2f", pos.x, pos.y, pos.h);
            telemetry.update();

            //on the real robot, this method call does nothing
            //on the simulator, it forces the opmode to sync its loop to Webots simulator time steps
            PlatformSupport.waitForSimulatorTimeStep();

        }
    }

    //to do: remove the following method by adding copy method to FTCSimulator Gamepad class
    private void copy(Gamepad g1, Gamepad g2) {
        if (PlatformSupport.isSimulator()) {
            g2.left_stick_x = g1.left_stick_x;
            g2.left_stick_y = g1.left_stick_y;
            g2.right_stick_x = g1.right_stick_x;
            g2.right_stick_y = g1.right_stick_y;
            g2.dpad_up = g1.dpad_up;
            g2.dpad_down = g1.dpad_down;
            g2.dpad_left = g1.dpad_left;
            g2.dpad_right = g1.dpad_right;
            g2.a = g1.a;
            g2.b = g1.b;
            g2.x = g1.x;
            g2.y = g1.y;
            g2.guide = g1.guide;
            g2.start = g1.start;
            g2.back = g1.back;
            g2.left_bumper = g1.left_bumper;
            g2.right_bumper = g1.right_bumper;
            g2.left_stick_button = g1.left_stick_button;
            g2.right_stick_button = g1.right_stick_button;
            g2.left_trigger = g1.left_trigger;
            g2.right_trigger = g1.right_trigger;
        } else
            g2.copy(g1);
    }
}
