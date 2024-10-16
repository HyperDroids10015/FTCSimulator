package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@Autonomous(name = "Autonomous")
public class Auto extends OpMode   {
    public VoltageSensor battery;

    public Drive drive = null;

    // x, y are in meters, heading is degrees
    SparkFunOTOS.Pose2D[] targets = {
            new SparkFunOTOS.Pose2D(0, -0.9, 0),
            new SparkFunOTOS.Pose2D(0.8, -1.4, -135),
    };
    int currentTarget = 0;


    @Override
    public void init() {
        drive = new Drive(hardwareMap, telemetry);
//        arm = new Arm(hardwareMap);
        battery = hardwareMap.voltageSensor.get("Control Hub");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        // this runs once, when "go" is pressed
        drive.setPosition(new SparkFunOTOS.Pose2D(0,-1.614,0));
    }

    @Override
    public void loop() {
        drive.read_sensors();
        // arm.read_sensors();

        double dx = targets[currentTarget].x - drive.current_position.x;
        double dy = targets[currentTarget].y - drive.current_position.y;
        double dh = targets[currentTarget].h - drive.current_position.h;
        drive.robotInputs(dx * 4, dy * 4, -dh * 0.033);
        if (Math.hypot(dx,dy) < 0.05 && Math.abs(dh) < 2) {
            currentTarget++;
            if (currentTarget == targets.length)
                currentTarget = 0;
        }

        TelemetryPacket pack = new TelemetryPacket();
        pack.put("battery", battery.getVoltage());
        FtcDashboard.getInstance().sendTelemetryPacket(pack);

        drive.loop();
        // arm.loop();
    }

}
