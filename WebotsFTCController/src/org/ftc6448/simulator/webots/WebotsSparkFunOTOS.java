package org.ftc6448.simulator.webots;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.cyberbotics.webots.controller.GPS;
import com.cyberbotics.webots.controller.InertialUnit;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

public class WebotsSparkFunOTOS extends SparkFunOTOS {
    protected final String name;
    protected final GPS gps;
    protected final InertialUnit imu;
    protected Pose2D offset = new Pose2D();

    public WebotsSparkFunOTOS(String name, GPS gps, InertialUnit imu) {
        this.name = name;
        this.gps = gps;
        this.imu = imu;
        _distanceUnit = DistanceUnit.INCH;
        _angularUnit = AngleUnit.DEGREES;
    }

    /**
     * Gets the position measured by the OTOS
     * @return Position measured by the OTOS
     */
    public Pose2D getPosition() {
        double[] gpsRawValues = gps.getValues();
        double[] angles = imu.getRollPitchYaw();
        return new Pose2D(
                _distanceUnit.fromMeters(gpsRawValues[0]-offset.x),
                _distanceUnit.fromMeters(gpsRawValues[1]-offset.y),
                _angularUnit.fromRadians(angles[2]-offset.h));
    }
    public void setOffset(Pose2D pose) {
        offset.x = _distanceUnit.toMeters(pose.x);
        offset.y = _distanceUnit.toMeters(pose.y);
        offset.h = _angularUnit.toRadians(pose.h);
    }
}