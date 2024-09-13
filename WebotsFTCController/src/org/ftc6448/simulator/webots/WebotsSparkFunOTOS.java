package org.ftc6448.simulator.webots;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.cyberbotics.webots.controller.GPS;
import com.cyberbotics.webots.controller.Compass;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

public class WebotsSparkFunOTOS extends SparkFunOTOS {
    protected final String name;
    protected final GPS gps;
    protected final Compass compass;
    protected final Pose2D origin, delta;

    public WebotsSparkFunOTOS(String name, GPS gps, Compass compass) {
        this.gps = gps;
        this.compass = compass;
        this.name = name;
        _distanceUnit = DistanceUnit.INCH;
        _angularUnit = AngleUnit.DEGREES;
        origin = new Pose2D();
        delta = new Pose2D();
    }

    /**
     * Resets the tracking algorithm, which resets the position to the
     * origin, but can also be used to recover from some rare tracking errors
     */
    public void resetTracking() {
	setPosition(getPosition());
    }

    /**
     * Gets the position measured by the OTOS
     * @return Position measured by the OTOS
     */
    @Override
    public Pose2D getPosition() {
        double[] gpsRawValues = gps.getValues();
        double[] compassRawValues = compass.getValues();
        double h = -Math.atan2(compassRawValues[1],compassRawValues[0])+delta.h;
        if (h > Math.PI) h -= 2*Math.PI;
        else if (h <= -Math.PI) h += 2*Math.PI;

        // Store in pose and convert to units
        Pose2D pose = new Pose2D();
        pose.x = _distanceUnit.fromMeters(gpsRawValues[0]*Math.cos(delta.h)+gpsRawValues[1]*Math.sin(delta.h)+delta.x);
        pose.y = _distanceUnit.fromMeters(gpsRawValues[1]*Math.cos(delta.h)-gpsRawValues[0]*Math.sin(delta.h)+delta.y);
        pose.h = _angularUnit.fromRadians(h);

        return pose;
    }

    /**
     * Sets the position measured by the OTOS. This is useful if your
     * robot does not start at the origin, or you have another source of
     * location information (eg. vision odometry); the OTOS will continue
     * tracking from this position
     * @param pose New position for the OTOS to track from
     */
    public void setPosition(Pose2D pose) {
        delta.h = _angularUnit.toRadians(pose.h)-origin.h;
        delta.x = _distanceUnit.toMeters(pose.x)-origin.x*Math.cos(delta.h)-origin.y*Math.sin(delta.h);
        delta.y = _distanceUnit.toMeters(pose.y)-origin.y*Math.cos(delta.h)+origin.x*Math.sin(delta.h);
        origin.h = _angularUnit.toRadians(pose.h);
	origin.x = _distanceUnit.toMeters(pose.x);
	origin.y = _distanceUnit.toMeters(pose.y);
    }

}