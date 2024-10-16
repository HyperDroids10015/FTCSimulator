package org.ftc6448.simulator.webots;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.cyberbotics.webots.controller.InertialUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.hardware.QuaternionBasedImuHelper;

public class WebotsIMU implements HardwareDevice,IMU {
	protected volatile Parameters parameters;
	private static final String TAG = "Webots IMU";
	private final QuaternionBasedImuHelper helper;
	InertialUnit imu;

	public WebotsIMU(InertialUnit imu) {
		this.imu = imu;
		parameters = new Parameters(new RevHubOrientationOnRobot(
				RevHubOrientationOnRobot.LogoFacingDirection.UP,
				RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
		helper = new QuaternionBasedImuHelper(parameters.imuOrientationOnRobot);
		if (initialize(parameters))
			helper.resetYaw(TAG, this::getRawQuaternion, 0);
	}

	protected Quaternion getRawQuaternion() {
		// Robot Coordinate System is RFU in new FTC SDK 8.1 universal IMU driver vs FLU in older FTC SDK and new Webots 2023b
//		double[] angles = imu.getRollPitchYaw();
//		System.out.printf("roll = %f, pitch = %f, yaw = %f\n",angles[0],angles[1],angles[2]);
//		return new YawPitchRollAngles(AngleUnit.RADIANS, values[2], -values[1], values[0], 0);
		double[] values = imu.getQuaternion();
//		if (Math.abs(values[0]) + Math.abs(values[1]) + Math.abs(values[2]) + Math.abs(values[3]) < 1e-16) values[3]=1.0;
//		System.out.printf("x = %e, y = %e, z = %e, w = %e\n",values[0],values[1],values[2],values[3]);
		return new Quaternion((float)values[3], (float)-values[0],(float)-values[1], (float)values[2], 0);
	}

	protected AngularVelocity getRawAngularVelocity() {
		//return new AngularVelocity(AngleUnit.DEGREES, x, y, z, 0);
		return null;
	}

	@Override public boolean initialize(Parameters parameters) {
		parameters = parameters.copy();
		this.parameters = parameters;
		helper.setImuOrientationOnRobot(parameters.imuOrientationOnRobot);
		return true;
	}

	@Override public void resetYaw() {
		helper.resetYaw(TAG, this::getRawQuaternion, 0);
	}

	@Override public YawPitchRollAngles getRobotYawPitchRollAngles() {
		return helper.getRobotYawPitchRollAngles(TAG, this::getRawQuaternion);
	}

	@Override public Orientation getRobotOrientation(AxesReference reference, AxesOrder order, AngleUnit angleUnit) {
		return helper.getRobotOrientation(TAG, this::getRawQuaternion, reference, order, angleUnit);
	}

	@Override public Quaternion getRobotOrientationAsQuaternion() {
		return helper.getRobotOrientationAsQuaternion(TAG, this::getRawQuaternion, true);
	}

	@Override public AngularVelocity getRobotAngularVelocity(AngleUnit angleUnit) {
		return helper.getRobotAngularVelocity(getRawAngularVelocity(), angleUnit);
	}

	@Override public Manufacturer getManufacturer() {
		return null;
	}

	@Override public String getDeviceName() {
		return "Webots IMU";
	}

	@Override
	public String getConnectionInfo() {
		return "";
	}

	@Override
	public int getVersion() {
		return 0;
	}

	@Override
	public void resetDeviceConfigurationForOpMode() {
	}

	@Override
	public void close() {
	}

}
