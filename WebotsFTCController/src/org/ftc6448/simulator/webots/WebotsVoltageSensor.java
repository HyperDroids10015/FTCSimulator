package org.ftc6448.simulator.webots;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Voltage Sensor
 */
public class WebotsVoltageSensor implements HardwareDevice,VoltageSensor {

	public double getVoltage() {
		return 12.0;
	}

	@Override
	public Manufacturer getManufacturer() {
		return null;
	}

	@Override
	public String getDeviceName() {
		return null;
	}

	@Override
	public String getConnectionInfo() {
		return null;
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
