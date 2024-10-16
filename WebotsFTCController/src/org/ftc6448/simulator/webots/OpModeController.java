package org.ftc6448.simulator.webots;

import java.util.Properties;

import org.ftc6448.simulator.Controller;
import org.ftc6448.simulator.PlatformSupport;

import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Device;
import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.GPS;
//import com.cyberbotics.webots.controller.Compass;
import com.cyberbotics.webots.controller.InertialUnit;
import com.cyberbotics.webots.controller.Keyboard;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.PositionSensor;
import com.cyberbotics.webots.controller.Supervisor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.studiohartman.jamepad.ControllerManager;

public class OpModeController implements Controller {

	protected final OpMode opMode;
	protected final Supervisor supervisor;
	protected final Properties properties;
	
	public final int timeStep;
	protected GamepadSupport gamepadSupport;
	protected ControllerManager controllerManager;
	protected Keyboard keyboard;
	int prevkey=0;
	
	public OpModeController(Supervisor supervisor,OpMode opMode,Properties properties) {
		this.opMode = opMode;
		this.supervisor=supervisor;
		this.properties=properties;
		// get the time step of the current world.
		timeStep = (int) Math.round(supervisor.getBasicTimeStep());
		System.out.println("timeStep " + timeStep);
	
		opMode.gamepad1=new Gamepad();
		opMode.gamepad2=new Gamepad();
		
	}
	
	@Override
	public void initialize() {
		keyboard = new Keyboard();
		keyboard.enable(timeStep);
		
		controllerManager = new ControllerManager();
		controllerManager.initSDLGamepad();
		
		gamepadSupport=new GamepadSupport(properties, controllerManager);
		
		initializeDevices();
		opMode.internalPreInit();
		System.out.println("Calling OpMode init");
		opMode.init();
		opMode.internalPostInitLoop();
	}
	
	//this method iterates the Robot and the simulation properties file and sets up the hardware map
	private void initializeDevices() {
		final HardwareMap hardwareMap=new HardwareMap();
		opMode.hardwareMap=hardwareMap;
		GPS gps = null;
		InertialUnit imu = null;
//		Compass compass = null;

		// add Control Hub as device
		System.out.println("Adding voltage sensor as Control Hub");
		hardwareMap.voltageSensor.put("Control Hub", new WebotsVoltageSensor());
		//load all motors into the hardware motor map
		for (int i=0;i<supervisor.getNumberOfDevices();i++) {
			Device device=supervisor.getDeviceByIndex(i);
			
			System.out.println(device+" "+i);

			if (device instanceof Camera) {
				Camera camera = (Camera)device;
				camera.enable(timeStep);
//				camera.recognitionEnable(timeStep);
				String mappedName=properties.getProperty(device.getName());
				if (mappedName!=null) {
					System.out.println("Loading webots Camera device " + device.getName() + " as " + mappedName);
				}
				else {
					mappedName=device.getName();
					System.out.println("Loading webots Camera device " + mappedName);
				}
				hardwareMap.put(mappedName, new WebotsCamera(mappedName,camera));
			}
			else if (device instanceof GPS) {
				//SparkFun Qwiic Optical Tracking Odometry Sensor (OTOS)
				gps=(GPS)device;
				gps.enable(timeStep);

				if (imu!=null) {
					String mappedName=properties.getProperty(device.getName());
					if (mappedName!=null) {
						System.out.println("Loading webots GPS and InertialUnit devices (" + gps.getName() + " and " + imu.getName()+") as "+mappedName);
					}
					else {
						mappedName=device.getName();
						System.out.println("Loading webots GPS and InertialUnit devices (" + gps.getName() + " and " + imu.getName()+")");
					}
					hardwareMap.put(mappedName, new WebotsSparkFunOTOS(mappedName,gps,imu));
				}
			}
			else if (device instanceof InertialUnit) {
				//SparkFun Qwiic Optical Tracking Odometry Sensor (OTOS)
				imu=(InertialUnit)device;
				imu.enable(timeStep);

				if (gps!=null) {
					String mappedName=properties.getProperty(device.getName());
					if (mappedName!=null) {
						System.out.println("Loading webots GPS and InertialUnit devices (" + gps.getName() + " and " + imu.getName()+") as "+mappedName);
					}
					else {
						mappedName=device.getName();
						System.out.println("Loading webots GPS and InertialUnit devices (" + gps.getName() + " and " + imu.getName()+")");
					}
					hardwareMap.put(mappedName, new WebotsSparkFunOTOS(mappedName,gps,imu));
				}
			}
/*
			else if (device instanceof Compass) {
				//SparkFun Qwiic Optical Tracking Odometry Sensor (OTOS)
				compass=(Compass)device;
				compass.enable(timeStep);

				if (gps!=null) {
					String mappedName=properties.getProperty(device.getName());
					if (mappedName!=null) {
						System.out.println("Loading webots GPS and Compass devices (" + gps.getName() + " and " + compass.getName()+") as "+mappedName);
					}
					else {
						mappedName=device.getName();
						System.out.println("Loading webots GPS and Compass devices (" + gps.getName() + " and " + compass.getName()+")");
					}
					hardwareMap.put(mappedName, new WebotsSparkFunOTOS(mappedName,gps,compass));
				}
			}
			else if (device instanceof InertialUnit) {
				//only one imu is supported
				InertialUnit imu = (InertialUnit)device;
				imu.enable(timeStep);
				String mappedName=properties.getProperty(device.getName());
				if (mappedName!=null) {
					System.out.println("Loading webots InertialUnit device " + device.getName() + " as " + mappedName);
				}
				else {
					mappedName=device.getName();
					System.out.println("Loading webots InertialUnit device " + mappedName);
				}
				hardwareMap.put(mappedName, new WebotsIMU(imu));
			}
*/
			else if (device instanceof Motor) {
				Motor motor=(Motor)device;
				
				//if there is an associated position sensor, enable it
				PositionSensor sensor=motor.getPositionSensor();
				if (sensor!=null) {
					sensor.enable(timeStep);
				}
				
				String mappedName=properties.getProperty(device.getName());
				if (mappedName!=null) {
					System.out.println("Loading webots motor " + device.getName()+" as "+mappedName);
				}
				else {
					mappedName=device.getName();
					System.out.println("Loading webots motor " + mappedName);
				}
				
				String type=properties.getProperty(mappedName+".type");
				if ("servo".equalsIgnoreCase(type)) {
					String baseRotationProperty=device.getName()+".baseRotation";
					String baseRotation=properties.getProperty(baseRotationProperty);
					if (baseRotation==null||baseRotation.trim().length()==0) {
						System.out.println("No property found for "+baseRotationProperty+", so using 0 as default");
						baseRotation="0";
					}
					System.out.println("Webots motor "+mappedName+" is a servo");
	    			WebotsServoImpl servo=new WebotsServoImpl(mappedName,motor);
	    			servo.setBaseRotation(Float.parseFloat(baseRotation));
	    			hardwareMap.put(mappedName, servo);
	    		}
				
				else {
					WebotsDcMotorImpl webotsMotor=new WebotsDcMotorImpl(mappedName,motor);
					
					String maxPowerProperty=device.getName()+".maxPower";
					String maxPower=properties.getProperty(maxPowerProperty);
					if (maxPower==null||maxPower.trim().length()==0) {
						System.out.println("No property found for "+maxPowerProperty+", so using 10 as default");
						maxPower="10";
					}
	
					System.out.println("Using "+maxPower+" for property "+maxPowerProperty);
					webotsMotor.setMaxPower(Float.parseFloat(maxPower));
					
					hardwareMap.dcMotor.put(mappedName, webotsMotor);	
				}
			}		
			
			
		}
		
		//add any missing motors
	    for (Object key:properties.keySet()) {
	    	String property=(String)key;
	    	HardwareDevice webotsDevice=null;
	    	if (property.endsWith(".type")) {
	    		String device=property.substring(0,property.length()-5);
	    		String type=properties.getProperty(property);
	    		if ("servo".equalsIgnoreCase(type)) {
	    			webotsDevice=new WebotsServo();
	    		}
	    		else if ("continuousServo".equalsIgnoreCase(type)) {
	    			webotsDevice=new WebotsContinuousServo();
	    		}
	    		else if ("motor".equalsIgnoreCase(type)) {
	    			webotsDevice=new WebotsDcMotor(device);
	    		}
	    		else if ("distance".equalsIgnoreCase(type)) {
	    			webotsDevice=new WebotsDistanceSensor(device);
	    		}
	    		else if ("digitalChannel".equalsIgnoreCase(type)) {
					//TODO: should this do something
					WebotsDigitalChannel channel=new WebotsDigitalChannel(device);

					System.out.println("Device "+device+" is a digital channel");
	    			hardwareMap.put(device, channel);
	    		}
	    		if (webotsDevice!=null &&  !hardwareMap.hasDevice(device)) {
	    			System.out.println("Adding empty "+type+" implementation for "+device);
	    			hardwareMap.put(device, webotsDevice);
	    		}
	    	}
	    }
		
	}

	@Override
	public void run() {
		boolean useKeyboard="true".equalsIgnoreCase(properties.getProperty("emulateGamepadsWithKeyboard"));

		System.out.println("Starting OpMode");
		opMode.start();
		if (opMode instanceof LinearOpMode) {
			LinearOpMode linearOpMode=(LinearOpMode)opMode;
			
			//Autonomous opmodes need special coordination between the OpMode loop and the simulator loop
			System.out.println("Running LinearOpMode");
		

			//if sleep time is 0, then we signal the simulator lock and wait to be signaled back
			//this effectively locks the OpMode frequency to the simulator
			//if sleep time is not 0, then the simulator lock is signaled and then the simulator will wait the associated time
			
			long sleepTime=0;
			String simSleepTimeStr=properties.getProperty("simulatorLoopSleepTime");
			if (simSleepTimeStr!=null&&simSleepTimeStr.trim().length()>0) {
				sleepTime=Long.parseLong(simSleepTimeStr);
			}

			while (supervisor.step(timeStep) != -1) {
				handleGamepads(useKeyboard);
				linearOpMode.loop();
				linearOpMode.internalPostLoop();

				//signal any threads that are waiting for a simulator tick
				PlatformSupport.signalSimulatorLock(sleepTime==0);
//				if (linearOpMode.isStopped()) {
//					System.out.println("OpMode stopped");
//				}
				if (sleepTime>0) { 
					try {
						//we want to sleep a little bit to allow the other code to keep up with us
						//if this thread is running faster than the OpMode, the OpMode will not be running at the proper frequency
						Thread.sleep(sleepTime);
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				}
			}
		}
		else {
			//TeleOp opmodes just loop and call the OpMode loop method along with the simulator step
			System.out.println("Running OpMode");
			while (supervisor.step(timeStep) != -1) {
				handleGamepads(useKeyboard);
				opMode.loop();
				opMode.internalPostLoop();

				//signal any threads that are waiting for a simulator tick (TeleOp opmodes should not, but do it just in case)
//				PlatformSupport.signalSimulatorLock(true);
			}
		}
	}

	private void handleGamepads(boolean useKeyboard) {
		
		if (!useKeyboard) {
			gamepadSupport.processJoystick(opMode.gamepad1, opMode.gamepad2);
		}
		else {
			//if we are using virtual gamepad, poll the keyboard
			int key = keyboard.getKey();
			if ((key >= 0) && (key != prevkey)) {
				switch (key) {
					case 3: // BACKSPACE
						opMode.gamepad1.back = true;
						break;
					case 5: // ENTER
						opMode.gamepad1.start = true;
						break;
					case 6: // INSERT
						opMode.gamepad1.y = true;
						break;
					case 7: // DELETE
						opMode.gamepad1.x = true;
						break;
					case Keyboard.END:
						opMode.gamepad1.a = true;
						break;
					case Keyboard.HOME:
						opMode.gamepad1.b = true;
						break;
					case Keyboard.LEFT:
						opMode.gamepad1.right_stick_x -= 0.2f;
						if (opMode.gamepad1.right_stick_x > 1f)
							opMode.gamepad1.right_stick_x = 1f;
						break;
					case Keyboard.UP:
						opMode.gamepad1.right_stick_y -= 0.2f;
						if (opMode.gamepad1.right_stick_y < -1f)
							opMode.gamepad1.right_stick_y = -1f;
						break;
					case Keyboard.RIGHT:
						opMode.gamepad1.right_stick_x += 0.2f;
						if (opMode.gamepad1.right_stick_x < -1f)
							opMode.gamepad1.right_stick_x = -1f;
						break;
					case Keyboard.DOWN:
						opMode.gamepad1.right_stick_y += 0.2f;
						if (opMode.gamepad1.right_stick_y > 1f)
							opMode.gamepad1.right_stick_y = 1f;
						break;
					case Keyboard.PAGEUP:
						opMode.gamepad1.left_stick_x -= 0.2f;
						if (opMode.gamepad1.left_stick_x < -1f)
							opMode.gamepad1.left_stick_x = -1f;
						break;
					case Keyboard.PAGEDOWN:
						opMode.gamepad1.left_stick_x += 0.2f;
						if (opMode.gamepad1.left_stick_x > 1f)
							opMode.gamepad1.left_stick_x = 1f;
						break;
					case '-':
						opMode.gamepad1.left_stick_y -= 0.2f;
						if (opMode.gamepad1.left_stick_y < -1f)
							opMode.gamepad1.left_stick_y = -1f;
						break;
					case '+':
						opMode.gamepad1.left_stick_y += 0.2f;
						if (opMode.gamepad1.left_stick_y > 1f)
							opMode.gamepad1.left_stick_y = 1f;
						break;
					case 11: // NUMPAD 5
					case ' ':
						opMode.gamepad1.right_stick_x = 0f;
						opMode.gamepad1.right_stick_y = 0f;
						opMode.gamepad1.left_stick_x = 0f;
						opMode.gamepad1.left_stick_y = 0f;
						break;
					case '*':
						opMode.gamepad1.right_bumper = true;
						break;
					case '/':
						opMode.gamepad1.left_bumper = true;
						break;
					case '1':
						opMode.gamepad1.left_trigger -= 0.2f;
						if (opMode.gamepad1.left_trigger < -1f)
							opMode.gamepad1.left_trigger = -1f;
						break;
					case '2':
						opMode.gamepad1.dpad_down = true;
						break;
					case '3':
						opMode.gamepad1.right_trigger -= 0.2f;
						if (opMode.gamepad1.right_trigger < -1f)
							opMode.gamepad1.right_trigger = -1f;
						break;
					case '4':
						opMode.gamepad1.dpad_left = true;
						break;
					case '5':
						opMode.gamepad1.left_trigger = 0f;
						opMode.gamepad1.right_trigger = 0f;
					case '6':
						opMode.gamepad1.dpad_right = true;
						break;
					case '7':
						opMode.gamepad1.left_trigger += 0.2f;
						if (opMode.gamepad1.left_trigger > 1f)
							opMode.gamepad1.left_trigger = 1f;
						break;
					case '8':
						opMode.gamepad1.dpad_up = true;
						break;
					case '9':
						opMode.gamepad1.right_trigger += 0.2f;
						if (opMode.gamepad1.right_trigger > 1f)
							opMode.gamepad1.right_trigger = 1f;
						break;
					default:
						System.out.printf("Wrong keyboard input: %d\n", key);
						break;
				}
			} else {
					opMode.gamepad1.back = false;
					opMode.gamepad1.start = false;
					opMode.gamepad1.y = false;
					opMode.gamepad1.x = false;
					opMode.gamepad1.a = false;
					opMode.gamepad1.b = false;
					opMode.gamepad1.right_bumper = false;
					opMode.gamepad1.left_bumper = false;
					opMode.gamepad1.dpad_down = false;
					opMode.gamepad1.dpad_left = false;
					opMode.gamepad1.dpad_right = false;
					opMode.gamepad1.dpad_up = false;
			}
			prevkey=key;
		}
		
	}

	@Override
	public void cleanup() {
		opMode.stop();
		controllerManager.quitSDLGamepad();
	}

}
