// File:          Kuka.java
// Date:
// Description:
// Author:
// Modifications:

// Add webots classes
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Keyboard;
import com.cyberbotics.webots.controller.Joystick;

// Here is the main class of your controller.
// This class defines how to initialize and how to run your controller.
public class Kuka extends Robot {
  private int timeStep;

  private Base base;
  private Arm arm;
  private Gripper gripper;
  private Kinect kinect;
  private Keyboard keyboard;
  private Joystick joystick;

  public Kuka() {
    // get the time step of the current world.
    timeStep = (int) Math.round(getBasicTimeStep());

    base = new Base(this, timeStep);
    arm = new Arm(this);
    gripper = new Gripper(this);
    kinect = new Kinect(this, timeStep);
  }
  
  private void step() {
    if (step(timeStep) == -1)
      System.exit(0);
  }
  
  private void passiveWait(double sec) {
    double start_time = getTime();
    do {
      step();
    } while (start_time + sec > getTime());
  }
  
  public void highLevelGoTo(double x, double y, double a) {
    base.setTarget(x, y, a);
    while (!base.reached()) {
      base.run();
      step();
    }
    base.reset();
  }
     
  private void automaticBehavior() {
    int GOTO_SPM = 0;
    double[][] goto_info = {{-0.989, 0.3, -Math.PI/2}};
    highLevelGoTo(goto_info[GOTO_SPM][0], goto_info[GOTO_SPM][1], goto_info[GOTO_SPM][2]);
  }

  private void displayHelperMessage() {
    System.out.println("\n\nKeyboard commands:");
    System.out.println(" Arrows:         Move the robot");
    System.out.println(" Page Up/Down:   Rotate the robot");
    System.out.println(" +/-:            (Un)grip");
    System.out.println(" Shift + arrows: Handle the arm");
    System.out.println(" Space:          Reset");

    if (joystick.isConnected()) {
      System.out.println("\n\nJoystick commands:");
      System.out.println(" Left thumbstick:  Move the robot");
      System.out.println(" Right thumbstick: Rotate the robot");
      System.out.println(" L(R) bumbers:     (Un)grip");
      System.out.println(" D-pad/XYAB:       Handle the arm");
      System.out.println(" Back, Start:      Reset");
    }
/*
    if (joystick.isConnected()) {
      System.out.printf("\n\nFound joystick: %s\n", joystick.getModel());
      int nAxes = joystick.getNumberOfAxes();
      int nPovs = joystick.getNumberOfPovs();
      System.out.printf("Number of axes: %d\n", nAxes);
      System.out.printf("Number of point of views (POV): %d\n", nPovs);
      while(true) {
        System.out.print("Axes:");
        for (int i = 0; i < nAxes; i++) {
          System.out.printf(" %d", joystick.getAxisValue(i));
        }
        System.out.print("  POVs:");
        for (int i = 0; i < nPovs; i++) {
          System.out.printf(" %d", joystick.getPovValue(i));
        }
        System.out.print("  Buttons:");
        int i = joystick.getPressedButton();
        while (i >= 0) {
          System.out.printf(" %d", i);
          i = joystick.getPressedButton();
        }
        System.out.println();
        passiveWait(0.1);
      }
    }
*/
  }
  
  public void run(String[] args) {
    passiveWait(2.0);

    if (args.length > 0 && args[0].equals("AUTO"))
      automaticBehavior();
    
    keyboard = getKeyboard();
    keyboard.enable(timeStep);
    joystick = getJoystick();
    joystick.enable(timeStep);
    passiveWait(2.0);
    
    displayHelperMessage();

    // Main loop:
    // - perform simulation steps until Webots is stopping the controller
    int pc = 0;
    int pd = 0;
    int pb = 0;
    while (step(timeStep) != -1) {
      
      int c = keyboard.getKey();
      if ((c >= 0) && (c != pc))
        switch (c) {
          case Keyboard.UP:
            base.forwardsIncrement();
            break;
          case Keyboard.DOWN:
            base.backwardsIncrement();
            break;
          case Keyboard.LEFT:
            base.strafeLeftIncrement();
            break;
          case Keyboard.RIGHT:
            base.strafeRightIncrement();
            break;
          case Keyboard.PAGEUP:
            base.turnLeftIncrement();
            break;
          case Keyboard.PAGEDOWN:
            base.turnRightIncrement();
            break;
          case Keyboard.END:
          case ' ':
            System.out.println("Reset");
            base.reset();
            arm.reset();
            break;
          case '+':
          case 388:
          case 65585:
          case 65579:
            System.out.println("Grip");
            gripper.grip();
            break;
          case '-':
          case 390:
            System.out.println("Ungrip");
            gripper.release();
            break;
          case 332:
          case Keyboard.UP | Keyboard.SHIFT:
            System.out.println("Increase arm height");
            arm.increaseHeight();
            break;
          case 326:
          case Keyboard.DOWN | Keyboard.SHIFT:
            System.out.println("Decrease arm height");
            arm.decreaseHeight();
            break;
          case 330:
          case Keyboard.RIGHT | Keyboard.SHIFT:
            System.out.println("Increase arm orientation");
            arm.increaseOrientation();
            break;
          case 328:
          case Keyboard.LEFT | Keyboard.SHIFT:
            System.out.println("Decrease arm orientation");
            arm.decreaseOrientation();
            break;
          default:
            System.err.println("Wrong keyboard input");
            break;
        }
      pc = c;
      
      if (joystick.isConnected()) {
        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double vx    = -Base.MAX_SPEED*joystick.getAxisValue(0)/32768;
        double vy    = -Base.MAX_SPEED*joystick.getAxisValue(1)/32768;
        double omega = -Base.MAX_SPEED*joystick.getAxisValue(3)/32768;
        base.move(vx, vy, omega);
  
        int d = joystick.getPovValue(0);
        if ((d > 0) && (d != pd))
          switch (d) {
            case 1: // D-pad Up
              System.out.println("Increase arm height");
              arm.increaseHeight();
              break;
            case 16: // D-pad Down
              System.out.println("Decrease arm height");
              arm.decreaseHeight();
              break; // D-pad Right
            case 256:
              System.out.println("Increase arm orientation");
              arm.increaseOrientation();
              break; // D-pad Left
            case 4096:
              System.out.println("Decrease arm orientation");
              arm.decreaseOrientation();
              break;
          }
        pd = d;
        
        int b = joystick.getPressedButton();
        if ((b >= 0) && (b != pb))
          switch (b) {
            case 0: // Start button
            case 1: // Back button
              System.out.println("Reset");
              base.reset();
              arm.reset();
              break;
            case 4: // Left Shoulder Button
              System.out.println("Grip");
              gripper.grip();
              break;
            case 5: // Right Shoulder Button
              System.out.println("Ungrip");
              gripper.release();
              break;
            case 8: // A button
              System.out.println("Decrease arm height");
              arm.decreaseHeight();
              break;
            case 9: // B button
              System.out.println("Increase arm orientation");
              arm.increaseOrientation();
              break;
            case 10: // X button
              System.out.println("Decrease arm orientation");
              arm.decreaseOrientation();
              break;
            case 11: // Y button
              System.out.println("Increase arm height");
              arm.increaseHeight();
              break;
          }
        pb = b;
      }

//      for (int i = 0; i < base.wheels.length; i++)
//        System.out.printf("Wheel %d: pos=%.2f[%.2f-%.2f] vel=%.2f[%.2f] acc=%.2f force=%.2f[%.2f] torque=%.2f[%.2f] mult=%.2f\n", i, base.wheels[i].getTargetPosition(), base.wheels[i].getMinPosition(), base.wheels[i].getMaxPosition(), base.wheels[i].getVelocity(), base.wheels[i].getMaxVelocity(), base.wheels[i].getAcceleration(), base.wheels[i].getAvailableForce(), base.wheels[i].getMaxForce(), base.wheels[i].getAvailableTorque(), base.wheels[i].getMaxTorque(), base.wheels[i].getMultiplier());
//      for (int i=0; i < arm.armElements.length; i++)
//        System.out.printf("Arm %d: pos=%.2f[%.2f-%.2f] vel=%.2f[%.2f] acc=%.2f force=%.2f[%.2f] torque=%.2f[%.2f] mult=%.2f\n", i, arm.armElements[i].getTargetPosition(), arm.armElements[i].getMinPosition(), arm.armElements[i].getMaxPosition(), arm.armElements[i].getVelocity(), arm.armElements[i].getMaxVelocity(), arm.armElements[i].getAcceleration(), arm.armElements[i].getAvailableForce(), arm.armElements[i].getMaxForce(), arm.armElements[i].getAvailableTorque(), arm.armElements[i].getMaxTorque(), arm.armElements[i].getMultiplier());

    }
  }
  
  public static void main(String[] args) {
    Kuka controller = new Kuka();
    controller.run(args);
  }
}
