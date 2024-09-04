import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Motor;

public class Arm {
  protected Motor[] armElements;
  private Height currentHeight;
  private Orientation currentOrientation;

  private static enum Height {
      ARM_BACK_PLATE_LOW, ARM_BACK_PLATE_HIGH, ARM_RESET, ARM_FRONT_CARDBOARD_BOX, ARM_HANOI_PREPARE, ARM_FRONT_PLATE, ARM_FRONT_FLOOR;

      public Height next() {
        Height[] heights = Height.values();
        // Prevents from going beyond index.
        return heights[Math.min(this.ordinal() + 1, heights.length - 1)];
      }
      
      public Height previous() {
          Height[] heights = Height.values();
          return heights[Math.max(this.ordinal() - 1, 0)];
      }
  }
  private static enum Orientation {
      ARM_BACK_RIGHT, ARM_RIGHT, ARM_FRONT_RIGHT, ARM_FRONT, ARM_FRONT_LEFT, ARM_LEFT, ARM_BACK_LEFT;

      public Orientation next() {
        Orientation[] orientations = Orientation.values();
        // Prevents from going beyond index.
        return orientations[Math.min(this.ordinal() + 1, orientations.length - 1)];
      }
      
      public Orientation previous() {
          Orientation[] orientations = Orientation.values();
          return orientations[Math.max(this.ordinal() - 1, 0)];
      }
  }
  public Arm(Robot robot) {
    armElements = new Motor[] {robot.getMotor("arm1"),robot.getMotor("arm2"),robot.getMotor("arm3"),robot.getMotor("arm4"),robot.getMotor("arm5")};
    for (int i = 0; i < armElements.length; i++)
      armElements[i].setVelocity(0.5);
    reset();
  }
  
  public void reset() {
    setHeight(Height.ARM_RESET);
    setOrientation(Orientation.ARM_FRONT);
  }

  private void setHeight(Height height) {
    switch (height) {
      case ARM_FRONT_FLOOR:
        armElements[1].setPosition(-0.97);
        armElements[2].setPosition(-1.55);
        armElements[3].setPosition(-0.61);
        armElements[4].setPosition(0.0);
        break;
      case ARM_FRONT_PLATE:
        armElements[1].setPosition(-0.62);
        armElements[2].setPosition(-0.98);
        armElements[3].setPosition(-1.53);
        armElements[4].setPosition(0.0);
        break;
      case ARM_FRONT_CARDBOARD_BOX:
        armElements[1].setPosition(0.0);
        armElements[2].setPosition(-0.77);
        armElements[3].setPosition(-1.21);
        armElements[4].setPosition(0.0);
        break;
      case ARM_RESET:
        armElements[1].setPosition(1.57);
        armElements[2].setPosition(-2.635);
        armElements[3].setPosition(1.78);
        armElements[4].setPosition(0.0);
        break;
      case ARM_BACK_PLATE_HIGH:
        armElements[1].setPosition(0.678);
        armElements[2].setPosition(0.682);
        armElements[3].setPosition(1.74);
        armElements[4].setPosition(0.0);
        break;
      case ARM_BACK_PLATE_LOW:
        armElements[1].setPosition(0.92);
        armElements[2].setPosition(0.42);
        armElements[3].setPosition(1.78);
        armElements[4].setPosition(0.0);
        break;
      case ARM_HANOI_PREPARE:
        armElements[1].setPosition(-0.4);
        armElements[2].setPosition(-1.2);
        armElements[3].setPosition(-Math.PI/2);
        armElements[4].setPosition(Math.PI/2);
        break;
      default:
        System.err.println("Arm.setHeight() called with a wrong argument");
        return;
    }
    currentHeight = height;
  }
  
  private void setOrientation(Orientation orientation) {
    switch (orientation) {
      case ARM_BACK_LEFT:
        armElements[0].setPosition(-2.949);
        break;
      case ARM_LEFT:
        armElements[0].setPosition(-Math.PI/2);
        break;
      case ARM_FRONT_LEFT:
        armElements[0].setPosition(-0.2);
        break;
      case ARM_FRONT:
        armElements[0].setPosition(0.0);
        break;
      case ARM_FRONT_RIGHT:
        armElements[0].setPosition(0.2);
        break;
      case ARM_RIGHT:
        armElements[0].setPosition(Math.PI/2);
        break;
      case ARM_BACK_RIGHT:
        armElements[0].setPosition(2.949);
        break;
      default:
        System.err.println("Arm.setOrientation() called with a wrong argument");
        return;
    }
    currentOrientation = orientation;
  }
  
    public void increaseHeight() {
      Height newHeight = currentHeight.next();
      // Prevents self-colliding poses.
      if (newHeight == Height.ARM_FRONT_FLOOR && (currentOrientation == Orientation.ARM_BACK_LEFT || currentOrientation == Orientation.ARM_BACK_RIGHT))
        newHeight = currentHeight;
      setHeight(newHeight);
    }
    
    public void decreaseHeight() {
      Height newHeight = currentHeight.previous();
      setHeight(newHeight);
    }

    public void increaseOrientation() {
      Orientation newOrientation = currentOrientation.next();
      // Prevents self-colliding poses.
      if (newOrientation == Orientation.ARM_BACK_LEFT && currentHeight == Height.ARM_FRONT_FLOOR)
        newOrientation = currentOrientation;
      setOrientation(newOrientation);
    }

    public void decreaseOrientation() {
      Orientation newOrientation = currentOrientation.previous();
      // Prevents self-colliding poses.
      if (newOrientation == Orientation.ARM_BACK_RIGHT && currentHeight == Height.ARM_FRONT_FLOOR)
        newOrientation = currentOrientation;
      setOrientation(newOrientation);
    }
}