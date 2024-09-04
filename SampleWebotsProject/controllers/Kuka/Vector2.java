public class Vector2 {
  protected double x, y;
  
  // Constructor
  public Vector2(double x, double y) {
    setValues(x, y);
  }

  public void setValues(double x, double y) {
    this.x = x;
    this.y = y;
  }
  
  // Method to compute the norm (magnitude) of the vector
  public double norm() {
    return Math.sqrt(x*x+y*y);
  }
  
  // Method to subtract another vector from this vector
  public Vector2 minus(Vector2 other) {
    return new Vector2(this.x - other.x, this.y - other.y);
  }
  
  // Method to compute the angle between this vector and another vector
  public double angle(Vector2 other) {
    double angle = Math.atan2(other.y,other.x) - Math.atan2(this.y,this.x);
    if (angle > Math.PI) angle -= 2*Math.PI;
    else if (angle <= -Math.PI) angle += 2*Math.PI;
    return angle;
  }
}
