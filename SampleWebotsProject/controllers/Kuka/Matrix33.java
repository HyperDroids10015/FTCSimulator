public class Matrix33 {
  protected Vector3 a, b, c; // matrix columns
  
  // Constructor
  public Matrix33(Vector3 a, Vector3 b, Vector3 c) {
    this.a = a;
    this.b = b;
    this.c = c;
  }
  
  public Vector3 multVector3(Vector3 v) {
    double x = a.x * v.x + b.x * v.y + c.x * v.z;
    double y = a.y * v.x + b.y * v.y + c.y * v.z;
    double z = a.z * v.x + b.z * v.y + c.z * v.z;
    return new Vector3(x, y, z);
  }
  
}
