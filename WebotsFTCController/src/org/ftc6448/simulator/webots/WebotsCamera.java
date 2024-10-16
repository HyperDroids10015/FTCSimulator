package org.ftc6448.simulator.webots;

import org.ftc6448.simulator.PlatformSupport;

import com.cyberbotics.webots.controller.Camera;
//import com.cyberbotics.webots.controller.CameraRecognitionObject;
import com.qualcomm.robotcore.util.SerialNumber;
import org.opencv.core.Size;
import org.opencv.core.Mat;
import org.opencv.core.CvType;

import io.github.deltacv.vision.internal.source.ftc.SourcedCameraName;
import io.github.deltacv.vision.external.source.VisionSource;
import io.github.deltacv.vision.external.source.VisionSourceBase;
import io.github.deltacv.vision.external.util.Timestamped;

import android.content.Context;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.robotcore.external.function.Continuation;

//import org.opencv.highgui.HighGui;
//import org.opencv.imgcodecs.Imgcodecs;

public class WebotsCamera extends SourcedCameraName {
	protected final String name;
	protected final Camera camera;
	protected final VisionSource source;

	protected class WebotsVisionSource extends VisionSourceBase {
		public int init() {
			return 0;
		}
		public boolean startSource(Size size) {
			return true;
		}
		public Timestamped<Mat> pullFrame() {
			int[] image = camera.getImage();
//				camera.saveImage("webots_frame.png", 0);
			Mat frame = new Mat(camera.getHeight(), camera.getWidth(), CvType.CV_8UC3);
			byte[] frameData = new byte[(int) (frame.total() * frame.channels())];
			for (int i=0; i<image.length; i++) {
				int pixel = image[i];
				frameData[3 * i]     = (byte)Camera.pixelGetRed(pixel);
				frameData[3 * i + 1] = (byte)Camera.pixelGetGreen(pixel);
				frameData[3 * i + 2] = (byte)Camera.pixelGetBlue(pixel);
			}
			frame.put(0,0,frameData);
//				HighGui.namedWindow("frame", HighGui.WINDOW_AUTOSIZE);
//				HighGui.imshow("frame", frame);
//				Imgcodecs.imwrite("frame.png", frame);
//				HighGui.waitKey();
/*
			if (camera.hasRecognition()) {
				System.out.println(camera.getRecognitionNumberOfObjects());
				CameraRecognitionObject[] objects = camera.getRecognitionObjects();
				for (CameraRecognitionObject obj : objects) {
					int objId = obj.getId();
					double[] objPosition = obj.getPosition();
					double[] objOrientation = obj.getOrientation();
					double[] objSize = obj.getSize();
					int[] objPositionOnImage = obj.getPositionOnImage();
					int[] objSizeOnImage = obj.getSizeOnImage();
					int objNumberOfColors = obj.getNumberOfColors();
					double[] objColors = obj.getColors();
					String objModel = obj.getModel();
					System.out.printf("Id: %d\n",objId);
					System.out.printf("Relative position: %f %f %f\n",objPosition[0],objPosition[1],objPosition[2]);
					System.out.printf("Relative orientation: %f %f %f %f\n",objOrientation[0],objOrientation[1],objOrientation[2],objOrientation[3]);
					System.out.printf("Size: %f %f\n",objSize[0],objSize[1]);
					System.out.printf("Position on the image: %d %d\n",objPositionOnImage[0],objPositionOnImage[1]);
					System.out.printf("Size on the image: %d %d\n",objSizeOnImage[0],objSizeOnImage[1]);
					for (int i=0; i<objNumberOfColors; i++)
						System.out.printf("Color %d: %f %f %f\n",i,objColors[0],objColors[1],objColors[2]);
					System.out.println();
				}
			} else
				System.out.println("recognition disabled");
*/
			long timestamp = PlatformSupport.getCurrentTimeMillis(); //System.currentTimeMillis();
			return new Timestamped<Mat>(frame, timestamp);
		}
		public  boolean stopSource() { return true; }
		public boolean close() { return true; }
	};

	public WebotsCamera(String name, Camera camera) {
		this.camera=camera;
		this.name=name;
		this.source=new WebotsVisionSource();
	}

	public VisionSource getSource() {
		return source;
	}
	public boolean isAttached() {
		return false;
	}
	public String getUsbDeviceNameIfAttached() {
		return null;
	}
	public SerialNumber getSerialNumber() {
		return null;
	}
	public void asyncRequestCameraPermission(Context context, Deadline deadline, Continuation continuation) {
	}
	public boolean requestCameraPermission(Deadline deadline) {
		return false;
	}
	public void close() {
	}
	public void resetDeviceConfigurationForOpMode() {
	}
	public int getVersion() {
		return 0;
	}
	public String getConnectionInfo() {
		return null;
	}
	public String getDeviceName() {
		return null;
	}
	public Manufacturer getManufacturer() {
		return null;
	}
}
