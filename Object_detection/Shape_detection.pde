import gab.opencv.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Core;
import org.opencv.imgproc.Moments;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.CvType;

import org.opencv.core.Point;
import org.opencv.core.Size;

int Xs;
int Ys;
int Zs;

class Shape_Detector {

  private OpenCV opencv;

  private boolean foundShape = false;

  Shape_Detector(PApplet p, int w, int h) {
    opencv = new OpenCV(p, w, h);
    opencv.gray();
  }



  void processFrame(PImage videoFrame, boolean showDetection) {

    foundShape = false;

    ArrayList<MatOfPoint> contours  = new ArrayList<MatOfPoint>();
    Mat hierarchy = new Mat();

    opencv.loadImage(videoFrame);

    if (showDetection) image(kinect.rgbImage(), 0, 0);

    Mat thresholdMat = OpenCV.imitate(opencv.getGray());

    Imgproc.adaptiveThreshold(opencv.getGray(), thresholdMat, 255, Imgproc.ADAPTIVE_THRESH_GAUSSIAN_C, Imgproc.THRESH_BINARY_INV, 451, -65);

    Imgproc.findContours(thresholdMat, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE); 

    ArrayList<Moments> mu = new ArrayList<Moments>(contours.size());

    for ( int i = 0; i< contours.size (); i++) {   

      MatOfPoint c = contours.get(i);
      mu.add(i, Imgproc.moments(c, false));
      Moments p = mu.get(i);

      if (hierarchy.get(0, i)[3] != -1) {

        if (checkShape_detectedangle(c)) {

          foundShape = true;

          if (showDetection) {
            strokeWeight(5);
            stroke(0, 0, 255);

            beginShape();
            Point[] points = c.toArray();

            for (int j = 0; j < points.length; j++) {
              vertex((float)points[j].x, (float)points[j].y);
            }
            endShape();
          }

          int x = (int) (p.get_m10() / p.get_m00());
          int y = (int) (p.get_m01() / p.get_m00());

          if (showDetection) {
            ellipse(x, y, 10, 10);

            int[] depthValues = kinect.depthMap();
            int Position = x + (y * 640);
            int Zs = depthValues[Position];
            //println(x, y, Zs);
            Xs = x;
            Ys = y;
          }
        }
      }
    }
  }

  public boolean frame_Has_Shape() {
    return foundShape;
  }

  private boolean checkShape_detectedangle(MatOfPoint c) {

    boolean Shape_detected = false;
    MatOfPoint2f approx = new MatOfPoint2f();
    MatOfPoint2f mMOP2f = new MatOfPoint2f(); 
    MatOfPoint   mMOP   = new MatOfPoint();

    c.convertTo(mMOP2f, CvType.CV_32FC2);
    Imgproc.approxPolyDP(mMOP2f, approx, Imgproc.arcLength(mMOP2f, true)*0.03, true);
    approx.convertTo(mMOP, CvType.CV_32S);



    if ( approx.rows()==3 && Imgproc.isContourConvex(mMOP) && Math.abs(Imgproc.contourArea(approx)) > 700) 
    { 
      double maxcosine = 0;
      Point[] list = approx.toArray();
      textSize(32);
      fill(#ffff00);
      text("Triangle", Xs-35, Ys+75);
      if ( maxcosine < 0.3 ) {
        Shape_detected = true;
      }
    } else if ( approx.rows()==4 && Imgproc.isContourConvex(mMOP) && Math.abs(Imgproc.contourArea(approx)) > 700) 
    { 
      double maxcosine = 0;
      Point[] list = approx.toArray();
      textSize(32);
      fill(#ffff00);
      text("Rectangle", Xs-35, Ys+75);
      if ( maxcosine < 0.3 ) {
        Shape_detected = true;
      }
    } else if ( approx.rows()>6 && Imgproc.isContourConvex(mMOP) && Math.abs(Imgproc.contourArea(approx)) > 200) 
    { 
      double maxcosine = 0;
      Point[] list = approx.toArray();
      textSize(32);
      fill(#ffff00);
      text("Circle", Xs-35, Ys+75);
      if ( maxcosine < 0.3 ) {
        Shape_detected = true;
      }
    }

    return Shape_detected;
  }
}
