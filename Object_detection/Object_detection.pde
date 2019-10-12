import SimpleOpenNI.*;
import java.awt.Color;
import java.util.ArrayList;
import gab.opencv.*;
import hypermedia.net.*;
import ddf.minim.*;
import processing.serial.*;
import cc.arduino.*;


SimpleOpenNI kinect;
OpenCV opencv;
PVector[] markers;
Shape_Detector ShapeDetector;
UDP udp;
Minim minim;
AudioSnippet AiDream;
Serial nano;
Arduino arduino;

int w = 640, h = 480;
int Xc, Yc;
int z;
String cmd = " ";
float X_axis;
float Y_axis;
float Z_axis;
float hmin, hmax, smin, smax, bmin, bmax;
color bgColour = color(0, 0, 0);
color fgColour = color(255, 255, 255);
int minBlobSize = 200;
boolean debug = false;
boolean pickup = false;
boolean place = false;
boolean walk = false;
boolean object = false;
int sensor;
char cmnd;
int grip = 0;


void setup() {
  size(1300, 460);
  strokeWeight(2);
  opencv = new OpenCV(this, 640, 480);
  kinect = new SimpleOpenNI(this);
  kinect.enableDepth();
  kinect.enableRGB();
  kinect.setMirror(true);
  ShapeDetector = new Shape_Detector(this, kinect.rgbWidth(), kinect.rgbHeight());
  udp = new UDP( this, 5003 );
  udp.listen( true );
  minim = new Minim(this);
  println(Arduino.list());
  arduino = new Arduino(this, Arduino.list()[32], 57600);
  nano = new Serial(this, Serial.list()[33], 9600);

  arduino.pinMode(0, Arduino.INPUT);
  arduino.pinMode(1, Arduino.INPUT);
  arduino.pinMode(2, Arduino.INPUT);
  arduino.pinMode(3, Arduino.INPUT);

  arduino.pinMode(2, Arduino.SERVO);
  arduino.pinMode(3, Arduino.SERVO);
  arduino.pinMode(4, Arduino.SERVO);
  arduino.pinMode(5, Arduino.SERVO);
  arduino.pinMode(6, Arduino.SERVO);
  arduino.pinMode(7, Arduino.SERVO);
  arduino.pinMode(8, Arduino.SERVO);
  arduino.pinMode(9, Arduino.SERVO);

  arduino.pinMode(50, Arduino.OUTPUT);
  arduino.pinMode(51, Arduino.OUTPUT);
  arduino.pinMode(52, Arduino.OUTPUT);
  arduino.pinMode(53, Arduino.OUTPUT);
}

void draw() {
  arduino.servoWrite(2, 15);
  arduino.servoWrite(6, 50);
  int grip = arduino.analogRead(0);

  kinect.update();
  image(kinect.rgbImage(), 641, 0);

  int[] userList = kinect.getUsers();
  opencv.loadImage(kinect.rgbImage());
  kinect.rgbImage().loadPixels();
  image(kinect.rgbImage(), 0, 0);

  for (int i = 0; i < kinect.rgbImage ().pixels.length; i += 1) {
    color c = kinect.rgbImage().pixels[i];
    float[] hsb = Color.RGBtoHSB((int) red(c), (int) green(c), (int) blue(c), null);
    if (!hsbConstrained(hsb)) {
      kinect.rgbImage().pixels[i] = fgColour;
    } else {
      kinect.rgbImage().pixels[i] = bgColour;
    }
  }

  kinect.rgbImage().updatePixels();
  smooth();
  kinect.rgbImage().filter(ERODE);
  kinect.rgbImage().filter(ERODE);
  kinect.rgbImage().filter(ERODE);
  kinect.rgbImage().filter(ERODE);
  kinect.rgbImage().filter(ERODE);
  kinect.rgbImage().filter(DILATE);
  kinect.rgbImage().filter(DILATE);
  kinect.rgbImage().filter(DILATE);
  kinect.rgbImage().filter(DILATE);
  kinect.rgbImage().filter(DILATE);

  grip = arduino.analogRead(0);
  if (object) {
    if (!pickup) {
      arduino.servoWrite(6, 0);
      grip = arduino.analogRead(0);
      delay(500);
      grip = arduino.analogRead(0);
      arduino.servoWrite(6, 60);
      grip = arduino.analogRead(0);
      if (grip > 500) {
        println("done");
        pickup = true;
      }
    }
  }
  if (pickup) {
    arduino.servoWrite(6, 0);
    pickup = false;
    place = true;
  }
  if (place) {
    arduino.servoWrite(2, 80);
    delay(1000);
    arduino.servoWrite(8, 60);
    delay(1000);
    arduino.servoWrite(9, 102);
    delay(1000);
    arduino.servoWrite(6, 50);
    place = false;
    delay(1000);

    arduino.servoWrite(2, 15);
    delay(1000);
    arduino.servoWrite(8, 90);
    delay(1000);
    arduino.servoWrite(9, 155);
  }
  //*************** Ultrasonic Sensors *****************************
  if (walk) {
    if (nano.available() > 0) { 
      sensor = nano.read();
      cmnd = char(sensor); 
      println(cmnd);
      if (cmnd == 'F') {
        forward();
      } else if (cmnd == 'B') {
        brake();
        delay(500);
        backward();
        delay(2000);
      } else if (cmnd == 'L') {
        brake();
        delay(500);
        backward();
        delay(2000);
        left();
        delay(2000);
      } else if (cmnd == 'R') {
        brake();
        delay(500);
        backward();
        delay(2000);
        right();
        delay(2000);
      }
    }
  }
  //****************************************************************
  if (line) {
    int one = arduino.analogRead(3);
    int two = arduino.analogRead(1);

    if (one<500) {
      right();
      delay(200);
    } else if (two<500) {
      left();
      delay(200);
    } else if (one >500 && two >500) {
      forward();
    }
  }
  //****************************************************************
  int[] labels = new int[kinect.rgbImage().pixels.length];
  int l = 1; // Label number

  for (int i = 0; i < kinect.rgbImage ().pixels.length; i += 1) {
    l = labelPixelsReturnL(labels, kinect.rgbImage().pixels, i, l);
  }

  int[] sizes = new int[l];
  for (int i = 0; i < kinect.rgbImage ().pixels.length; i += 1) {
    if (labels[i] > 0) {
      sizes[labels[i] - 1] += 1;
    }
  }

  ArrayList labelsToShow = new ArrayList();
  for (int i = 0; i < sizes.length; i++) {
    if (sizes[i] > minBlobSize) {
      labelsToShow.add(i + 1);
    }
  }
  draw_centroids(labels, w, labelsToShow);

  ShapeDetector.processFrame(kinect.rgbImage(), true);
}

boolean hsbConstrained(float[] hsb) {
  boolean constrained = false;
  constrained = constrained || constrained(hsb[0], hmin, hmax);
  constrained = constrained || constrained(hsb[1], smin, smax);
  constrained = constrained || constrained(hsb[2], bmin, bmax);
  return constrained;
}

boolean constrained(float x, float min, float max) {
  if (constrain(x, min, max) == x) {
    return false;
  } else {
    return true;
  }
}

void draw_centroid(int x, int y) {

  int[] depthValues = kinect.depthMap();
  int Position = x + (y * 640);
  z = depthValues[Position];
  int lineLength = 30;
  Xc = x;
  Yc = y;
  X_axis = map(Xs/3, 50, 400, 60, 180);
  Y_axis = map(Ys/3, 50, 400, 0, 160);
  Z_axis = map(Zs, 495, 1500, 0, 1000);
  int X_axs = int(X_axis+3);
  int Y_axs = int(Y_axis);
  int Z_axs = int(Z_axis);

  arduino.servoWrite(8, abs(X_axs));
  arduino.servoWrite(9, abs(Y_axs));

  if (x >= 0 && y >= 0 && z <= 500) {
    stroke(255, 255, 255);
    fill(17, 171, 245);
    ellipse(x, y, lineLength / 2, lineLength / 2);
    line(x - lineLength / 2, y, x + lineLength / 2, y);
    line(x, y - lineLength / 2, x, y + lineLength / 2);
  }
}

void forward() {
  arduino.digitalWrite(50, Arduino.HIGH);
  arduino.digitalWrite(51, Arduino.LOW);
  arduino.digitalWrite(53, Arduino.HIGH);
  arduino.digitalWrite(52, Arduino.LOW);
}
void backward() {
  arduino.digitalWrite(50, Arduino.LOW);
  arduino.digitalWrite(51, Arduino.HIGH);
  arduino.digitalWrite(53, Arduino.LOW);
  arduino.digitalWrite(52, Arduino.HIGH);
}
void left() {
  arduino.digitalWrite(50, Arduino.HIGH);
  arduino.digitalWrite(51, Arduino.LOW);
  arduino.digitalWrite(53, Arduino.LOW);
  arduino.digitalWrite(52, Arduino.HIGH);
}
void right() {
  arduino.digitalWrite(50, Arduino.LOW);
  arduino.digitalWrite(51, Arduino.HIGH);
  arduino.digitalWrite(53, Arduino.HIGH);
  arduino.digitalWrite(52, Arduino.LOW);
}
void brake() {
  arduino.digitalWrite(50, Arduino.LOW);
  arduino.digitalWrite(51, Arduino.LOW);
  arduino.digitalWrite(53, Arduino.LOW);
  arduino.digitalWrite(52, Arduino.LOW);
}

void receive( byte[] data ) {
  String rcv = new String(data);
  //println(rcv);

  String Q1 = "what's your name";
  String Q2 = "how can I control you";
  String Q3 = "what's your favourite food";
  String Q4 = "tell me about yourself";
  String Q5 = "how are you";
  String Q6 = "I am doing well";
  String Q7 = "how many sensor do you have";
  String Q8 = "that is impressive";
  String Q9 = "tell me about your vision";
  String Q10 = "detect blue colour";
  String Q11 = "detect yellow colour";
  String Q12 = "can I handshake with you";
  String Q13 = "move autonomously";
  String Q14 = "can you track me";
  String Q15 = "place";

  if (rcv.equals(Q1)) {
    AiDream = minim.loadSnippet("sound of AiDream/1.mp3");
    AiDream.play();
  } else if (rcv.equals(Q2)) {
    AiDream = minim.loadSnippet("sound of AiDream/2.mp3");
    AiDream.play();
  } else if (rcv.equals(Q3)) {
    AiDream = minim.loadSnippet("sound of AiDream/3.mp3");
    AiDream.play();
  } else if (rcv.equals(Q4)) {
    AiDream = minim.loadSnippet("sound of AiDream/4.mp3");
    AiDream.play();
  } else if (rcv.equals(Q5)) {
    AiDream = minim.loadSnippet("sound of AiDream/5.mp3");
    AiDream.play();
  } else if (rcv.equals(Q6)) {
    AiDream = minim.loadSnippet("sound of AiDream/6.mp3");
    AiDream.play();
  } else if (rcv.equals(Q7)) {
    AiDream = minim.loadSnippet("sound of AiDream/7.mp3");
    AiDream.play();
  } else if (rcv.equals(Q8)) {
    AiDream = minim.loadSnippet("sound of AiDream/9.mp3");
    AiDream.play();
  } else if (rcv.equals(Q9)) {
    AiDream = minim.loadSnippet("sound of AiDream/8.mp3");
    AiDream.play();
  } else if (rcv.equals(Q10)) {
    hmin = 0.20; 
    hmax = 0.7; 
    smin = 0.50; 
    smax = 1.00; 
    bmin = 0.40; 
    bmax = 1.00; // Blue
    object = true;
    //AiDream = minim.loadSnippet("sound of AiDream/10.mp3");
    //AiDream.play();
  } else if (rcv.equals(Q11)) {
    hmin = 0.00; 
    hmax = 0.21; 
    smin = 0.50; 
    smax = 1.00; 
    bmin = 0.10; 
    bmax = 1.00; // Yellow
    object = true;
  } else if (rcv.equals(Q12)) {
    cmd = "handshake";
  } else if (rcv.equals(Q13)) {
    walk = true;
  } else if (rcv.equals(Q14)) {
    cmd = "face";
  } else if (rcv.equals(Q15)) {
    cmd = "place";
  } else {
    cmd = " ";
  }
}
