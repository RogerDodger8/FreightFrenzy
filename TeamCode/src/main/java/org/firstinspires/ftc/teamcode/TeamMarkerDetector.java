package org.firstinspires.ftc.teamcode;


import android.graphics.Bitmap;
import android.graphics.Color;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.*;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.android.dx.util.Warning;
import org.firstinspires.ftc.teamcode.Constants;

/**
 * Detector class that uses the Vuforia engine to grab a bitmap from the phone camera and detect the location of the Team Marker.
 */
public class TeamMarkerDetector {
  private static final String VUFORIA_KEY = "AVIcXQz/////AAABmaxHh/pXc0uQkZHc0fvBXE1sRSGKkYe3KVscLUJr947+r2DwwmaO3NNtiboGAHrnclLNTNiMnl4tvIqvnMR2gh5ha/jYUbI9FrEFcyZcshOUv4TE06wk75BGd/3/66u5d1S5CLtCJ292FHX2Xq8wa54pXddipzgGKobL0FtsaH4BCrEv4g4wQ3sPZnJ6MwU1UWJ5Ti/msZ4SAh7Q+1zJ5Cb74pohbpdPKe8RpMSUBNIaa5Q2nIzK2Xo7BYQ5/m33NcHaeyVMbHPt01/rRbNomEFwudMcBihucBT1scIk7wIIqx6NfIKdzW+lZH4G0uo8Aiaqr1zVWOg+o1YfkCUIhMHqiB8mjWFZjS6SCMEutR7j";

  private VuforiaLocalizer vuforiaLocalizer;

  private static final int Y_COORD = 955;

  private int stoneRightX, stoneCenterX, stoneLeftX;

  private static final int STONE_WIDTH = 340, STONE_HEIGHT = 460;

  private static final ColorPreset ACTIVE_YELLOW = ColorPreset.PURE_YELLOW;
  private static final ColorPreset ACTIVE_BLACK = ColorPreset.PURE_GRAY;

  private enum ColorPreset {
    PURE_YELLOW(255, 255, 0),
    PURE_GRAY(128, 123, 125);

    int r, g, b;

    ColorPreset(int r, int g, int b) {
      this.r = r;
      this.g = g;
      this.b = b;
    }
  }

  public TeamMarkerDetector(int cameraMonitorViewId) {
    initVuforia(cameraMonitorViewId);
  }

  /**
   * Runs the image processing code.
   *
   * @return The location of the team marker; LEFT, CENTER, or RIGHT
   * @param blueSide
   * @param imageSavingEnabled
   */
  public Constants.SamplingLocation sample(boolean blueSide, boolean imageSavingEnabled) {
    if (blueSide) {
      stoneRightX = 440;
      stoneCenterX = 285;
      stoneLeftX = 75;
    } else {
      stoneRightX = 2590;
      stoneCenterX = 1520;
      stoneLeftX = 415;
    }
//look up to location
    // Note 5X front camera takes picture 3264x2448 pixel wide
    Bitmap vuBitmap = getBitmap();
    if (imageSavingEnabled) {
      FileUtils.saveImage(vuBitmap, null);
    }

    try {
      Thread.sleep(1000);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }


    Bitmap stoneRight = Bitmap.createBitmap(vuBitmap, stoneRightX, Y_COORD, STONE_WIDTH, STONE_HEIGHT);
    Bitmap stoneCenter = Bitmap.createBitmap(vuBitmap, stoneCenterX, Y_COORD, STONE_WIDTH, STONE_HEIGHT);
    Bitmap stoneLeft = Bitmap.createBitmap(vuBitmap, stoneLeftX, Y_COORD, STONE_WIDTH, STONE_HEIGHT);

    if (imageSavingEnabled) {
      FileUtils.saveImage(stoneRight, Constants.SamplingLocation.RIGHT);
      FileUtils.saveImage(stoneCenter, Constants.SamplingLocation.CENTER);
      FileUtils.saveImage(stoneLeft, Constants.SamplingLocation.LEFT);
    }

    // Ratio is measured blackness to yellowness. higher ratio is more likeliness to be a team marker.
    double ratioRight = getClosenessToColor(stoneRight, ACTIVE_BLACK) / getClosenessToColor(stoneRight, ACTIVE_YELLOW);
    double ratioCenter = getClosenessToColor(stoneCenter, ACTIVE_BLACK) / getClosenessToColor(stoneCenter, ACTIVE_YELLOW);
    double ratioLeft = getClosenessToColor(stoneLeft, ACTIVE_BLACK) / getClosenessToColor(stoneLeft, ACTIVE_YELLOW);

    if (ratioRight > ratioCenter && ratioRight > ratioLeft) {
      return Constants.SamplingLocation.RIGHT;
    } else if (ratioCenter > ratioRight && ratioCenter > ratioLeft) {
      return Constants.SamplingLocation.CENTER;
    } else {
      return Constants.SamplingLocation.LEFT;
    }
  }

  /**
   * Grabs a bitmap from the Vuforia engine for image processing
   * @return camera output as a bitmap
   */
  public Bitmap getBitmap(){
    Bitmap bitmap;
    VuforiaLocalizer.CloseableFrame frame;

    try {
      frame = vuforiaLocalizer.getFrameQueue().take();
    }catch (Exception e) {
      throw new Warning("couldn't find vuforia frame");
    }

    bitmap = vuforiaLocalizer.convertFrameToBitmap(frame);
    return bitmap;
  }

  /**
   * @return the closeness of a region to a color
   */
  private double getClosenessToColor(Bitmap bitmap, ColorPreset colorPreset) {
    int color;
    int r, g, b;
    double distanceSum = 0;

    int pixels = bitmap.getWidth() * bitmap.getHeight();
    for (int i = 0; i < bitmap.getWidth(); i++) {
      for (int j = 0; j < bitmap.getHeight(); j++) {
        color = bitmap.getPixel(i, j);
        r = Color.red(color);
        g = Color.green(color);
        b = Color.blue(color);
        distanceSum += getColorDistance(r, g, b, colorPreset.r, colorPreset.g, colorPreset.b);
      }
    }

    double averageDistance = distanceSum / pixels;

    if (averageDistance != 0) {
      return 1 / averageDistance;
    } else {
      return Double.POSITIVE_INFINITY;
    }
  }

  /**
   * Gets the distance between two colors via 3 dimensional distance formula.
   *
   * @param r       Actual red value
   * @param g       Actual green value
   * @param b       Actual blue value
   * @param targetR Target red value
   * @param targetG Target green value
   * @param targetB Target blue value
   * @return Distance between actual color and target color
   */
  private double getColorDistance(int r, int g, int b, int targetR, int targetG, int targetB) { // does the actual Maths
    int rDifference = r - targetR;
    int gDifference = g - targetG;
    int bDifference = b - targetB;

    int rDifferenceSquared = (int) Math.pow(rDifference, 2);
    int gDifferenceSquared = (int) Math.pow(gDifference, 2);
    int bDifferenceSquared = (int) Math.pow(bDifference, 2);

    int sum = rDifferenceSquared + gDifferenceSquared + bDifferenceSquared;

    return Math.sqrt(sum);
  }

  private void initVuforia(int cameraMonitorViewId) {
    VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
    params.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
    params.vuforiaLicenseKey = VUFORIA_KEY;

    this.vuforiaLocalizer = ClassFactory.getInstance().createVuforia(params);
    vuforiaLocalizer.enableConvertFrameToBitmap();
    vuforiaLocalizer.setFrameQueueCapacity(1);
  }
}
