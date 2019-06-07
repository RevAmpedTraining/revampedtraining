package com.revAmped.util;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.util.RobotLog;
import com.revAmped.components.RobotRevAmped;
import com.revAmped.sensors.RevColorDistanceSensor;

import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Victo on 11/5/2017.
 * Added new method and algrithm by wang.  3/26/2018
 */

public class SampleOrderDetector {
    public enum GoldenOrder {
        LEFT,
        RIGHT,
        CENTER,
        UNKNOWN
    }

    public double              perfectArea      = 6500;

    private GoldenOrder currentOrder = GoldenOrder.UNKNOWN;
    private GoldenOrder lastOrder    = GoldenOrder.UNKNOWN;

    private Mat hiarchy  = new Mat();
    List<MatOfPoint> hexgons = new ArrayList<>();

    //constants that defined to check the contours.
    int YELLOW_THRESH = 100;// color filtering-changed to 100 3/06/2019 to account for dim lighting

    double RATIO_GOLD = 0.4;                 // threahold for  contour area / min circle area
    double RATIO_BALL = 0.5;
    double AREA_BALL_MIN = 5000;       // area of the ball contour: about 40K
    double AREA_BALL_MAX = 80000;

    // all units are pixels.
    double R_BALL_MIN = 30;            // JEWEL radius:  about 140
    double R_BALL_MAX = 90;
    int Y_SAMPLE_MAX  = 480;            // MAX Y of the Mineral.  If it is on the hook , or on the ground the Y MAX can be different.
    int X_LEFT_MAX = 400;               // the JEWEL is located at x<400, y < 300.  while x should > 50, and y > 50
    int X_CENTER_MAX = 900;

    int NEIGHBOR_LIMIT = 250;
    int XLIMIT = 20;   // pixeloffset in X and Y to be a new match.
    int YLIMIT = 20;

    int x_yellow;
    int y_yellow;

    List<Integer> x_whiteballs = new ArrayList<>();
    List<Integer> y_whiteballs = new ArrayList<>();
    List<Integer> x_yellows = new ArrayList<>();
    List<Integer> y_yellows = new ArrayList<>();

    Mat img = null;

    boolean goldenFound = false;
    public double downscale = 0.8;    // image with front camera 1280x720.  scale down Y-axis,  from 720 -> 720*.65,  keeping top portion of the image.
    private Size newSize = new Size();
    private Size initSize = new Size();
    public float getArea(float r)
    {
        return (float) Math.PI * r * r;
    }

    public GoldenOrder GetCurrentOrder()
    {
        return currentOrder;
    }
    //Using OpenCV to detect the color.  this code still need to be tuned.
    public Boolean updateSampleOrder(Bitmap bitmap)
    {
        try
        {
            long t_start = System.currentTimeMillis();

            if(img ==null) {
                img = new Mat(bitmap.getHeight(), bitmap.getWidth(), CvType.CV_8UC4);
            }
            Utils.bitmapToMat(bitmap, img);
            currentOrder = GetSampleOrder(img);
            RobotLog.vv("UpdateSampleOrder", "%d,%s", (System.currentTimeMillis() - t_start), currentOrder);
            return true;
        }
        catch (Exception e)
        {
            RobotLog.e("Vuforia", e.getMessage());
        }
        return false;
    }
    public GoldenOrder GetSampleOrder(Mat origimg) {
        RobotLog.vv("GetSampleOrder", "Entering");
        long t_start = System.currentTimeMillis();
        //================ Initialize variables ===============================
        Mat mask = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();
        List<Mat> channels = new ArrayList<>();
        goldenFound = false;

        try {
            //===========================resize it. remove anything in crater or corners.===========================
            initSize.width = origimg.width();  initSize.height = origimg.height();
            newSize.width = initSize.width; newSize.height = initSize.height * downscale;
            Rect rect_Crop = new Rect(0, 0, (int) initSize.width, (int) (initSize.height*downscale));

            Mat image_roi = new Mat(origimg,rect_Crop);

            //=================================================== FIND GOLDEN COLOR ====================================================

            // Use YUV color space
            Mat imgYUV = image_roi.clone();
            Imgproc.cvtColor(image_roi, imgYUV, Imgproc.COLOR_RGB2YUV);
            Imgproc.GaussianBlur(imgYUV, imgYUV, new Size(5, 5), 0);
            Core.split(imgYUV, channels);

            // get the Yellow color space and threshold.  ONLY yellow color objects stay. others are filtered out.
            Imgproc.threshold(channels.get(1), mask, YELLOW_THRESH, 255, Imgproc.THRESH_BINARY);
            //new add 11/16

            Mat kernel = new Mat(new Size(3, 3), CvType.CV_8UC1, new Scalar(255));
            Mat maskOpenFill = new Mat();
//# remove small dots and fill the gaps.
            Imgproc.morphologyEx(mask, maskOpenFill, Imgproc.MORPH_OPEN,kernel );
            Imgproc.findContours(maskOpenFill, contours, hiarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            //new add 11/16
            //Imgproc.findContours(mask, contours, hiarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            //check each coutours:  if it matches the criteria,  contour area / circle > RATIO,  MIN < radius < MAX,  X> X_MIN,  Y>Y_MIN
            //RobotLog.vv("BLUEContours", "%d", contours.size());
            for (MatOfPoint c : contours) {
                double area = Imgproc.contourArea(c);
                if (area > AREA_BALL_MIN && area < AREA_BALL_MAX) {
                    float[] radius = new float[1];
                    Point center = new Point();
                    Imgproc.minEnclosingCircle(new MatOfPoint2f(c.toArray()), center, radius);
                    float ratio = (float) area / getArea(radius[0]);
                    RobotLog.vv("CHECK YELLOW", "area=%4.1f,x=%d,y=%d,r=%d,ratio=%3.2f", area, (int) center.x, (int) center.y, (int) radius[0], ratio);

                    if (ratio > RATIO_GOLD &&
                           //        radius[0] > R_BALL_MIN &&
                           //        radius[0] < R_BALL_MAX &&
                                    center.y < Y_SAMPLE_MAX )
                    {
                        goldenFound = true;
                        if (isNewPoint((int)center.x, (int)center.y, x_yellows, y_yellows)) {
                            if (Math.abs(center.x - X_LEFT_MAX) < NEIGHBOR_LIMIT) {
                                currentOrder = GoldenOrder.CENTER;
                            } else if (Math.abs(center.x - X_CENTER_MAX) < NEIGHBOR_LIMIT) {
                                currentOrder = GoldenOrder.LEFT;
                            } /*else  {
                                currentOrder = GoldenOrder.RIGHT;
                            }*/

                            x_yellow = (int) center.x;
                            y_yellow = (int) center.y;
                            RobotLog.vv("YELLOW", "SAMPLE.area=%4.1f,x=%d,y=%d,r=%d,ratio=%3.2f", area, (int) center.x, (int) center.y, (int) radius[0], ratio);
                        }
                        //break;
                    }
                }
            }
            imgYUV.release();

            long t_end = System.currentTimeMillis();
            RobotLog.vv("TimeForYELLOW", "%d", (t_end - t_start));

            t_start = t_end;
            //=========================================Check White ======================================================
            /*if (!goldenFound) {
                RobotLog.vv("SAMPLE", "Checking WHITE");
                //new add 11/16
                Mat maskCloseFill = new Mat();
kernel = new Mat(new Size(3, 3), CvType.CV_8UC1, new Scalar(255));
//# remove small dots and fill the gaps.
//Imgproc.morphologyEx(whitemask, maskOpenFill, Imgproc.MORPH_OPEN,kernel );
Imgproc.morphologyEx(whitemask, maskCloseFill, Imgproc.MORPH_CLOSE,kernel );
Imgproc.morphologyEx(whitemask, maskOpenFill, Imgproc.MORPH_OPEN,kernel );

Imgproc.findContours(maskCloseFill, contours, hiarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
//new add 11/16
                Mat input = origimg.clone();
                Imgproc.cvtColor(input,input,Imgproc.COLOR_RGB2HSV_FULL);

                // Blur it
                Imgproc.GaussianBlur(input,input,new Size(5,5),0);
                Mat whitemask = new Mat();
                // Run in range check
                Core.inRange(input,new Scalar(0, 0, 200),new Scalar(50, 40, 255),whitemask);
                input.release();
                //Mat maskOpenFill = new Mat();
                Mat maskCloseFill = new Mat();
                Mat kernel = new Mat(new Size(3, 3), CvType.CV_8UC1, new Scalar(255));
                //# remove small dots and fill the gaps.
                //Imgproc.morphologyEx(whitemask, maskOpenFill, Imgproc.MORPH_OPEN,kernel );
                Imgproc.morphologyEx(whitemask, maskCloseFill, Imgproc.MORPH_CLOSE,kernel );

                Imgproc.findContours(maskCloseFill, contours, hiarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
                //maskOpenFill.release();
                maskCloseFill.release();

                //check each coutours:  if it matches the criteria,  contour area / circle > RATIO,  MIN < radius < MAX,  X> X_MIN,  Y>Y_MIN
                //RobotLog.vv("REDContours", "%d", contours.size());
                int countWhite = 0;
                for (MatOfPoint c : contours) {
                    double area = Imgproc.contourArea(c);
                    if (area > AREA_BALL_MIN && area < AREA_BALL_MAX) {
                        float[] radius = new float[1];
                        Point center = new Point();
                        Imgproc.minEnclosingCircle(new MatOfPoint2f(c.toArray()), center, radius);
                        float ratio = (float) area / getArea(radius[0]);
                        RobotLog.vv("WHITE", "area=%4.1f,x=%d,y=%d,r=%d,ratio=%3.2f", area, (int) center.x, (int) center.y, (int) radius[0], ratio);

                        if ( ratio > RATIO_BALL && radius[0] > R_BALL_MIN && radius[0] < R_BALL_MAX && center.y < Y_SAMPLE_MAX)
                        {
                            if (isNewPoint((int) center.x, (int) center.y, x_whiteballs, y_whiteballs)) {
                                x_whiteballs.add((int) center.x);
                                y_whiteballs.add((int) center.y);
                                countWhite++;
                                RobotLog.vv("WHITE", "area=%4.1f,x=%d,y=%d,r=%d,ratio=%3.2f", area, (int) center.x, (int) center.y, (int) radius[0], ratio);
                            }
                        }
                    }
                }
                if(countWhite>=1) {
                    currentOrder = GoldenOrder.RIGHT;
                }
                RobotLog.vv("WHITE", "count = %d", x_whiteballs.size());
                whitemask.release();

                t_end = System.currentTimeMillis();
                RobotLog.vv("TimeForWhite", "%d", (t_end - t_start));
            }*/
        } catch (Exception e) {
            RobotLog.e("ErrorForJewel", e.getMessage());
        }
        finally {
            //===============================================cleanup===================================================
            for (int i = 0; i < channels.size(); i++) {
                channels.get(i).release();
            }
            //hiarchy.release();
            contours.clear();
            mask.release();
        }
         return currentOrder;
    }

    /**
     *
     * Get the hexgon patterns to detect Key Column.   this needs to be called after Jewel color call to get the center of Jewel ocordinate.
     *
     */

    private Boolean isNewPoint(int x0, int y0, List<Integer> xarray, List<Integer> yarray)

    {
        if (xarray.size() == 0)
            return true;

        for (int x : xarray)
        {
            if (Math.abs(x - x0) < XLIMIT) {
                for (int y : yarray)
                {
                    if (Math.abs(y - y0) < YLIMIT)
                        return false;
                }
            }
        }

        return true;
    }


    public  GoldenOrder getCurrentOrder() {
        return currentOrder;
    }

    public  GoldenOrder getLastOrder() {
        return lastOrder;
    }
}
