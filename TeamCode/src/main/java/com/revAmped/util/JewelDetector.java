package com.revAmped.util;

import com.qualcomm.robotcore.util.RobotLog;
import com.revAmped.sensors.RevColorDistanceSensor;
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

import static java.lang.System.in;

/**
 * Created by Victo on 11/5/2017.
 * Added new method and algrithm by wang.  3/26/2018
 */

public class JewelDetector {

    public enum JewelOrder {
        RED_BLUE,
        BLUE_RED,
        UNKNOWN
    }

    public enum JewelDetectionMode {
        PERFECT_AREA, MAX_AREA
    }

    public enum JewelDetectionSpeed {
        VERY_FAST, FAST, BALANCED, SLOW, VERY_SLOW
    }


    public JewelDetectionMode  detectionMode    = JewelDetectionMode.MAX_AREA;
    public double              downScaleFactor  = 0.4;
    public double              perfectRatio     = 1;
    public boolean             rotateMat        = false;
    public JewelDetectionSpeed speed            = JewelDetectionSpeed.BALANCED;
    public double              perfectArea      = 6500;
    public double              areaWeight       = 0.05; // Since we're dealing with 100's of pixels
    public double              minArea          = 700;
    public double              ratioWeight      = 15; // Since most of the time the area diffrence is a decimal place
    public double              maxDiffrence     = 10; // Since most of the time the area diffrence is a decimal place
    public boolean             debugContours    = false;
    public LeviColorFilter   colorFilterRed   = new LeviColorFilter(LeviColorFilter.ColorPreset.RED);
    public LeviColorFilter   colorFilterBlue  = new LeviColorFilter(LeviColorFilter.ColorPreset.BLUE);


    private JewelOrder currentOrder = JewelOrder.UNKNOWN;
    private JewelOrder lastOrder    = JewelOrder.UNKNOWN;


    private Mat workingMat = new Mat();
    private Mat blurredMat  = new Mat();
    private Mat maskRed  = new Mat();
    private Mat maskBlue  = new Mat();
    private Mat hiarchy  = new Mat();
    List<MatOfPoint> hexgons = new ArrayList<>();

    //constants that defined to check the contours.
    int THRESH_BINARY = 30;
    int BLUE_THRESH = 145;
    int RED_THRESH = 164;               // color filtering

    int X_BALL = 0;                     // X, Y coordinate of JEWEL BALL.  Based on the X and Y, we will crop the image to get the HEXGON
    int Y_BALL = 0;
    double AREA_HEXGON_MIN = 400;       //for a 1280x720 frame, the hexgon area is about 500.
    double AREA_HEXGON_MAX = 900;
    double R_HEXGON_MIN = 10;            // hexgon radius:  about 14
    double R_HEXGON_MAX = 20;
    double RATIO = 0.5;                 // threahold for  contour area / min circle area
    double AREA_BALL_MIN = 20000;       // area of the ball contour: about 40K
    double AREA_BALL_MAX = 90000;
    //double AREA_BALL_MAX = 80000;

    // all units are pixels.
    double R_BALL_MIN = 100;            // JEWEL radius:  about 140
    double R_BALL_MAX = 300;
    //double R_BALL_MAX = 200;
    int X_HEXGON_MIN = 400;             // all hexgon should be at x>300 and y> 200 area.  no max is defined. it can be.
    int Y_HEXGON_MIN = 200;
    int X_BALL_MAX = 400;               // the JEWEL is located at x<400, y < 300.  while x should > 50, and y > 50
    int X_BALL_MIN = 150;
    int Y_BALL_MAX = 300;
    int X_OFFSET_HEXGONS = 150;         // from the center.x of jewel to the right side of hexgon area
    int Y_OFFSET_HEXGONS = 120;         // from the center.y of jewel to the down side of hexgon area
    int RECT_HEXGONS_W = 450;           //  hexgon area width
    int RECT_HEXGONS_H = 180;           // hexgon area height.

    float THRESH_TEMPLATE_MATCH = (float) 0.7;
    int XLIMIT = 10;   // pixeloffset in X and Y to be a new match.
    int YLIMIT = 10;

    RevColorDistanceSensor.COLORTYPE jColor = RevColorDistanceSensor.COLORTYPE.NONE;

    private Size newSize = new Size();

    public float getArea(float r)
    {
        return (float) Math.PI * r * r;
    }

    //Using OpenCV to detect the color.  this code still need to be tuned.
    public RevColorDistanceSensor.COLORTYPE GetJewelColor(Mat origimg) {
        RobotLog.vv("GetJewelColor", "Entering");
        long t_start = System.currentTimeMillis();
        Mat mask = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();
        List<Mat> channels = new ArrayList<>();

        try {
            //=================================================== FIND BALL COLOR ====================================================
            //initialize the value,  false
            Boolean jewelfound = false;
            RobotLog.vv("JEWEL", "Checking Blue");
            //Check Blue
            Mat imgYUV = origimg.clone();
            Imgproc.cvtColor(origimg, imgYUV, Imgproc.COLOR_RGB2YUV);
            Imgproc.GaussianBlur(imgYUV, imgYUV, new Size(5, 5), 0);
            Core.split(imgYUV, channels);

            mask = new Mat();
            Imgproc.threshold(channels.get(1), mask, BLUE_THRESH, 255, Imgproc.THRESH_BINARY);
            Imgproc.findContours(mask, contours, hiarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            //check each coutours:  if it matches the criteria,  contour area / circle > RATIO,  MIN < radius < MAX,  X> X_MIN,  Y>Y_MIN
            for (MatOfPoint c : contours) {
                double area = Imgproc.contourArea(c);
                if (area > AREA_BALL_MIN && area < AREA_BALL_MAX) {
                    float[] radius = new float[1];
                    Point center = new Point();
                    Imgproc.minEnclosingCircle(new MatOfPoint2f(c.toArray()), center, radius);
                    float ratio = (float) area / getArea(radius[0]);
                    RobotLog.vv("BALL_BLUE", "JEWEL=BLUE.area=%4.1f,x=%d,y=%d,r=%d,ratio=%3.2f", area, (int) center.x, (int) center.y, (int) radius[0], ratio);

                    if (area / getArea(radius[0]) > RATIO && radius[0] > R_BALL_MIN && radius[0] < R_BALL_MAX && center.x > X_BALL_MIN && center.x < X_BALL_MAX && center.y < Y_BALL_MAX) {
                        jColor = RevColorDistanceSensor.COLORTYPE.BLUE;
                        jewelfound = true;
                        X_BALL = (int) center.x;
                        Y_BALL = (int) center.y;
                        RobotLog.vv("BALL_BLUE", "JEWEL=BLUE.area=%4.1f,x=%d,y=%d,r=%d,ratio=%3.2f", area, (int) center.x, (int) center.y, (int) radius[0], ratio);
                        break;
                    }
                    else
                    {
                        RobotLog.vv("BLUE", "NOT SEEN BLUE");
                    }
                }
            }
            imgYUV.release();
            mask.release();

            long t_end = System.currentTimeMillis();
            RobotLog.vv("TimeForBule", "%d", (t_end - t_start));
            t_start = t_end;
            //=========================================Check Red ======================================================
            if (!jewelfound) {
               // jColor = RevColorDistanceSensor.COLORTYPE.RED;
                RobotLog.vv("JEWEL", "Checking Red");
                Mat imgLAB = origimg.clone();
                Imgproc.cvtColor(origimg, imgLAB, Imgproc.COLOR_RGB2Lab);
                Imgproc.GaussianBlur(imgLAB, imgLAB, new Size(5, 5), 0);
                Core.split(imgLAB, channels);

                mask = new Mat();
                Imgproc.threshold(channels.get(1), mask, RED_THRESH, 255, Imgproc.THRESH_BINARY);
                Imgproc.findContours(mask, contours, hiarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
                //check each coutours:  if it matches the criteria,  contour area / circle > RATIO,  MIN < radius < MAX,  X> X_MIN,  Y>Y_MIN
                for (MatOfPoint c : contours) {
                    double area = Imgproc.contourArea(c);
                    if (area > AREA_BALL_MIN && area < AREA_BALL_MAX) {
                        float[] radius = new float[1];
                        Point center = new Point();
                        Imgproc.minEnclosingCircle(new MatOfPoint2f(c.toArray()), center, radius);
                        float ratio = (float) area / getArea(radius[0]);
                        RobotLog.vv("BALL_RED", "JEWEL=RED. area=%4.1f,x=%d,y=%d,r=%d,ratio=%3.2f", area, (int) center.x, (int) center.y, (int) radius[0], ratio);

                        if (area / getArea(radius[0]) > RATIO && radius[0] > R_BALL_MIN && radius[0] < R_BALL_MAX && center.x > X_BALL_MIN && center.x < X_BALL_MAX && center.y < Y_BALL_MAX) {
                            jColor = RevColorDistanceSensor.COLORTYPE.RED;
                            X_BALL = (int) center.x;
                            Y_BALL = (int) center.y;
                            RobotLog.vv("BALL_RED", "JEWEL=RED. area=%4.1f,x=%d,y=%d,r=%d,ratio=%3.2f", area, (int) center.x, (int) center.y, (int) radius[0], ratio);
                            break;
                        }
                        else
                        {
                            RobotLog.vv("RED", "NOT SEEN");
                        }
                    }
                }
                imgLAB.release();
                mask.release();
                t_end = System.currentTimeMillis();
                RobotLog.vv("TimeForRED", "%d", (t_end - t_start));
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
        finally {
            //===============================================cleanup===================================================
            for (int i = 0; i < channels.size(); i++) {
                channels.get(i).release();
            }
            hiarchy.release();
            contours.clear();
            mask.release();
        }
         return jColor;
    }

    /**
     *
     * Get the hexgon patterns to detect Key Column.   this needs to be called after Jewel color call to get the center of Jewel ocordinate.
     * @param origimg
     */
    public void getHexgonPattern(Mat origimg, Mat templ)
    {
        //==========================  Find patterns of hexgon to detect KEY COLUMN =======================================
        // convert pictograph to gray and use threshold to change to black and white, then contour it.
        long t_start = System.currentTimeMillis();
        Mat mask = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();
        List<Mat> channels = new ArrayList<>();

        try
        {
            Mat img_gray = new Mat();
            int x1 = X_BALL + X_OFFSET_HEXGONS;
            int y1 = Y_BALL + Y_OFFSET_HEXGONS;
            int w = RECT_HEXGONS_W;
            int h = RECT_HEXGONS_H;

            RobotLog.vv("HEXGON", "x=%d,y=%d,w=%d,h=%d", x1, y1, w, h);
            //crop out the area of hexgons:
            Rect rectCrop = new Rect(x1, y1 , w, h);
            Mat img2= origimg.submat(rectCrop);

            Imgproc.cvtColor(img2, img_gray, Imgproc.COLOR_RGBA2GRAY);

            //================================use template matching.===========================

            Mat result = new Mat();
            try {
                int result_cols = img_gray.cols() - templ.cols() + 1;
                int result_rows = img_gray.rows() - templ.rows() + 1;
                result.create(result_rows, result_cols, CvType.CV_32FC1);
                /// Do the Matching and Normalize
                int match_method = Imgproc.TM_CCOEFF_NORMED;   //.TM_SQDIFF;
                Imgproc.matchTemplate(img_gray, templ, result, match_method);
                Core.normalize(result, result, 0, 1, Core.NORM_MINMAX, -1, new Mat());
                //any matching score between 0, THRESH_TEMPLATE_MATCH, to  zero.
                Imgproc.threshold(result, result, THRESH_TEMPLATE_MATCH, 1, Imgproc.THRESH_TOZERO);
                RobotLog.vv("Template", "Complete MatchTemplate");
                // find all the locations whose match score is > Threshold.
                // Localizing the best match with minMaxLoc. We localize the minimum and maximum values in the result matrix R by using minMaxLoc.

                List<Integer> xarray = new ArrayList<Integer>();
                List<Integer> yarray = new ArrayList<Integer>();

                //List<Double> matchScores = new ArrayList<Double>();
                for (int row = 0; row < result.rows(); row++) {
                    for (int col = 0; col < result.cols(); col++) {
                        if (result.get(row, col)[0] > (double) THRESH_TEMPLATE_MATCH && isNewPoint(col, row, xarray, yarray))
                        {
                            xarray.add(col);
                            yarray.add(row);
                            //matchScores.add(result.get(row, col)[0]);
                            RobotLog.vv("MatchPoint", "%4.2f, x=%d, y=%d", result.get(row, col)[0],  col , row);
                        }
                    }
                }
            }
            catch(Exception e)
            {
                RobotLog.ee("Exception", e.getMessage());
            }
            long t_end = System.currentTimeMillis();
            RobotLog.vv("TimeForTemplateMatch", "%d", (t_end - t_start));
            //====================================================================================
            t_end = t_start;
            Imgproc.GaussianBlur(img_gray, img_gray, new Size(5, 5), 0);

            Imgproc.threshold(img_gray, mask, THRESH_BINARY, 255, Imgproc.THRESH_BINARY);
            Imgproc.findContours(mask, contours, hiarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            //check each coutours:  if it matches the criteria,  contour area / circle > RATIO,  MIN < radius < MAX,  X> X_MIN,  Y>Y_MIN
            for (MatOfPoint c : contours) {
                double area = Imgproc.contourArea(c);
                if (area > AREA_HEXGON_MIN && area < AREA_HEXGON_MAX) {
                    float[] radius = new float[1];
                    Point center = new Point();
                    Imgproc.minEnclosingCircle(new MatOfPoint2f(c.toArray()), center, radius);
                    float ratio = (float) area / getArea(radius[0]);
                    //if (area / getArea(radius[0]) > RATIO && radius[0] > R_HEXGON_MIN && radius[0] < R_HEXGON_MAX && center.x > X_HEXGON_MIN && center.y > Y_HEXGON_MIN) {
                    if (area / getArea(radius[0]) > RATIO && radius[0] > R_HEXGON_MIN && radius[0] < R_HEXGON_MAX ) {
                        hexgons.add(c);
                        RobotLog.vv("HEXGON", "area=%4.1f,x=%d,y=%d,r=%d,ratio=%3.2f", area, (int) center.x, (int) center.y, (int) radius[0], ratio);
                    }
                }
            }
            img_gray.release();
            mask.release();
            t_end = System.currentTimeMillis();
            RobotLog.vv("TimeForContour", "%d", (t_end - t_start));
        } catch (Exception e) {
            RobotLog.ee("OPENCV" , e.getMessage());
        }
        finally {
            //===============================================cleanup===================================================
            for (int i = 0; i < channels.size(); i++) {
                channels.get(i).release();
            }
            hiarchy.release();
            contours.clear();
            mask.release();
        }

    }

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
    public Mat processFrame(Mat rgba, Boolean rotateMat) {

        Size initSize= rgba.size();
        newSize  = new Size(initSize.width * downScaleFactor, initSize.height * downScaleFactor);
        rgba.copyTo(workingMat);

        Imgproc.resize(workingMat, workingMat,newSize);

        if(rotateMat){
            Mat tempBefore = workingMat.t();

            Core.flip(tempBefore, workingMat, -1); //mRgba.t() is the transpose

            tempBefore.release();
        }

        Mat redConvert = workingMat.clone();
        Mat blueConvert = workingMat.clone();

        colorFilterRed.process(redConvert, maskRed);
        colorFilterBlue.process(blueConvert, maskBlue);

        List<MatOfPoint> contoursRed = new ArrayList<>();

        Imgproc.findContours(maskRed, contoursRed, hiarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(workingMat,contoursRed,-1,new Scalar(230,70,70),2);
        Rect chosenRedRect = null;
        double chosenRedScore = Integer.MAX_VALUE;

        MatOfPoint2f approxCurve = new MatOfPoint2f();

        for(MatOfPoint c : contoursRed) {
            MatOfPoint2f contour2f = new MatOfPoint2f(c.toArray());

            //Processing on mMOP2f1 which is in type MatOfPoint2f
            double approxDistance = Imgproc.arcLength(contour2f, true) * 0.02;
            Imgproc.approxPolyDP(contour2f, approxCurve, approxDistance, true);

            //Convert back to MatOfPoint
            MatOfPoint points = new MatOfPoint(approxCurve.toArray());

            // Get bounding rect of contour
            Rect rect = Imgproc.boundingRect(points);

            // You can find this by printing the area of each found rect, then looking and finding what u deem to be perfect.
            // Run this with the bot, on a balance board, with jewels in their desired location. Since jewels should mostly be
            // in the same position, this hack could work nicely.


            double area = Imgproc.contourArea(c);
            double areaDiffrence = 0;

            switch(detectionMode){
                case MAX_AREA:
                    areaDiffrence = -area * areaWeight;
                    break;
                case PERFECT_AREA:
                    areaDiffrence = Math.abs(perfectArea - area);
                    break;
            }

            // Just declaring vars to make my life eassy
            double x = rect.x;
            double y = rect.y;
            double w = rect.width;
            double h = rect.height;
            Point centerPoint = new Point(x + ( w/2), y + (h/2));



            double cubeRatio = Math.max(Math.abs(h/w), Math.abs(w/h)); // Get the ratio. We use max in case h and w get swapped??? it happens when u account for rotation
            double ratioDiffrence = Math.abs(cubeRatio - perfectRatio);


            double finalDiffrence = (ratioDiffrence * ratioWeight) + (areaDiffrence * areaWeight);


            // Optional to ALWAYS return a result.

            // Update the chosen rect if the diffrence is lower then the curreny chosen
            // Also can add a condition for min diffrence to filter out VERY wrong answers
            // Think of diffrence as score. 0 = perfect
            if(finalDiffrence < chosenRedScore && finalDiffrence < maxDiffrence && area > minArea){
                chosenRedScore = finalDiffrence;
                chosenRedRect = rect;
            }

            if(debugContours && area > 100){
                Imgproc.circle(workingMat,centerPoint,3,new Scalar(0,255,255),3);
                Imgproc.putText(workingMat,"Area: " + area,centerPoint,0,0.5,new Scalar(0,255,255));
            }

        }

        List<MatOfPoint> contoursBlue = new ArrayList<>();

        Imgproc.findContours(maskBlue, contoursBlue,hiarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(workingMat,contoursBlue,-1,new Scalar(70,130,230),2);
        Rect chosenBlueRect = null;
        double chosenBlueScore = Integer.MAX_VALUE;

        for(MatOfPoint c : contoursBlue) {
            MatOfPoint2f contour2f = new MatOfPoint2f(c.toArray());

            //Processing on mMOP2f1 which is in type MatOfPoint2f
            double approxDistance = Imgproc.arcLength(contour2f, true) * 0.02;
            Imgproc.approxPolyDP(contour2f, approxCurve, approxDistance, true);

            //Convert back to MatOfPoint
            MatOfPoint points = new MatOfPoint(approxCurve.toArray());

            // Get bounding rect of contour
            Rect rect = Imgproc.boundingRect(points);

            // You can find this by printing the area of each found rect, then looking and finding what u deem to be perfect.
            // Run this with the bot, on a balance board, with jewels in their desired location. Since jewels should mostly be
            // in the same position, this hack could work nicely.


            double area = Imgproc.contourArea(c);
            double areaDiffrence = 0;

            switch(detectionMode){
                case MAX_AREA:
                    areaDiffrence = -area * areaWeight;
                    break;
                case PERFECT_AREA:
                    areaDiffrence = Math.abs(perfectArea - area);
                    break;
            }


            // Just declaring vars to make my life eassy
            double x = rect.x;
            double y = rect.y;
            double w = rect.width;
            double h = rect.height;
            Point centerPoint = new Point(x + ( w/2), y + (h/2));

            double cubeRatio = Math.max(Math.abs(h/w), Math.abs(w/h)); // Get the ratio. We use max in case h and w get swapped??? it happens when u account for rotation
            double ratioDiffrence = Math.abs(cubeRatio - 1);

            double finalDiffrence = (ratioDiffrence * ratioWeight) + (areaDiffrence * areaWeight);



            // Update the chosen rect if the diffrence is lower then the curreny chosen
            // Also can add a condition for min diffrence to filter out VERY wrong answers
            // Think of diffrence as score. 0 = perfect
            if(finalDiffrence < chosenBlueScore && finalDiffrence < maxDiffrence && area > minArea){
                chosenBlueScore = finalDiffrence;
                chosenBlueRect = rect;
            }

            if(debugContours && area > 100){
                Imgproc.circle(workingMat,centerPoint,3,new Scalar(0,255,255),3);
                Imgproc.putText(workingMat,"Area: " + area,centerPoint,0,0.5,new Scalar(0,255,255));
            }

        }

        if(chosenRedRect != null){
            Imgproc.rectangle(workingMat,
                    new Point(chosenRedRect.x, chosenRedRect.y),
                    new Point(chosenRedRect.x + chosenRedRect.width, chosenRedRect.y + chosenRedRect.height),
                    new Scalar(255, 0, 0), 2);

            Imgproc.putText(workingMat,
                    "Red: " + String.format("%.2f", chosenRedScore),
                    new Point(chosenRedRect.x - 5, chosenRedRect.y - 10),
                    Core.FONT_HERSHEY_PLAIN,
                    1.3,
                    new Scalar(255, 0, 0),
                    2);
        }

        if(chosenBlueRect != null){
            Imgproc.rectangle(workingMat,
                    new Point(chosenBlueRect.x, chosenBlueRect.y),
                    new Point(chosenBlueRect.x + chosenBlueRect.width, chosenBlueRect.y + chosenBlueRect.height),
                    new Scalar(0, 0, 255), 2);

            Imgproc.putText(workingMat,
                    "Blue: " + String.format("%.2f", chosenBlueScore),
                    new Point(chosenBlueRect.x - 5, chosenBlueRect.y - 10),
                    Core.FONT_HERSHEY_PLAIN,
                    1.3,
                    new Scalar(0, 0, 255),
                    2);
        }


        if(chosenBlueRect != null && chosenRedRect != null){
            if(chosenBlueRect.x < chosenRedRect.x){
                currentOrder = JewelOrder.BLUE_RED;
                lastOrder = currentOrder;
            }else{
                currentOrder = JewelOrder.RED_BLUE;
                lastOrder = currentOrder;
            }
        }else{
            currentOrder = JewelOrder.UNKNOWN;
        }

        if (chosenBlueRect !=null)
        {

        }
        Imgproc.putText(workingMat,"Result: " + lastOrder.toString(),new Point(10,newSize.height - 30),0,1, new Scalar(255,255,0),1);
        Imgproc.putText(workingMat,"Current Track: " + currentOrder.toString(),new Point(10,newSize.height - 10),0,0.5, new Scalar(255,255,255),1);

        Imgproc.resize(workingMat,workingMat,initSize);

        redConvert.release();
        blueConvert.release();
        Imgproc.putText(workingMat,"DogeCV 1.1 Jewel: " + newSize.toString() + " - " + speed.toString() + " - " + detectionMode.toString() ,new Point(5,30),0,1.2,new Scalar(0,255,255),2);

        return workingMat;
    }


    public JewelOrder getCurrentOrder() {
        return currentOrder;
    }

    public JewelOrder getLastOrder() {
        return lastOrder;
    }
}
