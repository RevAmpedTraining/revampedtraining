package com.revAmped.util;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.Frame;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.State;
import com.vuforia.Vuforia;
import java.io.File;
import java.io.FileOutputStream;
import android.content.Context;
import android.os.Environment;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;

import java.util.concurrent.Semaphore;

// A hack that works around lack of access in v2.2 to camera image data when Vuforia is running
// Note: this may or may not be supported in future releases.
public class VuforiaLocalizerImplSubClass extends VuforiaLocalizerImpl {

    /** {@link CloseableFrame} exposes a close() method so that we can proactively
     * reduce memory pressure when we're done with a Frame */


    public Bitmap bm;
    private Bitmap bitmapTemp = null;
    static Semaphore semaphore = new Semaphore(1);
    public Context context;
    int image_debug_cnt = 0;
    public class CloseableFrame extends Frame {
        public CloseableFrame(Frame other) { // clone the frame so we can be useful beyond callback
            super(other);
        }
        public void close() {
            super.delete();
        }
    }

    public class VuforiaCallbackSubclass extends VuforiaLocalizerImpl.VuforiaCallback {

        @Override
        public synchronized void Vuforia_onUpdate(State state) {

            super.Vuforia_onUpdate(state);
            // We wish to accomplish two things: (a) get a clone of the Frame so we can use
            // it beyond the callback, and (b) get a variant that will allow us to proactively
            // reduce memory pressure rather than relying on the garbage collector (which here
            // has been observed to interact poorly with the image data which is allocated on a
            // non-garbage-collected heap). Note that both of this concerns are independent of
            // how the Frame is obtained in the first place.
            CloseableFrame frame = new CloseableFrame(state.getFrame());
            RobotLog.vv(TAG, "received Vuforia frame#=%d", frame.getIndex());
            //telmetry.addData("received frame#=%d")
            long numImages = frame.getNumImages();

            for (int i = 0; i < numImages; i++) {
                if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                    Image rgb = frame.getImage(i);
                    if (rgb.getPixels() !=null) {
                        image_debug_cnt = image_debug_cnt + 1;
                       // if (image_debug_cnt % 3 == 0) {   //every 3 frame,  update
                            try {
                                semaphore.acquire();
                                if (bitmapTemp == null) {
                                    bitmapTemp = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
                                }
                                bitmapTemp.copyPixelsFromBuffer((rgb.getPixels()));
                                //RobotLog.vv(TAG, "bitmap recived: #=%d", i);
                                //writeBitmap(bitmapTemp, true);
                            }catch(Exception e)
                            {
                                //e.printStackTrace();
                                RobotLog.vv("Vuforia", e.getMessage());
                            }
                            finally
                            {
                                semaphore.release();
                            }
                       // }
                    }
                    break;
                }
            }

            frame.close();
        }
    }

    public VuforiaLocalizerImplSubClass(VuforiaLocalizer.Parameters parameters) {
        super(parameters);
        stopAR();
        clearGlSurface();

        this.vuforiaCallback = new VuforiaCallbackSubclass();
        startAR();

        context = this.activity.getApplicationContext();
        RobotLog.vv(TAG, "Got Context: " + context.getFilesDir().getAbsolutePath());
        // Optional: set the pixel format(s) that you want to have in the callback
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
    }

    public void clearGlSurface() {
        if (this.glSurfaceParent != null) {
            appUtil.synchronousRunOnUiThread(new Runnable() {
                @Override
                public void run() {
                    glSurfaceParent.removeAllViews();
                    glSurfaceParent.getOverlay().clear();
                    glSurface = null;
                }
            });
        }
    }

    //
    public Bitmap getBitmap()
    {
            try
            {
                semaphore.acquire();
                bm = bitmapTemp.copy(bitmapTemp.getConfig(), true);
                writeBitmap(bitmapTemp, false);
            }
            catch(Exception e)
            {
                //e.printStackTrace();
                RobotLog.vv("Vuforia", e.getMessage());
            }
            finally {
                semaphore.release();
            }
            return bm;
    }

    /*
    store 5 images to debugging.
    *///
    public void writeBitmap(Bitmap data, Boolean rotateMat)
    {

        //RobotLog.vv(TAG, "BitmapCounter=%d" , image_debug_cnt);
        //if (image_debug_cnt > 8 && image_debug_cnt < 11  ) {
            String filename = "pictograph" + Integer.toString(image_debug_cnt) + ".jpg";
            File sd = Environment.getExternalStorageDirectory();//context.getFilesDir();
            File dest1 = new File(sd, filename);
            RobotLog.vv(TAG, "ImageSavingTo:" + dest1.getAbsolutePath());
            //Bitmap bitmap = data.getExtras().get("data");
            try {

                FileOutputStream out = new FileOutputStream(dest1);

                data.compress(Bitmap.CompressFormat.JPEG, 90, out);

                out.flush();
                out.close();
                RobotLog.vv(TAG, "ImageSavedTo:" + dest1.getAbsolutePath());

/*
                //Crop the image and save.
                Mat origimg = new Mat(data.getHeight(), data.getWidth(), CvType.CV_8UC3);   // 8 bits  3 channel
                Utils.bitmapToMat(data, origimg);


                Mat dst2 = new Mat(origimg, new Range(0, origimg.rows()/2), new Range(0,
                        origimg.cols()/2));
                Bitmap l_bitmap = Bitmap.createBitmap(dst2.cols(), dst2.rows(),
                        Bitmap.Config.RGB_565);
                Utils.matToBitmap(dst2, l_bitmap);


                if(rotateMat){
                    Mat tempBefore = origimg.t();

                    Core.flip(tempBefore, origimg, -1); //mRgba.t() is the transpose

                    tempBefore.release();
                    Utils.matToBitmap(origimg, data);
                }
                File dest2 = new File(sd, "trans_"+ filename);
                FileOutputStream out2 = new FileOutputStream(dest2);
                l_bitmap.compress(Bitmap.CompressFormat.PNG, 90, out2);
                out2.flush();
                out2.close();
*/
            } catch (Exception e) {
                //e.printStackTrace();
                RobotLog.ee("Vuforia", e.getMessage());
            }
       // }
    }
    /*
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        ImageView iv = new ImageView(this);
        Mat dst = new Mat();
        Mat img = Highgui.imread("/sdcard/img.png");
        Imgproc.cvtColor(img, dst, Imgproc.COLOR_BGR2RGBA , 4);

        //crop:  specify row and then col.
        Mat dst2 = new Mat(dst, new Range(0, dst.rows()/2), new Range(0,
                dst.cols()/2));
        Bitmap bitmap = Bitmap.createBitmap(dst2.cols(), dst2.rows(),
                Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(dst2, bitmap);
        iv.setImageBitmap(bitmap);
        setContentView(iv);
    }*/
}