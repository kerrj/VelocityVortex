package org.firstinspires.ftc.teamcode.VelocityVortex;

import android.graphics.Bitmap;
import android.renderscript.Allocation;
import android.renderscript.Element;
import android.renderscript.RenderScript;
import android.renderscript.ScriptIntrinsicBlur;
import android.renderscript.Type;
import android.util.Log;

import com.qualcomm.ftcrobotcontroller.ScriptC_blue;
import com.qualcomm.ftcrobotcontroller.ScriptC_red;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.robot.RobotState;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.teamcode.CameraStuff.FTCVuforia;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.ArrayList;

/**
 * Created by Justin on 1/23/2017.
 */
@Autonomous
public class CrashTest extends OpMode {
    private boolean initialized =false;
    private ByteBuffer buf;
    private Mat redMat,blueMat,original,hierarchy;
    AnalyzeThread analyzeThread;
    int beaconAnalysisResult=0, finalAnalysisResult;
    FTCVuforia vuforia;
    RenderScript mRS;
    ScriptC_blue blue;
    ScriptC_red red;
    ScriptIntrinsicBlur blur;
    Allocation mAllocationIn;
    Allocation getmAllocationOut;
    Object threadLock=new Object();
    Bitmap RGB565Bitmap=Bitmap.createBitmap(1280, 720, Bitmap.Config.RGB_565),RGBABitmap=Bitmap.createBitmap(1280, 720, Bitmap.Config.ARGB_8888);
    boolean resetPosition=true;
    long bookKeepingTime;
    @Override
    public void init() {
        analyzeThread=new AnalyzeThread();
        analyzeThread.start();
        InitThread t=new InitThread();
        t.start();
        blueMat=new Mat();
        redMat=new Mat();
        hierarchy=new Mat();
        original=new Mat();
        beaconAnalysisResult=0;
        vuforia=new FTCVuforia(FtcRobotControllerActivity.getActivity());
        vuforia.initVuforia();
    }

    public void init_loop(){
        telemetry.addData("initialized",initialized);
    }

    @Override
    public void loop() {
        if(initialized) {
            int result;
            synchronized (threadLock) {
                result = beaconAnalysisResult;
            }
            if (resetPosition) {
                resetPosition = false;
                result = 0;
                bookKeepingTime = System.currentTimeMillis();
            }
            if (result == 0) {
                if (System.currentTimeMillis() - bookKeepingTime > 100) {
                    bookKeepingTime = System.currentTimeMillis();
                    analyzeThread.analyze();
                }
            } else if (result == 1 || result == -1) {
                finalAnalysisResult = result;
                bookKeepingTime = System.currentTimeMillis();
                resetPosition = true;
            } else if (result == 999) {
            }
        }else{
            telemetry.addData("Not","initialized");
        }
    }
    public void stop(){
       try{
           vuforia.destroy();
           analyzeThread.kill();
       } catch (Exception e) {
           e.printStackTrace();
       }
    }

    public class InitThread extends Thread{
        public void run(){
            mRS= FtcRobotControllerActivity.getRenderScript();
            blur= ScriptIntrinsicBlur.create(mRS, Element.RGBA_8888(mRS));
            mAllocationIn = Allocation.createTyped(mRS, Type.createXY(mRS, Element.RGBA_8888(mRS), 1280, 720),
                                                   Allocation.MipmapControl.MIPMAP_NONE, Allocation.USAGE_GRAPHICS_TEXTURE | Allocation.USAGE_SCRIPT);
            getmAllocationOut = Allocation.createTyped(mRS, Type.createXY(mRS, Element.RGBA_8888(mRS), 1280, 720),
                                                       Allocation.MipmapControl.MIPMAP_NONE, Allocation.USAGE_GRAPHICS_TEXTURE | Allocation.USAGE_SCRIPT);
            Log.d("Alive","1");
            red=new ScriptC_red(mRS);
            Log.d("Alive","1");
            blue=new ScriptC_blue(mRS);
            Log.d("Alive","1");
            initialized =true;
            Log.d("renderscript","finished");
        }
    }

    public class AnalyzeThread extends Thread{
        private boolean running=true,analyze=false;

        public void kill(){
            running=false;
        }
        public void analyze(){
            analyze=true;
        }
        public void run(){
            while(running) {
                if(analyze){
//                    synchronized (threadLock) {
//                        beaconAnalysisResult = 999;
//                    }
//                    buf=vuforia.getLastFrame();
//                    Log.d("alive","1");
//                    RGB565Bitmap.copyPixelsFromBuffer(buf);
//                    Log.d("alive","1");
//                    buf.rewind();
//                    analyze=false;
//                    synchronized (threadLock) {
//                        beaconAnalysisResult = 0;
//                    }
                    //===================================================
//                    buf=vuforia.getLastFrame();
                    if(true){
                        synchronized (threadLock) {
                            beaconAnalysisResult = 999;
                        }
                        RGB565Bitmap=vuforia.getLastBitmap();
                        Utils.bitmapToMat(RGB565Bitmap, original);
                        Imgproc.cvtColor(original, original, Imgproc.COLOR_BGR2BGRA);
                        Utils.matToBitmap(original, RGBABitmap);
//                        try {
//                            File directory = FtcRobotControllerActivity.getActivity().getBaseContext().getExternalFilesDir(null);
//                            File image = new File(directory, Long.toString(System.currentTimeMillis()) + ".jpeg");
//                            image.createNewFile();
//                            FileOutputStream fos = new FileOutputStream(image);
//                            RGBABitmap.compress(Bitmap.CompressFormat.JPEG, 50,fos) ;
//                        } catch (FileNotFoundException e) {
//                            e.printStackTrace();
//                        } catch (IOException e) {
//                            e.printStackTrace();
//                        }
                        //split color onto blueMat Mat
                        mAllocationIn.copyFrom(RGBABitmap);
                        blur.setInput(mAllocationIn);
                        blur.setRadius(1);
                        blur.forEach(getmAllocationOut);
                        blue.forEach_split(getmAllocationOut,mAllocationIn);
                        mAllocationIn.copyTo(RGBABitmap);
                        Utils.bitmapToMat(RGBABitmap, blueMat);
                        red.forEach_split(getmAllocationOut,mAllocationIn);
                        mAllocationIn.copyTo(RGBABitmap);
                        Utils.bitmapToMat(RGBABitmap, redMat);
                        //convert blueMat to grayscale for contours
                        Imgproc.cvtColor(blueMat,blueMat,Imgproc.COLOR_RGBA2GRAY);
                        ArrayList<MatOfPoint> contours=new ArrayList<>();
                        Imgproc.findContours(blueMat, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
                        double blueAverage=0;
                        double blueArea=0;
                        int index=0;
                        for(MatOfPoint p:contours){
                            MatOfPoint2f points2f=new MatOfPoint2f();
                            p.convertTo(points2f, CvType.CV_32FC2);
                            Point center=Imgproc.minAreaRect(points2f).center;
                            if(center.x>20&&center.x<1180){
                                double area=Imgproc.contourArea(p);
                                if(area>5000) {
                                    index+=area;
                                    blueAverage+=center.x*area;
                                }
                            }
                        }
                        blueArea=index;
                        if(index==0)blueAverage=0;
                        if(index>0)blueAverage/=index;

                        //repeat for red mat
                        Imgproc.cvtColor(redMat,redMat,Imgproc.COLOR_RGBA2GRAY);
                        contours=new ArrayList<>();//initialize OpenCV objects needed for processing
                        //find contours
                        Imgproc.findContours(redMat, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
                        double redAverage=0;//initialize average variables
                        double redArea=0;
                        index=0;
                        for(MatOfPoint points:contours){//iterate over contours
                            MatOfPoint2f points2f=new MatOfPoint2f();//semantics
                            points.convertTo(points2f, CvType.CV_32FC2);//semantics
                            Point center=Imgproc.minAreaRect(points2f).center;//calculate the center of the contour
                            if(center.x<1180&&center.x>20) {//if the contour is in the center
                                double area = Imgproc.contourArea(points);//find contour area
                                if (area > 5000) {//if the contour is large
                                    index += area;//increment the total area of contours
                                    redAverage += center.x * area;//add the x position in the image multiplied by area
                                }//if
                            }//if
                        }//for
                        redArea=index;
                        if(index==0)redAverage=0;
                        if(index>0)redAverage/=index;//calculate the average center of red "mass" weighted by area

                        int r=0;
                        if(redAverage>0&&blueAverage>0){
                            if(redAverage>blueAverage) r=1;
                            if(redAverage<blueAverage) r=-1;
                        }else if(redAverage>0&&blueAverage==0&&redArea>100000){
                            r=3;
                        }else if(blueAverage>0&&redAverage==0&&blueArea>100000){
                            r=2;
                        }
                        Log.d("Result","=====================================================");
                        Log.d("Result",Integer.toString(r));
                        synchronized (threadLock) {
                            beaconAnalysisResult = r;
                        }
                    }else{
                        synchronized (threadLock) {
                            beaconAnalysisResult = 0;
                        }
                    }
                    analyze=false;
                }else{
                    //idle
                }
            }
        }
    }
}
