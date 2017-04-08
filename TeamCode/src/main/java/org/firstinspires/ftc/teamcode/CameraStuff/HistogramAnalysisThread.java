package org.firstinspires.ftc.teamcode.CameraStuff;

import android.graphics.Bitmap;
import android.renderscript.Allocation;
import android.renderscript.RSIllegalArgumentException;
import android.renderscript.Script;
import android.renderscript.ScriptIntrinsicHistogram;

import com.justin.opencvcamera.ScriptC_colorsplit;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.HashMap;

/**
 * Created by Justin on 1/27/2017.
 */
public class HistogramAnalysisThread extends Thread {
    private boolean running=true;
    private boolean analyze=true;

    private FTCVuforia vuforia;
    private Allocation mAllocationIn;
    private Allocation leftHistogramAllocation,rightHistogramAllocation,mAllocationOut;
    private ScriptIntrinsicHistogram leftHistogram,rightHistogram;
    private ScriptC_colorsplit colorsplit;
    private Script.LaunchOptions leftOptions,rightOptions;
    private Bitmap RGBABitmap =Bitmap.createBitmap(1280, 720, Bitmap.Config.ARGB_8888);
    private Bitmap RGB565Bitmap =Bitmap.createBitmap(1280, 720, Bitmap.Config.RGB_565);
    private Object dataLock= new Object();
    private double distance;

    public enum BeaconResult{ RED_LEFT,RED_RIGHT,INCONCLUSIVE}

    private int accumulationValue=0;
    private final int DECISION_THRESHOLD=20000;

    public void resetResult(){
        accumulationValue=0;
    }

    public BeaconResult getBeaconResult(){
        if(accumulationValue>DECISION_THRESHOLD){
            synchronized (dataLock) {
                return BeaconResult.RED_LEFT;
            }
        }else if(accumulationValue<-DECISION_THRESHOLD){
            synchronized (dataLock) {
                return BeaconResult.RED_RIGHT;
            }
        }else{
            synchronized (dataLock) {
                return BeaconResult.INCONCLUSIVE;
            }
        }
    }
    public void kill(){
        running=false;
    }

    public void startAnalyzing(){
        analyze=true;
    }

    public void stopAnalyzing(){
        analyze=false;
    }

    public HistogramAnalysisThread(FTCVuforia vuforia){
        this.vuforia=vuforia;
        mAllocationIn= FtcRobotControllerActivity.getmAllocationIn();
        leftHistogram=FtcRobotControllerActivity.getLeftHistogram();
        rightHistogram=FtcRobotControllerActivity.getRightHistogram();
        leftHistogramAllocation=FtcRobotControllerActivity.getLeftHistogramAllocation();
        rightHistogramAllocation=FtcRobotControllerActivity.getRightHistogramAllocation();
        leftOptions=FtcRobotControllerActivity.getLeftOptions();
        rightOptions=FtcRobotControllerActivity.getRightOptions();
        colorsplit=FtcRobotControllerActivity.getColorsplit();
        mAllocationOut=FtcRobotControllerActivity.getmAllocationOut();
        running=true;
    }
    public int getConfidence(){
        synchronized (dataLock){
            return accumulationValue;
        }
    }
    public void run(){
        while(running) {
            if (!analyze) {
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            } else {
                HashMap<String, double[]> data = vuforia.getVuforiaData();
                int top, left, bottom, right, middleLeft,middleRight;
                if(data.containsKey("Wheels")) {
                    top = (int) data.get("Wheels")[16];
                    bottom = (int) Math.max(0,data.get("Wheels")[14]);
                    left = (int) data.get("Wheels")[9];
                    right = (int) data.get("Wheels")[11];
                    middleLeft = (int) data.get("Wheels")[7];
                    middleRight = (int) data.get("Wheels")[17];
                }else        if(data.containsKey("Tools")) {
                    top = (int) data.get("Tools")[16];
                    bottom = (int) Math.max(0,data.get("Tools")[14]);
                    left = (int) data.get("Tools")[9];
                    right = (int) data.get("Tools")[11];
                    middleLeft = (int) data.get("Tools")[7];
                    middleRight = (int) data.get("Tools")[17];
                }else        if(data.containsKey("Gears")) {
                    top = (int) data.get("Gears")[16];
                    bottom = (int) Math.max(0,data.get("Gears")[14]);
                    left = (int) data.get("Gears")[9];
                    right = (int) data.get("Gears")[11];
                    middleLeft = (int) data.get("Gears")[7];
                    middleRight = (int) data.get("Gears")[17];
                }else        if(data.containsKey("Legos")) {
                    top = (int) data.get("Legos")[16];
                    bottom = (int) Math.max(0,data.get("Legos")[14]);
                    left = (int) data.get("Legos")[9];
                    right = (int) data.get("Legos")[11];
                    middleLeft = (int) data.get("Legos")[7];
                    middleRight = (int) data.get("Legos")[17];
                }else{
                    continue;
                }

                RGB565Bitmap = vuforia.getLastBitmap();
                Mat original = new Mat();
                Utils.bitmapToMat(RGB565Bitmap, original);
                Imgproc.cvtColor(original, original, Imgproc.COLOR_BGR2BGRA);
                Utils.matToBitmap(original, RGBABitmap);
                mAllocationIn.copyFrom(RGBABitmap);

                try {
                    File directory = FtcRobotControllerActivity.getActivity().getBaseContext().getExternalFilesDir(null);
                    File image = new File(directory, Long.toString(System.currentTimeMillis()) + ".jpeg");
                    image.createNewFile();
                    FileOutputStream fos = new FileOutputStream(image);
                    RGBABitmap.compress(Bitmap.CompressFormat.JPEG, 50,fos) ;
                } catch (FileNotFoundException e) {
                    e.printStackTrace();
                } catch (IOException e) {
                    e.printStackTrace();
                }

                left = clipX(left);
                right = clipX(right);
                top = clipY(top);
                bottom = clipY(bottom);
                middleLeft=clipX(middleLeft);
                middleRight=clipX(middleRight);
                if (bottom == top && top == 719) {
                    top--;
                } else if (bottom == top && top == 1) {
                    bottom++;
                }
                colorsplit.forEach_split(mAllocationIn,mAllocationOut);
                try {
                    leftOptions.setX(middleLeft,left);
                    //        leftOptions.setX(left,middleLeft);
                    leftOptions.setY(bottom,top);
                    //        leftOptions.setY(top,bottom);
                    leftHistogram.forEach(mAllocationOut, leftOptions);

                    rightOptions.setX(right,middleRight);
                    //        rightOptions.setX(middleRight,right);
                    rightOptions.setY(bottom,top);
                    //        rightOptions.setY(top,bottom);
                    rightHistogram.forEach(mAllocationOut, rightOptions);
                } catch (RSIllegalArgumentException e) {
                    e.printStackTrace();
                    continue;
                }

                int[] leftData = new int[leftHistogramAllocation.getBytesSize() / 4];
                int[] leftR = new int[256];
                int[] leftG = new int[256];
                int[] leftB = new int[256];

                int[] rightData = new int[rightHistogramAllocation.getBytesSize() / 4];
                int[] rightR = new int[256];
                int[] rightG = new int[256];
                int[] rightB = new int[256];

                leftHistogramAllocation.copyTo(leftData);
                rightHistogramAllocation.copyTo(rightData);

                //==================================================================================================================
                //RED AND BLUE CHANNELS ARE SWITCHED SO WE SET THEM INTENTIONALLY TO THE WRONG CHANNEL
                //==================================================================================================================
                decodeHistogram(leftData, leftB, leftG, leftR);
                decodeHistogram(rightData, rightB, rightG, rightR);
                //==================================================================================================================
                //RED AND BLUE CHANNELS ARE SWITCHED SO WE SET THEM INTENTIONALLY TO THE WRONG CHANNEL
                //==================================================================================================================

                int histogramThreshold = 200;
                int total = 0;
                int index = 0;
                int leftRTotal = 0, leftBTotal = 0, rightRTotal = 0, rightBTotal = 0;
                for (int i : leftR) {
                    if (index > histogramThreshold) {
                        total += i;
                    }
                    index++;
                }
                leftRTotal = total;

                index = 0;
                total = 0;
                for (int i : leftB) {
                    if (index > histogramThreshold) {
                        total += i;
                    }
                    index++;
                }
                leftBTotal = total;


                index = 0;
                total = 0;
                for (int i : rightR) {
                    if (index > histogramThreshold) {
                        total += i;
                    }
                    index++;
                }
                rightRTotal = total;

                index = 0;
                total = 0;
                for (int i : rightB) {
                    if (index > histogramThreshold) {
                        total += i;
                    }
                    index++;
                }
                rightBTotal = total;

                //            Log.d("LeftR",Integer.toString(leftRTotal));
                //            Log.d("LeftB",Integer.toString(leftBTotal));
                //            Log.d("RightR",Integer.toString(rightRTotal));
                //            Log.d("RightB",Integer.toString(rightBTotal));

                //positive is on the left
                int redDifference = leftRTotal - rightRTotal;
                int blueDifference = leftBTotal - rightBTotal;
                //positive is red on the left
                int perceptronOutput = redDifference - blueDifference;

                //            Log.d("RedDifference",Integer.toString(redDifference));
                //            Log.d("BlueDifference",Integer.toString(blueDifference));
                //            Log.d("PerceptronValue",Integer.toString(perceptronOutput));

                accumulationValue += perceptronOutput;
                if (Math.abs(accumulationValue) > DECISION_THRESHOLD+1) {
                    stopAnalyzing();
                }
            }
        }
    }
    public int  clipX(int x){
        if(x>1279){
            x=1279;
        }else if(x<1){
            x=1;
        }
        return x;
    }
    public int clipY(int y){
        if(y>719){
            y=719;
        }else if(y<1){
            y=1;
        }
        return y;
    }
    public void decodeHistogram(int[] input, int[] r,int[] g,int[] b){
        int inputIndex=0;
        int outputIndex=0;
        while(inputIndex<1024){
            r[outputIndex]=input[inputIndex];
            inputIndex++;
            g[outputIndex]=input[inputIndex];
            inputIndex++;
            b[outputIndex]=input[inputIndex];
            inputIndex+=2;
            outputIndex++;
        }
    }
}
