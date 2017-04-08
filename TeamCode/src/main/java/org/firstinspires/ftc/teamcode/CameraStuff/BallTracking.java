package org.firstinspires.ftc.teamcode.CameraStuff;

import android.graphics.Bitmap;
import android.renderscript.Allocation;
import android.renderscript.ScriptC;
import android.renderscript.ScriptGroup;

import com.qualcomm.ftcrobotcontroller.ScriptC_blue;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.teamcode.VelocityVortex.Robot;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

/**
 * Created by Justin on 3/26/2017.
 */

public class BallTracking implements FTCCamera.AllocationListener {
    private FTCCamera camera;
    private Robot robot;
    int WIDTH=800,HEIGHT=480;
    private Robot.Side side;

    public TrackingState trackingState=TrackingState.Searching;
    public enum TrackingState{
        Searching,Tracking,Rotating,PickingUp
    }


    public BallTracking(Robot robot,Robot.Side side){
        camera=new FTCCamera(this);
        this.side=side;
        this.robot=robot;
        this.redGroup= FtcRobotControllerActivity.getRedGroup();
        this.blueGroup=FtcRobotControllerActivity.getBlueGroup();
    }

    public void startCamera(){
        camera.startCamera();
    }
    public void stopCamera(){
        camera.stopCamera();
    }

    public TrackingState scoreBallz(){
        switch(trackingState){
            case Searching:
                robot.swerveDrive.drive(0,0,1,.1);
                if(lastCircle!=null){
                    trackingState=TrackingState.Tracking;
                }
                break;

            case Tracking:
                double[] circle=lastCircle.clone();
                if(circle==null){
                    trackingState=TrackingState.Searching;
                    break;
                }
                robot.telemetry.addData("x",circle[0]);
                robot.telemetry.addData("y",circle[1]);
                robot.telemetry.addData("r",circle[2]);
                double circleOffset=circle[0]-WIDTH/2;
                robot.swerveDrive.drive(1,0,2*circleOffset/WIDTH,.2);
                break;
        }//switch(trackingState)
        return trackingState;
    }






    private ScriptGroup redGroup,blueGroup;
    private Bitmap bitmap=Bitmap.createBitmap(WIDTH, HEIGHT, Bitmap.Config.ARGB_8888);
    private Mat mat,circles=new Mat();
    double[] lastCircle;
    Mat ROI;
    float referenceX,referenceY;
    int up=0,down=HEIGHT,left=0,right=WIDTH;
    @Override
    public void onAllocationAvailable(Allocation inAlloc,Allocation outAlloc) {
        if(side==Robot.Side.BLUE)blueGroup.execute();
        if(side==Robot.Side.RED) redGroup.execute();
        outAlloc.copyTo(bitmap);
        Utils.bitmapToMat(bitmap, mat);
        if(lastCircle==null){
            ROI=mat;
            up=0;
            down=HEIGHT;
            left=0;
            right=WIDTH;
            referenceX=0;
            referenceY=0;
        }else{
            double radiusscalar=3;
            left=(int)(lastCircle[0]+referenceX-(radiusscalar)*lastCircle[2]);
            right=(int)(lastCircle[0]+referenceX+(radiusscalar)*lastCircle[2]);
            up=(int)(lastCircle[1]+referenceY-(radiusscalar)*lastCircle[2]);
            down=(int)(lastCircle[1]+referenceY+(radiusscalar)*lastCircle[2]);

            if(left<0){
                left=1;
            }
            if(right>mat.cols()){
                right=mat.cols()-1;
            }
            if(up<0){
                up=1;
            }
            if(down>mat.rows()){
                down=mat.rows()-1;
            }
            if(left>right){
                left=0;
                right=mat.cols()-1;
                up=0;
                down=mat.rows()-1;
            }else if(up>down){
                left=0;
                right=mat.cols()-1;
                up=0;
                down=mat.rows()-1;
            }
            ROI=mat.submat(up,down,left,right);
            referenceX=left;
            referenceY=up;
        }
        Imgproc.cvtColor(ROI, ROI, Imgproc.COLOR_RGBA2GRAY);
        Imgproc.HoughCircles(ROI,circles,Imgproc.CV_HOUGH_GRADIENT, 2//resolution modifier
                ,2000//minimum distance
                ,10//param1, canny algorithm top param unused????????
                ,40//param2, circle accumulation param
                ,20//min radius
                ,130//max radius
        );
        if(circles.cols()==0) {
            lastCircle=null;
            return;
        }
        double[] c=circles.get(0,0);
        lastCircle=c.clone();
    }
}
