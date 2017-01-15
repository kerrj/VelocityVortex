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
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.vuforia.Image;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.teamcode.CameraStuff.FTCCamera;
import org.firstinspires.ftc.teamcode.CameraStuff.FTCTarget;
import org.firstinspires.ftc.teamcode.CameraStuff.FTCVuforia;
import org.firstinspires.ftc.teamcode.Swerve.Core.Vector;
import org.opencv.android.InstallCallbackInterface;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
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
import java.util.HashMap;

/**
 * Created by Justin on 10/7/2016.
 */
@Autonomous
public class Shoot extends Robot {

    //---------------------------------------------------------------------------------------
    //the first beacon has the WHEELS image target, and the second has the LEGOS image target
    //---------------------------------------------------------------------------------------


    private boolean resetPosition=true;

    private enum RobotState{//list states here
        Shoot, RotateToFirstBeacon, DriveForward,wait,DriveToFirstBeacon,AlignWithBeacon, AnalyzeBeacon, PressBeacon,DriveToSecondBeacon,Stop, VerifyBeacon,BackUp,DoubleCheckBeacon,ReAlignWithBeacon
    }

    private RobotState robotState=RobotState.DriveForward;//initialize start state here

    private int beaconsPressed=0;

    private Allocation mAllocationIn;
    private Allocation getmAllocationOut;
    private ScriptC_blue blue;
    private ScriptC_red red;
    private ScriptIntrinsicBlur blur;
    private RenderScript mRS;
    private Bitmap RGB565Bitmap =Bitmap.createBitmap(1280, 720, Bitmap.Config.RGB_565);
    private Bitmap RGBABitmap =Bitmap.createBitmap(1280, 720, Bitmap.Config.ARGB_8888);
    private FTCCamera ftcCamera;

    private int beaconAnalysisResult=0;
    private boolean wait=true;
    private double DRIVE_DISTANCE;
    private final double BUTTON_OFFSET_FROM_TARGET=65;
    private final double CAMERA_OFFSET_FROM_PLOW=40;
    private final double SPONGE_OFFSET_FROM_CAMERA=75;
    private final double BUTTON_OFFSET_FROM_WALL=60;
    private final double BUTTON_HEIGHT_ABOVE_CAMERA=100;
    private Vector spongeVector;
    private Vector buttonVector;
    private double neckUpPosition;
    private long bookKeepingTime;
    boolean resetServoTime=true;
    private long pushTime, startTime,lastShot,servoTravelStart;
    private int shots=0;
    enum ShootServoState{MovingUp,MovingDown}
    private ShootServoState servoState=ShootServoState.MovingUp;



    @Override
    public void init() {
        super.init();
        lfm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rfm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        beaconsPressed=0;
        beaconAnalysisResult=0;
        wait=true;
        resetPosition=true;
        swerveDrive.resetPosition();
        shots=0;
        bookKeepingTime=System.currentTimeMillis();
    }


    @Override
    public void loop() {
        super.loop();
        //first gra b an instance of FTCTarget for each target we care about: Wheels and Legos
        if(Math.abs(System.currentTimeMillis()-bookKeepingTime)<10000){
            swerveDrive.drive(1,0,0,0);
            swerveDrive.update(wait,15,false);
            return;
        }
        switch(robotState){
            case DriveForward:
                if(resetPosition){
                    swerveDrive.resetPosition();
                    resetPosition=false;
                    shootLeft.setPower(.65);
                    shootRight.setPower(.65);
                }
                if(swerveDrive.getLinearInchesTravelled()<10){
                    swerveDrive.drive(-1,0,0,.2);
                }else{
                    robotState=RobotState.Shoot;
                    resetPosition=true;
                }
                break;

            case Shoot:
                swerveDrive.drive(1,0,0,0);
                if(resetPosition){
                    startTime=System.currentTimeMillis();
                    resetPosition=false;
                }
                shootLeft.setPower(.65);
                shootRight.setPower(.65);
                if(System.currentTimeMillis()-startTime>200){
                    if(resetServoTime){
                        servoTravelStart=System.currentTimeMillis();
                        resetServoTime=false;
                    }
                    if(servoState==ShootServoState.MovingUp){
                        if(System.currentTimeMillis()-servoTravelStart>300){
                            shots++;
                            lastShot=System.currentTimeMillis();
                            if(shots<2) {
                                servoState = ShootServoState.MovingDown;
                            }else{
                                shootLeft.setPower(0);
                                shootRight.setPower(0);
                                robotState=RobotState.Stop;
                            }
                        }else{
                            shootServo.setPosition(SHOOTER_UP);
                        }
                    }else{
                        shootServo.setPosition(SHOOTER_DOWN);
                        if(System.currentTimeMillis()-lastShot>600){
                            servoState=ShootServoState.MovingUp;
                            servoTravelStart=System.currentTimeMillis();
                        }
                    }
                }
                break;

            case Stop:
                swerveDrive.stop();
                break;
        }//switch
        swerveDrive.update(wait,15,false);
        telemetry.addData("State",robotState);
        telemetry.addData("result",beaconAnalysisResult);
    }//loop
}
