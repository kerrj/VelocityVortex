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
import org.json.JSONException;
import org.json.JSONObject;
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
import java.io.FileInputStream;
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
public class BlueFirstSecondCapPark extends Robot {

    //---------------------------------------------------------------------------------------
    //the first beacon has the WHEELS image target, and the second has the LEGOS image target
    //---------------------------------------------------------------------------------------



    private enum RobotState{//list states here
        Shoot, RotateIntoCapBall,RotateToFirstBeacon, DriveForward,AlignWithBeacon, AnalyzeBeacon, PressBeacon,DriveToSecondBeacon,Stop, BackUp,DriveToCapBall
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

    private int beaconAnalysisResult=0, finalAnalysisResult;
    private boolean wait=true;
    private double DRIVE_DISTANCE;
    private final double BUTTON_OFFSET_FROM_TARGET=65;
    private final double CAMERA_OFFSET_FROM_PLOW=42;//side to side
    private final double SPONGE_OFFSET_FROM_CAMERA=70;//outwards
    private final double BUTTON_OFFSET_FROM_WALL=50;
    private final double BUTTON_HEIGHT_ABOVE_CAMERA=70;
    private Vector spongeVector;
    private Vector buttonVector;
    private double neckUpPosition;
    private long bookKeepingTime;
    boolean resetServoTime=true;
    private long pushTime, startTime,lastShot,servoTravelStart;
    private int shots=0;
    enum ShootServoState{MovingUp,MovingDown}
    private ShootServoState servoState=ShootServoState.MovingUp;
    private JSONObject json;
    double shootPower;
    double driveAngle;
    double rotateConstant;
    private AnalyzeThread analyzeThread;
    private Object threadLock=new Object();
    private Vector imageVector;
    private boolean initialized=true;
    private Mat redMat,blueMat,original,hierarchy;




    @Override
    public void init() {
        super.init();
//        InitThread t=new InitThread();
//        t.start();
        mRS=FtcRobotControllerActivity.getRenderScript();
        blue=FtcRobotControllerActivity.getBlue();
        red=FtcRobotControllerActivity.getRed();
        blur=FtcRobotControllerActivity.getBlur();
        mAllocationIn=FtcRobotControllerActivity.getmAllocationIn();
        getmAllocationOut=FtcRobotControllerActivity.getGetmAllocationOut();
        blueMat=new Mat();
        redMat=new Mat();
        hierarchy=new Mat();
        original=new Mat();

        analyzeThread=new AnalyzeThread();
        analyzeThread.start();
        lfm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rfm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lfm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        beaconsPressed=0;
        beaconAnalysisResult=0;
        wait=true;
        vuforia=new FTCVuforia(FtcRobotControllerActivity.getActivity());
        vuforia.initVuforia();
        vuforia.addTrackables("FTC_2016-17.xml");
        swerveDrive.resetPosition();
        shots=0;

        File directory=FtcRobotControllerActivity.getActivity().getExternalFilesDir(null);
        File blue=new File(directory,"blue.txt");
        if(!blue.exists()){
            try {
                blue.createNewFile();
                FileOutputStream fos=new FileOutputStream(blue);
                String contents="{\"ShootPower\":.65,\"DriveAngle\":30}";
                fos.write(contents.getBytes());
                fos.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        FileInputStream fis= null;
        try {
            fis = new FileInputStream(blue);
            byte[] data=new byte[fis.available()];
            fis.read(data);
            fis.close();
            String contents=new String(data,"UTF-8");
            json=new JSONObject(contents);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        } catch (JSONException e) {
            e.printStackTrace();
        }

        try {
            shootPower=json.getDouble("ShootPower");
        } catch (JSONException e) {
            shootPower=.65;
            e.printStackTrace();
        }
        try {
            driveAngle=Math.toRadians(json.getDouble("DriveAngle"));
        } catch (JSONException e) {
            driveAngle=Math.toRadians(30);
            e.printStackTrace();
        }
        try {
            rotateConstant=json.getDouble("RotateConstant");
        } catch (JSONException e) {
            rotateConstant=-.4;
            e.printStackTrace();
        }
    }

    public void init_loop(){
        swerveDrive.refreshValues();
        swerveDrive.drive(-1,0,0,0);
        swerveDrive.update(true,15,false);
        telemetry.addData("Initialized",initialized);
    }

    @Override
    public void loop() {
        super.loop();
        if(!initialized){
            return;
        }
        //first gra b an instance of FTCTarget for each target we care about: Wheels and Legos
        HashMap<String, double[]> data = vuforia.getVuforiaData();
        FTCTarget wheels = new FTCTarget();
        FTCTarget legos = new FTCTarget();
        try {
            if (getTargets(data).contains("Wheels")){
                wheels = new FTCTarget(data, "Wheels");
            }
            if (getTargets(data).contains("Legos")){
                legos = new FTCTarget(data, "Legos");
            }
        }catch(NullPointerException e){
            e.printStackTrace();
        }
        FTCTarget currentBeacon;
        if(beaconsPressed==0){
            currentBeacon=wheels;
        }else{
            currentBeacon=legos;
        }
        if(gyro.isCalibrating()){
            swerveDrive.drive(1,0,0,0);
            swerveDrive.update(true,15,false);
            return;
        }

        switch(robotState){
            case DriveForward:
                if(resetPosition){
                    swerveDrive.resetPosition();
                    resetPosition=false;
                    shootLeft.setPower(shootPower);
                    shootRight.setPower(shootPower);
                }
                if(swerveDrive.getLinearInchesTravelled()<15){
                    swerveDrive.drive(-1,0,0,.2);
                }else{
                    robotState=RobotState.Shoot;
                    resetPosition=true;
                }
                break;
            case Shoot:
                swerveDrive.drive(0,-1,rotateConstant,0);
//                swerveDrive.setPivotPoint(-7,7);
//                swerveDrive.drive(0,0,1,0);
                if(resetPosition){
                    startTime=System.currentTimeMillis();
                    resetPosition=false;
                }
                shootLeft.setPower(shootPower);
                shootRight.setPower(shootPower);
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
                                robotState=RobotState.RotateToFirstBeacon;
                                shootLeft.setPower(0);
                                shootRight.setPower(0);
                                resetPosition=true;
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

            case RotateToFirstBeacon:
                if(resetPosition){
                    swerveDrive.resetPosition();
                    resetPosition=false;
                }
                double DISTANCE=40;
                double currentHeading=gyro.getHeading();
                Vector targetVector = new Vector(Math.cos(3*Math.PI/2), Math.sin(3*Math.PI/2));
                Vector currentVector = new Vector(Math.cos(Math.toRadians(currentHeading)), Math.sin(Math.toRadians(currentHeading)));
                //angleBetween is the angle from currentPosition to target position in radians
                //it has a range of -pi to pi, with negative values being clockwise and positive counterclockwise of the current angle
                double angleBetween = Math.atan2(currentVector.x * targetVector.y - currentVector.y * targetVector.x, currentVector.x * targetVector.x + currentVector.y * targetVector.y);
                if(Math.abs(angleBetween)>Math.toRadians(4)){
                    swerveDrive.drive(0,-1,rotateConstant,.35);
                }else{
                    resetPosition=true;
                    robotState=RobotState.AlignWithBeacon;
                }
                break;


            case AlignWithBeacon:
                double Y_ROTATION_TOLERANCE=3;//degrees
                if(currentBeacon.isFound()) {
                    Vector direction = new Vector(currentBeacon.getDistance() - 250, currentBeacon.getHorizontalDistance());
                    if (direction.getMagnitude() > 15||Math.abs(currentBeacon.getYRotation())>Math.toRadians(Y_ROTATION_TOLERANCE)) {
                        swerveDrive.drive(direction.x, direction.y, currentBeacon.getYRotation()*2, scale(direction.getMagnitude(),0,250,.05,.15));
                    } else {//robot is fully aligned
                        imageVector=new Vector(currentBeacon.getDistance(),currentBeacon.getHorizontalDistance());
                        swerveDrive.drive(1,0,0,0);
                        robotState = RobotState.AnalyzeBeacon;
                        beaconAnalysisResult = 0;
                        double angle=Math.toDegrees(Math.asin((BUTTON_HEIGHT_ABOVE_CAMERA)/(currentBeacon.getDistance()-BUTTON_OFFSET_FROM_WALL)));
                        double position=NECK_FLAT-degreesToServoPosition(angle);
                        if(position>1){
                            position=1;
                        }
                        neckUpPosition =position;
                        neck.setPosition(neckUpPosition);
                        bookKeepingTime=System.currentTimeMillis();
                    }
                }else{
                    swerveDrive.drive(1,0,0,0);
                }
                break;

            case AnalyzeBeacon:
                vuforia.cameraLight(false);
                int result;
                synchronized (threadLock){
                    result=beaconAnalysisResult;
                }
                if(resetPosition){
                    result=0;
                    resetPosition=false;
                }
                if(result==0){//inconclusive
                    swerveDrive.drive(1,0,0,0);
                    if(System.currentTimeMillis()-bookKeepingTime>100){
                        bookKeepingTime=System.currentTimeMillis();
                        analyzeThread.analyze();
                    }
                }else if(result==1||result==-1){//decision made
                    finalAnalysisResult=result;
                    buttonWheel.setPosition(WHEEL_OUT);
                    neck.setPosition(NECK_FLAT);
                    robotState=RobotState.PressBeacon;
                    bookKeepingTime=System.currentTimeMillis();
                    resetPosition=true;
                }else if(result==999){//processing
                    swerveDrive.drive(1,0,0,0);
                }
                break;

            case PressBeacon:
                neck.setPosition(NECK_FLAT);
                buttonWheel.setPosition(WHEEL_OUT);
                if(resetPosition){
                    resetPosition = false;
                    swerveDrive.resetPosition();
                    spongeVector = new Vector(imageVector.x-SPONGE_OFFSET_FROM_CAMERA-BUTTON_OFFSET_FROM_WALL-5, imageVector.y + CAMERA_OFFSET_FROM_PLOW);
                    if (finalAnalysisResult == 1){
                        buttonVector = new Vector(spongeVector.x, spongeVector.y - BUTTON_OFFSET_FROM_TARGET);
                    } else if (finalAnalysisResult == -1){
                        buttonVector = new Vector(spongeVector.x, spongeVector.y + BUTTON_OFFSET_FROM_TARGET);
                    }
                    DRIVE_DISTANCE = mmToInch(buttonVector.getMagnitude())+2;
                }
                if (swerveDrive.getLinearInchesTravelled() < DRIVE_DISTANCE){
                    swerveDrive.drive(buttonVector.x, buttonVector.y, 0, .2);
                } else {
                    beaconsPressed++;
                    if(beaconsPressed==1){
                        robotState = RobotState.DriveToSecondBeacon;
                    }else{
                        robotState=RobotState.DriveToCapBall;
                    }
                    neck.setPosition(NECK_FLAT);
                    buttonWheel.setPosition(WHEEL_IN);
                    pushTime=System.currentTimeMillis();
                    resetPosition = true;
                }
                break;

            case DriveToSecondBeacon:
                sweeper.setPower(SWEEPER_INTAKE);

                currentHeading=gyro.getHeading();
                targetVector = new Vector(Math.cos(3*Math.PI/2), Math.sin(3*Math.PI/2));
                currentVector = new Vector(Math.cos(Math.toRadians(currentHeading)), Math.sin(Math.toRadians(currentHeading)));
                //angleBetween is the angle from currentPosition to target position in radians
                //it has a range of -pi to pi, with negative values being clockwise and positive counterclockwise of the current angle
                angleBetween = Math.atan2(currentVector.x * targetVector.y - currentVector.y * targetVector.x, currentVector.x * targetVector.x + currentVector.y * targetVector.y);

                neck.setPosition(NECK_FLAT);
                if(resetPosition){
                    neck.setPosition(NECK_FLAT);
                    buttonWheel.setPosition(WHEEL_IN);
                    swerveDrive.resetPosition();
                    resetPosition=false;
                }
                DISTANCE=45;
                if(swerveDrive.getLinearInchesTravelled()<DISTANCE){
                    swerveDrive.drive(-.4, -1, angleBetween / 2, .4);
                }else {
                    if (!getTargets(data).contains("Legos")) {
                        swerveDrive.drive(-.4, -1, angleBetween / 2, .1);
//                        swerveDrive.drive(-.4, -1, angleBetween / 2, .3 - scale(swerveDrive.getLinearInchesTravelled(), 0, DISTANCE, 0, .2));
                    } else {
                        beaconAnalysisResult = 0;
                        robotState = RobotState.AlignWithBeacon;
                        sweeper.setPower(0);
                    }
                }
                break;

            case DriveToCapBall:
                if(resetPosition){
                    resetPosition=false;
                    swerveDrive.resetPosition();
                }
                currentHeading=gyro.getHeading();
                targetVector = new Vector(Math.cos(3*Math.PI/2), Math.sin(3*Math.PI/2));
                currentVector = new Vector(Math.cos(Math.toRadians(currentHeading)), Math.sin(Math.toRadians(currentHeading)));
                //angleBetween is the angle from currentPosition to target position in radians
                //it has a range of -pi to pi, with negative values being clockwise and positive counterclockwise of the current angle
                angleBetween = Math.atan2(currentVector.x * targetVector.y - currentVector.y * targetVector.x, currentVector.x * targetVector.x + currentVector.y * targetVector.y);

                DRIVE_DISTANCE=60;
                if(swerveDrive.getLinearInchesTravelled()<DRIVE_DISTANCE){
                    swerveDrive.drive(-1,.7,angleBetween/2,.4);
                }else{
                    robotState=RobotState.Stop;
                    resetPosition=true;
                }

                break;

            case RotateIntoCapBall:
                if(resetPosition){
                    resetPosition=false;
                    swerveDrive.resetPosition();
                }
                if(swerveDrive.getLinearInchesTravelled()<8) {
                    swerveDrive.drive(0, 0, -1, .3);
                }else{
                    robotState=RobotState.Stop;
                }

                break;


            case Stop:
                swerveDrive.drive(0,0,1,0);
                break;
        }//switch
        swerveDrive.update(true,15,true);
        telemetry.addData("State",robotState);
        telemetry.addData("result",beaconAnalysisResult);
    }//loop







    //==========================================================================================================================
    public double mmToInch(double mm){
        return mm*.0393701;
    }
    @Override
    public void stop(){
        super.stop();
        try {
            vuforia.destroy();
        } catch (Exception e) {
            e.printStackTrace();
        }
        analyzeThread.kill();
    }

    public double degreesToServoPosition(double degrees){
        double position=degrees/180;
        return position;
    }

    /**
     *
     * @param data The HashMap generated from vuforia.getVuforiaData();
     * @return an ArrayList containing all targets found. length 0 if none found
     */
    public ArrayList<String> getTargets(HashMap<String,double[]> data){
        ArrayList<String> s=new ArrayList<>();
        if(data.containsKey("Tools")) s.add("Tools");
        if(data.containsKey("Wheels")) s.add("Wheels");
        if(data.containsKey("Legos")) s.add("Legos");
        if(data.containsKey("Gears")) s.add("Gears");
        return s;
    }

    public double scale(double current,double min,double max,double newMin, double newMax){
        double d=current/(max-min);
        double scaled=d*(newMax-newMin)+newMin;
        if(scaled>newMax){
            scaled=newMax;
        }
        if(scaled<newMin){
            scaled=newMin;
        }
        return scaled;
    }

    public class InitThread extends Thread{
        public void run(){
            mRS=FtcRobotControllerActivity.getRenderScript();
            blur=ScriptIntrinsicBlur.create(mRS,Element.RGBA_8888(mRS));
            red=new ScriptC_red(mRS);
            blue=new ScriptC_blue(mRS);
            mAllocationIn = Allocation.createTyped(mRS, Type.createXY(mRS, Element.RGBA_8888(mRS), 1280, 720),
                                                   Allocation.MipmapControl.MIPMAP_NONE, Allocation.USAGE_GRAPHICS_TEXTURE | Allocation.USAGE_SCRIPT);
            getmAllocationOut = Allocation.createTyped(mRS, Type.createXY(mRS, Element.RGBA_8888(mRS), 1280, 720),
                                                       Allocation.MipmapControl.MIPMAP_NONE, Allocation.USAGE_GRAPHICS_TEXTURE | Allocation.USAGE_SCRIPT);
            initialized=true;
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
                    synchronized (threadLock) {
                        beaconAnalysisResult = 999;
                    }
                    RGB565Bitmap=vuforia.getLastBitmap();
                    //convert to rgba from rgb565
                    Utils.bitmapToMat(RGB565Bitmap, original);
                    Imgproc.cvtColor(original,original,Imgproc.COLOR_BGR2BGRA);
                    Utils.matToBitmap(original, RGBABitmap);
                    //                Log.d("Alive","1");
                    //                try {
                    //                    File directory = FtcRobotControllerActivity.getActivity().getBaseContext().getExternalFilesDir(null);
                    //                    File image = new File(directory, Long.toString(System.currentTimeMillis()) + ".jpeg");
                    //                    image.createNewFile();
                    //                    FileOutputStream fos = new FileOutputStream(image);
                    //                    RGBABitmap.compress(Bitmap.CompressFormat.JPEG, 50,fos) ;
                    //                } catch (FileNotFoundException e) {
                    //                    e.printStackTrace();
                    //                } catch (IOException e) {
                    //                    e.printStackTrace();
                    //                }
                    //split color onto blueMat Mat
                    mAllocationIn.copyFrom(RGBABitmap);
                    blur.setInput(mAllocationIn);
                    blur.setRadius(1);
                    blur.forEach(getmAllocationOut);
                    blue.forEach_split(getmAllocationOut,mAllocationIn);
                    mAllocationIn.copyTo(RGBABitmap);
                    //                try {
                    //                    File directory = FtcRobotControllerActivity.getActivity().getBaseContext().getExternalFilesDir(null);
                    //                    File image = new File(directory, Long.toString(System.currentTimeMillis()) + ".jpeg");
                    //                    image.createNewFile();
                    //                    FileOutputStream fos = new FileOutputStream(image);
                    //                    RGBABitmap.compress(Bitmap.CompressFormat.JPEG, 50,fos) ;
                    //                } catch (FileNotFoundException e) {
                    //                    e.printStackTrace();
                    //                } catch (IOException e) {
                    //                    e.printStackTrace();
                    //                }
                    Utils.bitmapToMat(RGBABitmap, blueMat);
                    red.forEach_split(getmAllocationOut,mAllocationIn);
                    mAllocationIn.copyTo(RGBABitmap);
                    //                try {
                    //                    File directory = FtcRobotControllerActivity.getActivity().getBaseContext().getExternalFilesDir(null);
                    //                    File image = new File(directory, Long.toString(System.currentTimeMillis()) + ".jpeg");
                    //                    image.createNewFile();
                    //                    FileOutputStream fos = new FileOutputStream(image);
                    //                    RGBABitmap.compress(Bitmap.CompressFormat.JPEG, 50,fos) ;
                    //                } catch (FileNotFoundException e) {
                    //                    e.printStackTrace();
                    //                } catch (IOException e) {
                    //                    e.printStackTrace();
                    //                }
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
                    analyze=false;
                }else{
                    //idle
                }
            }
        }
    }
}
