package org.firstinspires.ftc.teamcode.VelocityVortex;

import android.graphics.Bitmap;
import android.renderscript.Allocation;
import android.renderscript.Element;
import android.renderscript.RenderScript;
import android.renderscript.ScriptIntrinsicBlur;
import android.renderscript.Type;
import android.util.Log;

import com.justin.opencvcamera.ScriptC_blue;
import com.justin.opencvcamera.ScriptC_red;
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
import org.opencv.objdetect.Objdetect;

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
public class Red extends Robot {

    //---------------------------------------------------------------------------------------
    //the first beacon has the TOOLS image target, and the second has the GEARS image target
    //---------------------------------------------------------------------------------------


    private boolean resetPosition=true;

    private enum RobotState{//list states here
        Shoot, RotateToFirstBeacon, DriveForward,AlignWithBeacon, AnalyzeBeacon, PressBeacon,DriveToSecondBeacon,Stop, BackUp
    }

    private RobotState robotState=RobotState.Shoot;//initialize start state here

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
    private final double CAMERA_OFFSET_FROM_PLOW=42;
    private final double SPONGE_OFFSET_FROM_CAMERA=70;
    private final double BUTTON_OFFSET_FROM_WALL=50;
    private final double BUTTON_HEIGHT_ABOVE_CAMERA=70;
    private Vector spongeVector;
    private Vector buttonVector;
    private double neckUpPosition;
    private long bookKeepingTime;
    private long pushTime, startTime,lastShot,servoTravelStart;
    private int shots=0;
    enum ShootServoState{MovingUp,MovingDown}
    private ShootServoState servoState=ShootServoState.MovingUp;
    boolean resetServoTime=true;
    private JSONObject json;
    double shootPower;
    double driveAngle;
    double rotateConstant;
    private Object threadLock=new Object();
    private AnalyzeThread analyzeThread;
    private Vector imageVector;




    @Override
    public void init() {
        super.init();
        analyzeThread=new AnalyzeThread();
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
        OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_2_4_11, FtcRobotControllerActivity.getActivity().getBaseContext(), new LoaderCallbackInterface() {
            @Override
            public void onManagerConnected(int status) {
                if (status == LoaderCallbackInterface.SUCCESS) {
                    Log.d("opencv","success");
                }
            }

            @Override
            public void onPackageInstall(int operation, InstallCallbackInterface callback) {

            }
        });
        mRS=RenderScript.create(FtcRobotControllerActivity.getActivity().getBaseContext());
        blur=ScriptIntrinsicBlur.create(mRS,Element.RGBA_8888(mRS));
        mAllocationIn = Allocation.createTyped(mRS, Type.createXY(mRS, Element.RGBA_8888(mRS), 1280, 720),
                                               Allocation.MipmapControl.MIPMAP_NONE, Allocation.USAGE_GRAPHICS_TEXTURE | Allocation.USAGE_SCRIPT);
        getmAllocationOut = Allocation.createTyped(mRS, Type.createXY(mRS, Element.RGBA_8888(mRS), 1280, 720),
                                                   Allocation.MipmapControl.MIPMAP_NONE, Allocation.USAGE_GRAPHICS_TEXTURE | Allocation.USAGE_SCRIPT);
        red=new ScriptC_red(mRS);
        blue=new ScriptC_blue(mRS);
        swerveDrive.resetPosition();
        //        dataLogger.start();
        File directory=FtcRobotControllerActivity.getActivity().getExternalFilesDir(null);
        File red=new File(directory,"red.txt");
        if(!red.exists()){
            try {
                red.createNewFile();
                FileOutputStream fos=new FileOutputStream(red);
                String contents="{\"ShootPower\":.65,\"DriveAngle\":30,\"RotateConstant\":.3}";
                fos.write(contents.getBytes());
                fos.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        FileInputStream fis= null;
        try {
            fis = new FileInputStream(red);
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
        } catch (JSONException e){
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
            rotateConstant=.3;
            e.printStackTrace();
        }
    }
    @Override
    public void init_loop(){
        swerveDrive.refreshValues();
        swerveDrive.drive(1,0,0,0);
        swerveDrive.update(true,15,false);
    }

    @Override
    public void loop() {
        super.loop();

        //first gra b an instance of FTCTarget for each target we care about: Wheels and Legos
        HashMap<String, double[]> data = vuforia.getVuforiaData();
        FTCTarget tools = new FTCTarget();
        FTCTarget gears = new FTCTarget();
        try {
            if (getTargets(data).contains("Tools")) {
                tools = new FTCTarget(data, "Tools");
            }
            if (getTargets(data).contains("Gears")) {
                gears = new FTCTarget(data, "Gears");
            }
        }catch(NullPointerException e){
            e.printStackTrace();
        }
        FTCTarget currentBeacon;
        if(beaconsPressed==0){
            currentBeacon=gears;
        }else{
            currentBeacon=tools;
        }
        switch(robotState){
            case DriveForward:
                if(resetPosition){
                    swerveDrive.resetPosition();
                    resetPosition=false;
                    shootLeft.setPower(shootPower);
                    shootRight.setPower(shootPower);
                }
                if(swerveDrive.getLinearInchesTravelled()<10){
                    swerveDrive.drive(-1,0,0,.3);
                }else{
                    robotState=RobotState.Shoot;
                    resetPosition=true;
                }
                break;
            case Shoot:
                swerveDrive.drive(0,1,-rotateConstant,0);
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
                        if(System.currentTimeMillis()-lastShot>500){
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
                double DISTANCE=50;
                double currentHeading=gyro.getHeading();
                double desiredAngle=Math.toRadians(45);
                Vector targetVector = new Vector(Math.cos(Math.PI/2), Math.sin(Math.PI/2));
                Vector currentVector = new Vector(Math.cos(Math.toRadians(currentHeading)), Math.sin(Math.toRadians(currentHeading)));
                //angleBetween is the angle from currentPosition to target position in radians
                //it has a range of -pi to pi, with negative values being clockwise and positive counterclockwise of the current angle
                double angleBetween = Math.atan2(currentVector.x * targetVector.y - currentVector.y * targetVector.x, currentVector.x * targetVector.x + currentVector.y * targetVector.y);
                if(!getTargets(data).contains("Wheels")){
                    swerveDrive.drive(0,1,-rotateConstant,.5-scale(swerveDrive.getLinearInchesTravelled(),0,DISTANCE,0,.4));
                }else{
                    resetPosition=true;
                    robotState=RobotState.AlignWithBeacon;
                }
                break;

            case AlignWithBeacon:
                double Y_ROTATION_TOLERANCE=5;//degrees
                if(currentBeacon.isFound()) {
                    Vector direction = new Vector(currentBeacon.getDistance() - 250, currentBeacon.getHorizontalDistance());
                    if (direction.getMagnitude() > 20||Math.abs(currentBeacon.getYRotation())>Math.toRadians(Y_ROTATION_TOLERANCE)) {//10mm tolerance
                        swerveDrive.drive(direction.x, direction.y, currentBeacon.getYRotation(), scale(direction.getMagnitude(),0,250,.02,.2));
                    } else {//robot is fully aligned
                        imageVector=new Vector(currentBeacon.getDistance(),currentBeacon.getHorizontalDistance());
                        swerveDrive.drive(direction.x,direction.y,0,0);
                        robotState = RobotState.AnalyzeBeacon;
                        beaconAnalysisResult = 0;
                        wait = false;
                        double angle=Math.toDegrees(Math.asin(BUTTON_HEIGHT_ABOVE_CAMERA/(currentBeacon.getDistance()-BUTTON_OFFSET_FROM_WALL)));
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
                if(result==0){
                    swerveDrive.drive(1,0,0,0);
                    if(System.currentTimeMillis()-bookKeepingTime>300){
                        analyzeThread=new AnalyzeThread();
                        analyzeThread.start();
                        if(beaconAnalysisResult!=0) {
                            wait = true;
                        }else{
                            //need to adjust here
                        }
                    }
                }else if(result==1||result==-1){
                    finalAnalysisResult=result;
                    buttonWheel.setPosition(WHEEL_OUT);
                    neck.setPosition(NECK_FLAT);
                    robotState=RobotState.PressBeacon;
                    bookKeepingTime=System.currentTimeMillis();
                    resetPosition=true;
                }else if(result==999){
                    swerveDrive.drive(1,0,0,0);
                }
                break;
            case PressBeacon:
                neck.setPosition(NECK_FLAT);
                buttonWheel.setPosition(WHEEL_OUT);
                if(resetPosition){
                    resetPosition = false;
                    swerveDrive.resetPosition();
                    spongeVector = new Vector(imageVector.x-SPONGE_OFFSET_FROM_CAMERA-BUTTON_OFFSET_FROM_WALL, imageVector.y + CAMERA_OFFSET_FROM_PLOW);
                    if (finalAnalysisResult == -1){
                        buttonVector = new Vector(spongeVector.x, spongeVector.y - BUTTON_OFFSET_FROM_TARGET);
                    } else if (finalAnalysisResult == 1){
                        buttonVector = new Vector(spongeVector.x, spongeVector.y + BUTTON_OFFSET_FROM_TARGET);
                    }
                    DRIVE_DISTANCE = mmToInch(buttonVector.getMagnitude()+1);
                }
                if (swerveDrive.getLinearInchesTravelled() < DRIVE_DISTANCE) {
                    swerveDrive.drive(buttonVector.x, buttonVector.y, 0, .4);
                } else {
                    robotState = RobotState.BackUp;
                    pushTime=System.currentTimeMillis();
                    resetPosition = true;
                }
                break;
//            case PressBeacon:
//                vuforia.cameraLight(true);
//                neck.setPosition(NECK_FLAT);
//                buttonWheel.setPosition(WHEEL_OUT);
//                if(System.currentTimeMillis()-bookKeepingTime>300) {
//                    if (resetPosition){
//                        if(currentBeacon.isFound()) {
//                            resetPosition = false;
//                            swerveDrive.resetPosition();
//                            spongeVector = new Vector(currentBeacon.getDistance()-SPONGE_OFFSET_FROM_CAMERA-BUTTON_OFFSET_FROM_WALL, currentBeacon.getHorizontalDistance() + CAMERA_OFFSET_FROM_PLOW);
//                            if (finalAnalysisResult == -1) {
//                                buttonVector = new Vector(spongeVector.x, spongeVector.y - BUTTON_OFFSET_FROM_TARGET);
//                            } else if (finalAnalysisResult == 1) {
//                                buttonVector = new Vector(spongeVector.x, spongeVector.y + BUTTON_OFFSET_FROM_TARGET);
//                            }
//                            DRIVE_DISTANCE = mmToInch(buttonVector.getMagnitude()+1);
//                        }else{
//                            swerveDrive.drive(1,0,0,0);
//                            break;
//                        }
//                    }
//                }else{
//                    swerveDrive.drive(1,0,0,0);
//                    break;
//                }
//                if (swerveDrive.getLinearInchesTravelled() < DRIVE_DISTANCE){
//                    swerveDrive.drive(buttonVector.x, buttonVector.y, 0, .4);
//                } else {
//                    robotState = RobotState.BackUp;
//                    pushTime=System.currentTimeMillis();
//                    resetPosition = true;
//                }
//                break;


            case BackUp:
                if(resetPosition){
                    swerveDrive.resetPosition();
                    resetPosition=false;
                }
                if (swerveDrive.getLinearInchesTravelled() < 1*DRIVE_DISTANCE) {
                    swerveDrive.drive(-buttonVector.x, -.5*buttonVector.y, 0, .4);
                } else {
                    buttonWheel.setPosition(WHEEL_IN);
                    resetPosition = true;
                    beaconsPressed++;
                    if(beaconsPressed==1){
                        robotState=RobotState.DriveToSecondBeacon;
                    }else{
                        robotState=RobotState.Stop;
                    }
                    beaconAnalysisResult=0;
                }

                break;

            case DriveToSecondBeacon:
                sweeper.setPower(SWEEPER_OUTAKE);
                currentHeading=gyro.getHeading();
                targetVector = new Vector(Math.cos(Math.PI/2), Math.sin(Math.PI/2));
                currentVector = new Vector(Math.cos(Math.toRadians(currentHeading)), Math.sin(Math.toRadians(currentHeading)));
                //angleBetween is the angle from currentPosition to target position in radians
                //it has a range of -pi to pi, with negative values being clockwise and positive counterclockwise of the current angle
                angleBetween = Math.atan2(currentVector.x * targetVector.y - currentVector.y * targetVector.x, currentVector.x * targetVector.x + currentVector.y * targetVector.y);

                vuforia.cameraLight(true);
                neck.setPosition(NECK_FLAT);
                if(resetPosition){
                    swerveDrive.resetPosition();
                    resetPosition=false;
                }
                DISTANCE=40;
                if(!getTargets(data).contains("Tools")){
                    swerveDrive.drive(-.3,1,angleBetween/2,.5-scale(swerveDrive.getLinearInchesTravelled(),0,DISTANCE,0,.4));
                }else{
                    beaconAnalysisResult=0;
                    robotState=RobotState.AlignWithBeacon;
                }
                break;

            case Stop:
                swerveDrive.drive(1,0,0,0);
                break;
        }//switch
        swerveDrive.update(wait,15,true);
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
    public class AnalyzeThread extends Thread{
        public void run(){
            Image i=vuforia.getLastFrame();
            if(i!=null) {
                synchronized (threadLock) {
                    beaconAnalysisResult = 999;
                }
                ByteBuffer buf=i.getPixels();
                RGB565Bitmap.copyPixelsFromBuffer(buf);
                Mat original=new Mat();
                Log.d("alive","1");
                //convert to rgba from rgb565
                Utils.bitmapToMat(RGB565Bitmap, original);
                Imgproc.cvtColor(original,original,Imgproc.COLOR_BGR2BGRA);
                Utils.matToBitmap(original, RGBABitmap);
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
                //split color onto blueMat Mat
                Log.d("alive","2");
                mAllocationIn.copyFrom(RGBABitmap);
                Log.d("alive","3");
                blur.setInput(mAllocationIn);
                blur.setRadius(10);
                Log.d("alive","4");
                blur.forEach(getmAllocationOut);
                Log.d("alive","5");
                blue.forEach_split(getmAllocationOut,mAllocationIn);
                Log.d("alive","6");
                mAllocationIn.copyTo(RGBABitmap);
                Mat blueMat=new Mat();
                Utils.bitmapToMat(RGBABitmap, blueMat);
                Log.d("alive","7");
                red.forEach_split(getmAllocationOut,mAllocationIn);
                Log.d("alive","8");
                mAllocationIn.copyTo(RGBABitmap);
                Mat redMat=new Mat();
                Utils.bitmapToMat(RGBABitmap, redMat);
                //convert blueMat to grayscale for contours
                Imgproc.cvtColor(blueMat,blueMat,Imgproc.COLOR_RGBA2GRAY);
                ArrayList<MatOfPoint> contours=new ArrayList<>();
                Mat hierarchy=new Mat();
                Imgproc.findContours(blueMat, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
                double blueAverage=0;
                double blueArea=0;
                int index=0;
                for(MatOfPoint p:contours){
                    MatOfPoint2f points2f=new MatOfPoint2f();
                    p.convertTo(points2f, CvType.CV_32FC2);
                    Point center=Imgproc.minAreaRect(points2f).center;
                    if(center.x>100&&center.x<1100){
                        double area=Imgproc.contourArea(p);
                        if(area>30000) {
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
                contours=new ArrayList<>();
                hierarchy=new Mat();
                Imgproc.findContours(redMat, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
                double redAverage=0;
                double redArea=0;
                index=0;
                for(MatOfPoint points:contours){
                    MatOfPoint2f points2f=new MatOfPoint2f();
                    points.convertTo(points2f, CvType.CV_32FC2);
                    Point center=Imgproc.minAreaRect(points2f).center;
                    if(center.x<1100&&center.x>100) {
                        double area = Imgproc.contourArea(points);
                        if (area > 30000) {
                            index += area;
                            redAverage += center.x * area;
                        }
                    }
                }
                redArea=index;
                if(index==0)redAverage=0;
                if(index>0)redAverage/=index;

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
        }
    }
}
