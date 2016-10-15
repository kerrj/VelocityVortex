package org.firstinspires.ftc.teamcode.Swerve;

import android.renderscript.Allocation;
import android.renderscript.Element;
import android.renderscript.RenderScript;
import android.renderscript.Type;

import com.justin.opencvcamera.ScriptC_colorsplit;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.vuforia.Renderer;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.teamcode.CameraStuff.FTCCamera;
import org.firstinspires.ftc.teamcode.FTCTarget;
import org.firstinspires.ftc.teamcode.CameraStuff.FTCVuforia;

import java.util.ArrayList;
import java.util.HashMap;

/**
 * Created by Justin on 10/7/2016.
 */
@Autonomous
public class BlueSwerveVuforiaAuto extends OpMode {

    //---------------------------------------------------------------------------------------
    //the first beacon has the WHEELS image target, and the second has the LEGOS image target
    //---------------------------------------------------------------------------------------

    private static final double WIDTH=16;
    private static final double LENGTH=16;

    private boolean resetPosition=true;

    private enum RobotState{//list states here
        MoveAwayFromWall,DriveToFirstBeacon,AlignWithBeacon,PressBeacon,DriveToSecondBeacon,Stop
    }

    private FTCSwerve swerveDrive;
    private RobotState robotState=RobotState.AlignWithBeacon;//initialize start state here

    private FTCVuforia vuforia;

    private int beaconsPressed=0;

//    private FTCCamera ftcCamera;
//    private RenderScript renderScript;
//    private ScriptC_colorsplit colorsplit;
//    private Allocation allocationOut;


    @Override
    public void init() {
        vuforia=new FTCVuforia(FtcRobotControllerActivity.getActivity());
        vuforia.initVuforia();
        vuforia.addTrackables("FTC_2016-17.xml");
        swerveDrive=new FTCSwerve(hardwareMap.analogInput.get("frontLeftEncoder"),hardwareMap.analogInput.get("frontRightEncoder"),
                                  hardwareMap.analogInput.get("backLeftEncoder"),hardwareMap.analogInput.get("backRightEncoder"),
                                  hardwareMap.dcMotor.get("front"),hardwareMap.dcMotor.get("back"),
                                  hardwareMap.servo.get("frontLeftServo"),hardwareMap.servo.get("frontRightServo"),
                                  hardwareMap.servo.get("backLeftServo"),hardwareMap.servo.get("backRightServo"));
        hardwareMap.dcMotor.get("front").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareMap.dcMotor.get("back").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


//        // STUFF FOR CAMERA USAGE IN THE FUTURE
//        ftcCamera=new FTCCamera(1280,720);
//        renderScript=RenderScript.create(FtcRobotControllerActivity.getActivity().getBaseContext());
//        allocationOut=Allocation.createTyped(renderScript, Type.createXY(renderScript, Element.RGBA_8888(renderScript), 1280, 720),
//                                             Allocation.MipmapControl.MIPMAP_NONE, Allocation.USAGE_IO_INPUT | Allocation.USAGE_GRAPHICS_TEXTURE | Allocation.USAGE_SCRIPT);
//
//        colorsplit=new ScriptC_colorsplit(renderScript);
//        ftcCamera.setListener(new FTCCamera.AllocationListener() {
//            @Override
//            public void onAllocationAvailable(Allocation allocation) {
//                colorsplit.forEach_split(allocation,allocationOut);
//
//            }
//        });
    }

    @Override
    public void loop() {//CURRENTLY NOT GOING ON TO PRESS BEACON
        //first grab an instance of FTCTarget for each target we care about: Wheels and Legos
        HashMap<String,double[]> data=vuforia.getVuforiaData();
        FTCTarget wheels=new FTCTarget();
        FTCTarget legos=new FTCTarget();
        if(getTargets(data).contains("Wheels")){
            wheels=new FTCTarget(data,"Wheels");
        }
        if(getTargets(data).contains("Legos")){
            legos=new FTCTarget(data,"Legos");
        }



        switch(robotState){
            case MoveAwayFromWall:
                if(resetPosition) {
                    swerveDrive.resetPosition();
                    resetPosition=false;
                }
                if(swerveDrive.getInchesTravelled()<5) {
                    swerveDrive.driveTowards(new Vector(0, 1), 1);
                }else{
                    resetPosition=true;
                    robotState=RobotState.DriveToFirstBeacon;
                }
                break;
            case DriveToFirstBeacon:
                if(!getTargets(data).contains("Wheels")) {
                    swerveDrive.driveTowards(new Vector(1, 1), .5);
                }else{
                    resetPosition=true;
                    robotState=RobotState.AlignWithBeacon;
                }
                break;
            case AlignWithBeacon:
                //first initialize currentBeacon with the correct target depending on how many we've pressed
                FTCTarget currentBeacon;
                if(beaconsPressed==0){
                    currentBeacon=wheels;
                }else{
                    currentBeacon=legos;
                }

                if(currentBeacon.isFound()) {//if the target is available
                    //align to be mostly parallel with the beacon
                    if (currentBeacon.getYRotation() < -.02) {//if angle is negative, rotate counterclockwise
                        swerveDrive.rotate(-.2);
                    } else if (currentBeacon.getYRotation() > .02) {//if angle is positive, rotate clockwise
                        swerveDrive.rotate(.2);
                    } else {//beacon is parallel, continue aligning
                        if (currentBeacon.getAngle() < -.05) {//if beacon is to the left, move left(relative to the beacon, which is up relative to the robot)
                            swerveDrive.driveTowards(new Vector(0, 1), .5);
                        } else if (currentBeacon.getAngle() > .05) {//if beacon is to the right, move right ("")
                            swerveDrive.driveTowards(new Vector(0, -1), .5);
                        } else {//if beacon is centered relative to the robot, drive towards it
                            if (currentBeacon.getDistance() > 175) {//if distance is more than 175mm
                                swerveDrive.driveTowards(new Vector(1, 0), .2);
                            }else{//robot is fully aligned
                                swerveDrive.driveTowards(new Vector(1,0),0);
//                                robotState=RobotState.PressBeacon;
                                beaconsPressed++;
                                if(beaconsPressed==1){
                                    robotState=RobotState.DriveToSecondBeacon;
                                }else if(beaconsPressed==2){
                                    robotState=RobotState.Stop;
                                }
                            }
                        }
                    }
                }else{
                    swerveDrive.driveTowards(new Vector(1,1),0);
                }
                break;
            case PressBeacon:

                break;
            case DriveToSecondBeacon:
                if(!getTargets(data).contains("Legos")){
                    swerveDrive.driveTowards(new Vector(-.5,1),.5);
                }else{
                    robotState=RobotState.AlignWithBeacon;
                }
                break;
            case Stop:
                swerveDrive.driveTowards(new Vector(1,1),0);
                break;
        }//switch
        swerveDrive.update(true);
    }//loop




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
}
