package org.firstinspires.ftc.teamcode.VelocityVortex;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.BatteryChecker;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.teamcode.CameraStuff.FTCTarget;
import org.firstinspires.ftc.teamcode.CameraStuff.FTCVuforia;

import java.util.ArrayList;
import java.util.HashMap;

/**
 * Created by Justin on 10/7/2016.
 */
@Autonomous
public class BlueSwerveVuforiaAuto extends Robot {

    //---------------------------------------------------------------------------------------
    //the first beacon has the WHEELS image target, and the second has the LEGOS image target
    //---------------------------------------------------------------------------------------


    private boolean resetPosition=true;

    private enum RobotState{//list states here
        DriveToFirstBeacon,AlignWithBeacon,PressBeacon,DriveToSecondBeacon,Stop
    }

    private RobotState robotState=RobotState.DriveToFirstBeacon;//initialize start state here

    private FTCVuforia vuforia;

    private int beaconsPressed=0;

//    private FTCCamera ftcCamera;
//    private RenderScript renderScript;
//    private ScriptC_colorsplit colorsplit;
//    private Allocation allocationOut;


    @Override
    public void init() {
        super.init();

        vuforia=new FTCVuforia(FtcRobotControllerActivity.getActivity());
        vuforia.initVuforia();
        vuforia.addTrackables("FTC_2016-17.xml");
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
    public void loop() {
        //first grab an instance of FTCTarget for each target we care about: Wheels and Legos
        HashMap<String, double[]> data = vuforia.getVuforiaData();
        FTCTarget wheels = new FTCTarget();
        FTCTarget legos = new FTCTarget();
        try {
            if (getTargets(data).contains("Wheels")) {
                wheels = new FTCTarget(data, "Wheels");
                telemetry.addData("Wheels", "ok");
            } else {
                telemetry.addData("Wheels", "no");
            }
            if (getTargets(data).contains("Legos")) {
                legos = new FTCTarget(data, "Legos");
            }
        }catch(NullPointerException e){
            e.printStackTrace();
        }

        switch(robotState){
            case DriveToFirstBeacon:
                double DISTANCE=40;
                double scale=swerveDrive.getLinearInchesTravelled()/DISTANCE;
                if(scale>1)scale=1;
                if(!getTargets(data).contains("Wheels")) {
                    swerveDrive.drive(1,-1,0, 1-scale);
                }else{
                    resetPosition=true;
                    robotState=RobotState.AlignWithBeacon;
                }
                break;
            case AlignWithBeacon:
                double Y_ROTATION_TOLERANCE=5;
                double ANGLE_TOLERANCE=10;

                //first initialize currentBeacon with the correct target depending on how many we've pressed
                FTCTarget currentBeacon;
                if(beaconsPressed==0){
                    currentBeacon=wheels;
                }else{
                    currentBeacon=legos;
                }

                if(currentBeacon.isFound()) {//if the target is available
                    //align to be mostly parallel with the beacon
                    if (currentBeacon.getYRotation() < -Math.toRadians(Y_ROTATION_TOLERANCE)) {//if angle is negative, rotate counterclockwise
                        swerveDrive.drive(0,0,-.4,.2);
                    } else if (currentBeacon.getYRotation() > Math.toRadians(Y_ROTATION_TOLERANCE)) {//if angle is positive, rotate clockwise
                        swerveDrive.drive(0,0,.4,.2);

                    } else {//beacon is parallel, continue aligning
                        if (currentBeacon.getAngle() < -Math.toRadians(ANGLE_TOLERANCE)) {//if beacon is to the left, move left(relative to the beacon, which is up relative to the robot)
                            swerveDrive.drive(0,-1,0, .1);
                        } else if (currentBeacon.getAngle() > Math.toRadians(ANGLE_TOLERANCE)) {//if beacon is to the right, move right ("")
                            swerveDrive.drive(0,1,0, .1);

                        } else {//if beacon is centered relative to the robot, drive towards it
                            if (currentBeacon.getDistance() > 150) {//if distance is more than 150mm
                                swerveDrive.drive(1,0,0, .1);

                            }else{//robot is fully aligned
                                swerveDrive.stop();
//                                robotState=RobotState.PressBeacon;
                                //temporary code which should go in pressbeacon instead
                                beaconsPressed++;
                                if(beaconsPressed==1){
                                    robotState=RobotState.DriveToSecondBeacon;
                                    resetPosition=true;
                                }else if(beaconsPressed==2){
                                    robotState=RobotState.Stop;
                                }
                            }
                        }
                    }
                }else{
                    swerveDrive.stop();
                }
                break;
            case PressBeacon:

                break;
            case DriveToSecondBeacon:
                if(resetPosition){
                    swerveDrive.resetPosition();
                    resetPosition=false;
                }
                DISTANCE=40;
                scale=swerveDrive.getLinearInchesTravelled()/DISTANCE;
                if(scale>1)scale=1;
                if(!getTargets(data).contains("Legos")){
                    swerveDrive.drive(-.1,-1,0,1-scale);
                }else{
                    robotState=RobotState.AlignWithBeacon;
                }
                break;


            case Stop:
                swerveDrive.stop();
                break;
        }//switch
        swerveDrive.update(true,10);
    }//loop

    @Override
    public void stop(){
        try {
            vuforia.destroy();
        } catch (Exception e) {
            e.printStackTrace();
        }
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
}
