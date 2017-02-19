package org.firstinspires.ftc.teamcode.VelocityVortex.StateAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CameraStuff.HistogramAnalysisThread;
import org.firstinspires.ftc.teamcode.VelocityVortex.Robot;

/**
 * Created by Justin on 1/27/2017.
 */
@Autonomous
public class RedAperture extends Robot {
    HistogramAnalysisThread.BeaconResult beaconResult;
    enum RobotState{DriveForward,Shoot,RotateToFirstBeacon,DriveToSecondBeacon,PressFirstBeacon,
        PressSecondBeacon,DriveToDefend,DriveToCapBall,Defend,Stop,AlignToShoot}
    RobotState state=RobotState.DriveForward;
    boolean waitForServos=true, internalResetPosition=true;
    double extraDistance=0,startGyroHeading,deviationHeading;
    long startTime;


    @Override
    public void init() {
        super.init();
        initAutonomous();
    }

    public void init_loop(){
        if(!gyro.isCalibrating()) {
            swerveDrive.refreshValues();
            swerveDrive.drive(-1, 0, 0, 0);
            swerveDrive.update(true, 15, false);
        }
    }
    @Override
    public void loop() {
        super.loop();

        if(gyro.isCalibrating()){
            return;
        }
        if(internalResetPosition){
            startGyroHeading=gyro.getHeading();
            internalResetPosition=false;
            startTime=System.currentTimeMillis();
        }
        beaconResult=thread.getBeaconResult();

        switch(state){
            //old version
            case DriveForward:
                shootRight.setPower(.6);
                shootLeft.setPower(.6);
                if(driveWithEncodersAndGyro(-1, 0, 0, .2, 15)){
                    state=RobotState.Shoot;
                    deviationHeading=gyro.getHeading()-startGyroHeading;
                }
                break;
            case Shoot:
                swerveDrive.setPivotPoint(-30,0);
                swerveDrive.drive(0,0,1,0);
                if(shoot(1,.67)){
                    state=RobotState.RotateToFirstBeacon;
                    swerveDrive.setPivotPoint(0,0);
                }
                break;

            case RotateToFirstBeacon:
                if(turnAroundPivotPoint(-30, 0, .4,Direction.CLOCKWISE, 135-(int)deviationHeading, 4)){
                    state=RobotState.DriveToSecondBeacon;
                }
                break;

            case DriveToSecondBeacon:
                if(driveWithEncoders(1,0,0,.2,10)){
                    state=RobotState.PressSecondBeacon;
                }
                break;

            case PressSecondBeacon:
                if(alignWithAndPushBeacon("Tools",beaconResult,Side.RED,.2,2)){
                    state=RobotState.DriveToDefend;
                }
                break;
            case DriveToDefend:
                if(System.currentTimeMillis()-startTime<10000){
                    swerveDrive.drive(-1,-.2,0,0);
                }else {
                    if(driveWithHeading(-1, .2, 0, .3, 50, startGyroHeading + 90)) {
                        state = RobotState.Stop;
                        waitForServos = true;
                    }
                }
                break;
            case Stop:
                swerveDrive.drive(0,0,1,0);
                break;
        }


        telemetry.addData("BeaconResult",beaconResult);
        telemetry.addData("Confidence",thread.getConfidence());
        telemetry.addData("State",state);
        swerveDrive.update(waitForServos,30,true);
    }

    public void stop(){
        super.stop();
        thread.kill();
        try {
            vuforia.destroy();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

}
