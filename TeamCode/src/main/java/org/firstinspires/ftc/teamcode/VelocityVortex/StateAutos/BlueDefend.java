package org.firstinspires.ftc.teamcode.VelocityVortex.StateAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CameraStuff.HistogramAnalysisThread;
import org.firstinspires.ftc.teamcode.VelocityVortex.Robot;

/**
 * Created by Justin on 1/27/2017.
 */
@Autonomous
public class BlueDefend extends Robot {
    HistogramAnalysisThread.BeaconResult beaconResult;
    enum RobotState{DriveForward,Shoot,RotateToFirstBeacon,DriveToSecondBeacon,PressFirstBeacon,
        PressSecondBeacon,DriveToDefend,DriveToCapBall,Defend,Stop,AlignToShoot}
    RobotState state=RobotState.DriveForward;
    boolean waitForServos=true, internalResetPosition=true;
    double extraDistance=0,startGyroHeading,deviationHeading;


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
        }
        beaconResult=thread.getBeaconResult();

        switch(state){
            //old version
            case DriveForward:
                shootRight.setPower(AUTONOMOUS_SHOOTING_POWER);
                shootLeft.setPower(AUTONOMOUS_SHOOTING_POWER);
                if(driveWithEncodersAndGyro(-1, 0, 0, .2, 15)){
                    state=RobotState.Shoot;
                    deviationHeading=gyro.getHeading()-startGyroHeading;
                }
                break;
            case Shoot:
                swerveDrive.setPivotPoint(-20,0);
                swerveDrive.drive(0,0,1,0);
                if(shoot(2,AUTONOMOUS_SHOOTING_POWER)){
                    state=RobotState.RotateToFirstBeacon;
                    swerveDrive.setPivotPoint(0,0);
                }
                break;

            case RotateToFirstBeacon:
                if(turnAroundPivotPoint(-20, 0, .5,Direction.COUNTERCLOCKWISE, 90+(int)deviationHeading, 4)){
                    state=RobotState.PressFirstBeacon;
                }
                break;

            case PressFirstBeacon:
                if(beaconResult== HistogramAnalysisThread.BeaconResult.RED_LEFT){
                    extraDistance=5;
                }
                if(alignWithAndPushBeacon("Wheels", beaconResult, Side.BLUE,.25,1)){
                    state=RobotState.DriveToSecondBeacon;
                    buttonWheel.setPosition(WHEEL_IN);
                }
                break;

            case DriveToSecondBeacon:
                if(driveWithHeading(-.6,-1,0,.5,35+extraDistance,startGyroHeading-90)){
                    state=RobotState.PressSecondBeacon;
                }
                break;

            case PressSecondBeacon:
                buttonWheel.setPosition(WHEEL_OUT);
                if(alignWithAndPushBeacon("Legos", beaconResult, Side.BLUE,.25,1)){
                    state=RobotState.DriveToDefend;
                    buttonWheel.setPosition(WHEEL_IN);
                }
                break;
            case DriveToDefend:
                if(driveWithHeading(-1, -.2, 0, .3, 50,startGyroHeading-90)){
                    state=RobotState.Stop;
                    waitForServos=true;
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
