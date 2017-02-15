package org.firstinspires.ftc.teamcode.VelocityVortex.StateAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CameraStuff.HistogramAnalysisThread;
import org.firstinspires.ftc.teamcode.VelocityVortex.Robot;

/**
 * Created by Justin on 1/27/2017.
 */
@Autonomous
public class RedSlow extends Robot {
    HistogramAnalysisThread.BeaconResult beaconResult;
    enum RobotState{DriveForward,Shoot,RotateToFirstBeacon,DriveToSecondBeacon,PressFirstBeacon,PressSecondBeacon,
        DriveToDefend,Defend,DriveToCapBall,Stop,AlignToShoot}
    RobotState state=RobotState.DriveForward;
    boolean waitForServos=true,internalResetPosition=true;
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
            case DriveForward:
                shootRight.setPower(.65);
                shootLeft.setPower(.65);
                if(driveWithEncoders(-1,0,0,.2,15)){
                    state=RobotState.Shoot;
                    deviationHeading=gyro.getHeading()-startGyroHeading;
                }
                break;
            case Shoot:
                if(shoot(2,.65)){
                    state=RobotState.RotateToFirstBeacon;
                }
                break;
            case RotateToFirstBeacon:
                if(turnAroundPivotPoint(-20,0,.4,90+(int)deviationHeading,Direction.CLOCKWISE,4)){
                    state=RobotState.PressFirstBeacon;
                }
//            case DriveForward:
//                if(driveWithEncoders(.4,1,0,.3,35)){
//                    state=RobotState.AlignToShoot;
//                    waitForServos=true;
//                }
//                break;
//            case AlignToShoot:
//                shootLeft.setPower(.65);
//                shootRight.setPower(.65);
//                if(alignToShoot("Tools")){
//                    state=RobotState.Shoot;
//                    swerveDrive.drive(1,0,0,0);
//                }
//                break;
//            case Shoot:
//                if(shoot(2,.65)){
//                    state=RobotState.PressFirstBeacon;
//                }
//                break;

            case PressFirstBeacon:
                if(beaconResult== HistogramAnalysisThread.BeaconResult.RED_LEFT){
                    extraDistance=5;
                }
                if(alignWithAndPushBeacon("Tools", beaconResult, Side.RED,.25,1)){
                    state=RobotState.DriveToSecondBeacon;
                    buttonWheel.setPosition(WHEEL_IN);
                }
                break;

            case DriveToSecondBeacon:
                if(driveWithHeading(-.6,1,0,.4,35+extraDistance,startGyroHeading-90)){
                    state=RobotState.PressSecondBeacon;
                }
                break;

            case PressSecondBeacon:
                buttonWheel.setPosition(WHEEL_OUT);
                if(alignWithAndPushBeacon("Gears", beaconResult, Side.RED,.25,1)){
                    state=RobotState.DriveToCapBall;
                    buttonWheel.setPosition(WHEEL_IN);
                }
                break;
            case DriveToCapBall:
                if(driveWithHeading(-1,-1,0,.4,50,startGyroHeading-90)){
                    state=RobotState.Stop;
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
