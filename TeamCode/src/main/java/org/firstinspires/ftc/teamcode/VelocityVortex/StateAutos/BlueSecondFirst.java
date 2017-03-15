package org.firstinspires.ftc.teamcode.VelocityVortex.StateAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.CameraStuff.HistogramAnalysisThread;
import org.firstinspires.ftc.teamcode.Swerve.Core.FTCSwerve;
import org.firstinspires.ftc.teamcode.VelocityVortex.Robot;

/**
 * Created by Justin on 1/27/2017.
 */
@Autonomous
@Disabled
public class BlueSecondFirst extends Robot {
    HistogramAnalysisThread.BeaconResult beaconResult;
    enum RobotState{DriveForward,Shoot,RotateToFirstBeacon,DriveToSecondBeacon,PressFirstBeacon,
        PressSecondBeacon,DriveToDefend,DriveToCapBall,Defend,Stop,AlignToShoot,DrivetoCapBall2,RotateToCapBall,DriveToFirstBeacon}
    RobotState state=RobotState.DriveForward;
    boolean waitForServos=true, internalResetPosition=true;
    double extraDistance=0,startGyroHeading,deviationHeading;


    @Override
    public void init() {
        initAutonomous();
    }

    public void init_loop(){
        if(!gyro.isCalibrating()) {
            if(swerveDrive==null){
                swerveDrive=new FTCSwerve(lfa, rfa, lba, rba, lfm, rfm, lbm, rbm, lf, rf, lb, rb, 14, 14);
            }
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
                if(driveWithEncodersAndGyro(-1, 0, 0, .3, 15)){
                    state=RobotState.Shoot;
                    deviationHeading=gyro.getHeading()-startGyroHeading;
                }
                break;
            case Shoot:
                swerveDrive.setPivotPoint(-25,0);
                swerveDrive.drive(0,0,1,0);
                if(shoot(2,AUTONOMOUS_SHOOTING_POWER)){
                    state=RobotState.RotateToFirstBeacon;
                    swerveDrive.setPivotPoint(0,0);
                }
                break;

            case RotateToFirstBeacon:
                if(turnAroundPivotPoint(-25, 0, .7,Direction.COUNTERCLOCKWISE, 135+(int)deviationHeading, 4)){
                    state=RobotState.PressSecondBeacon;
                }
                break;
            case PressSecondBeacon:
                if(alignWithAndPushBeacon("Legos",beaconResult,Side.BLUE,.2,2,true)){
                    state=RobotState.DriveToFirstBeacon;
                }
                break;

            case DriveToFirstBeacon:
                if(driveWithHeading(-.7,1,0,.7,40,startGyroHeading-90)){
                    state=RobotState.PressFirstBeacon;
                }
                break;
            case PressFirstBeacon:
                if(alignWithAndPushBeacon("Wheels",beaconResult,Side.BLUE,.275,1,true)){
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
