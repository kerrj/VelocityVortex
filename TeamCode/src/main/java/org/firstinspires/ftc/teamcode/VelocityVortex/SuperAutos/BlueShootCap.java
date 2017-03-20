package org.firstinspires.ftc.teamcode.VelocityVortex.SuperAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CameraStuff.HistogramAnalysisThread;
import org.firstinspires.ftc.teamcode.Swerve.Core.FTCSwerve;
import org.firstinspires.ftc.teamcode.VelocityVortex.Robot;

/**
 * Created by Justin on 1/27/2017.
 */
@Autonomous
public class BlueShootCap extends Robot {
    HistogramAnalysisThread.BeaconResult beaconResult;
    enum RobotState{DriveForward,DriveSideways,Shoot,Rotate,DriveToCapBall,Stop}
    RobotState state=RobotState.DriveForward;
    boolean waitForServos=true,internalResetPosition=true;
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
        if(swerveDrive==null){
            swerveDrive=new FTCSwerve(lfa, rfa, lba, rba, lfm, rfm, lbm, rbm, lf, rf, lb, rb, 14, 14);
        }
        if(internalResetPosition){
            startGyroHeading=gyro.getHeading();
            internalResetPosition=false;
        }

        switch(state){
            case DriveForward:
                if(driveWithEncodersAndGyro(-1, 0, 0, .2, 15)){
                    state=RobotState.DriveSideways;
                    deviationHeading=gyro.getHeading()-startGyroHeading;
                }
                break;
            case DriveSideways:
                if(getRuntime()>15){
                    shootRight.setPower(AUTONOMOUS_SHOOTING_POWER);
                    shootLeft.setPower(AUTONOMOUS_SHOOTING_POWER);
                    if(driveWithHeading(0,-1,0,.2,24,startGyroHeading)){
                        state=RobotState.Shoot;
                    }
                }else{
                    swerveDrive.drive(0,-1,0,0);
                }
                break;
            case Shoot:
                if(shoot(2,AUTONOMOUS_SHOOTING_POWER)){
                    state=RobotState.Rotate;
                }
                break;
            case Rotate:
                if(turnAroundPivotPoint(0,0,.2,Direction.COUNTERCLOCKWISE,70,4)){
                    state=RobotState.DriveToCapBall;
                    sweeper.setPower(SWEEPER_OUTAKE);
                }
                break;
            case DriveToCapBall:
                if(driveWithEncoders(0,-1,0,.3,30)){
                    state=RobotState.Stop;
                    sweeper.setPower(0);
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
