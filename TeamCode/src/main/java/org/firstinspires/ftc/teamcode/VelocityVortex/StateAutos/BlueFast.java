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
public class BlueFast extends Robot {
    HistogramAnalysisThread.BeaconResult beaconResult;
    enum RobotState{DriveForward,Shoot,RotateToFirstBeacon,DriveToSecondBeacon,PressFirstBeacon,PressSecondBeacon,DriveToDefend,Defend,Stop}
    RobotState state=RobotState.DriveForward;
    private double extraDistance=0;
    boolean waitForServos=true,internalResetPosition=true;
    long bookKeepingTime;


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
            swerveDrive.drive(.4, -1, 0, 0);
            swerveDrive.update(true, 15, false);
        }
    }
    @Override
    public void loop() {
        super.loop();
        if(gyro.isCalibrating()){
            return;
        }
        beaconResult=thread.getBeaconResult();

        switch(state){
            case DriveForward:
                shootRight.setPower(AUTONOMOUS_SHOOTING_POWER);
                shootLeft.setPower(AUTONOMOUS_SHOOTING_POWER);
                if(swerveDrive.getLinearInchesTravelled()>AUTONOMOUS_SHOOT_DRIVE_DISTANCE){
                    shootServo.setPosition(SHOOTER_UP);
                    if(internalResetPosition){
                        bookKeepingTime=System.currentTimeMillis();
                        internalResetPosition=false;
                    }
                    if(System.currentTimeMillis()-bookKeepingTime>SHOOTER_MOVE_TIME){
                        shootServo.setPosition(SHOOTER_DOWN);
                    }
                    if(System.currentTimeMillis()-bookKeepingTime>SHOOTER_MOVE_TIME+SHOOTER_DELAY_TIME){
                        shootServo.setPosition(SHOOTER_UP);
                    }
                }
                if(driveWithEncodersAndGyro(.4, -1, 0, 1, 45)){
                    waitForServos=true;
                    state=RobotState.PressFirstBeacon;
                }
                break;

            case PressFirstBeacon:
                if(System.currentTimeMillis()-bookKeepingTime>SHOOTER_DELAY_TIME+SHOOTER_MOVE_TIME){
                    shootServo.setPosition(SHOOTER_UP);
                }
                if(beaconResult== HistogramAnalysisThread.BeaconResult.RED_LEFT){
                    extraDistance=5;
                }
                if(alignWithAndPushBeacon("Wheels", beaconResult, Side.BLUE,.3,1,true)){
                    state=RobotState.DriveToSecondBeacon;
                    buttonWheel.setPosition(WHEEL_IN);
                    shootLeft.setPower(0);
                    shootRight.setPower(0);
                }
                break;

            case DriveToSecondBeacon:
                if(driveWithEncodersAndGyro(-.4, -1, -.1, .4, 25+extraDistance)){
                    state=RobotState.PressSecondBeacon;
                }
                break;

            case PressSecondBeacon:
                buttonWheel.setPosition(WHEEL_OUT);
                if(alignWithAndPushBeacon("Legos", beaconResult, Side.BLUE,.2,2,false)){
                    state=RobotState.DriveToDefend;
                    buttonWheel.setPosition(WHEEL_IN);
                }
                break;
            case DriveToDefend:
                if(driveWithEncoders(-1,-.4,0,.4,15)){
                    state=RobotState.Defend;
                }
                break;
            case Defend:
                if(driveWithEncoders(-1,0,0,.4,20)){
                    state=RobotState.Stop;
                }
                break;

            case Stop:
                swerveDrive.drive(0,0,-1,0);
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
