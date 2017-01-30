package org.firstinspires.ftc.teamcode.VelocityVortex;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.OpMode;

        import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
        import org.firstinspires.ftc.teamcode.CameraStuff.FTCTarget;
        import org.firstinspires.ftc.teamcode.CameraStuff.FTCVuforia;
        import org.firstinspires.ftc.teamcode.CameraStuff.HistogramAnalysisThread;
        import org.firstinspires.ftc.teamcode.Swerve.Core.Vector;

        import java.sql.ResultSet;
        import java.util.Currency;
        import java.util.HashMap;

/**
 * Created by Justin on 1/27/2017.
 */
@Autonomous
public class DankMemesAutoBlue extends Robot {
    HistogramAnalysisThread.BeaconResult beaconResult;
    enum RobotState{DriveForward,Shoot,RotateToFirstBeacon,LineUpAndPressBeacon,Stop}
    RobotState state=RobotState.LineUpAndPressBeacon;
    private int beaconsPressed=0;
    private double rotateRadius=18;


    @Override
    public void init() {
        super.init();
        initAutonomous();
    }

    public void init_loop(){
        swerveDrive.drive(1,0,0,0);
        swerveDrive.update(true,15,true);
    }
    @Override
    public void loop() {
        super.loop();
        if(gyro.isCalibrating()){
            swerveDrive.drive(1,0,0,0);
            swerveDrive.update(true,15,false);
            return;
        }
        beaconResult=thread.getBeaconResult();
        HashMap<String, double[]> data = vuforia.getVuforiaData();
        FTCTarget wheels = new FTCTarget();
        FTCTarget legos=new FTCTarget();
        try {
            if (data.containsKey("Wheels")) {
                wheels = new FTCTarget(data, "Wheels");
            }
            if(data.containsKey("Legos")){
                legos=new FTCTarget(data,"Legos");
            }
        }catch(NullPointerException e){
            e.printStackTrace();
        }
        FTCTarget currentBeacon=new FTCTarget();
        if(beaconsPressed==0) {
            currentBeacon = wheels;
        }else if(beaconsPressed==1){
            currentBeacon=legos;
        }


        switch(state){
            case DriveForward:
                if(driveWithEncoders(1,0,0,.2,15)){
                    state=RobotState.Shoot;
                    swerveDrive.setPivotPoint(rotateRadius,0);
                    swerveDrive.drive(0,0,.4,0);
                    swerveDrive.setPivotPoint(0,0);
                }
                break;
            case Shoot:
                if(shoot(2,.65)){
                    state=RobotState.RotateToFirstBeacon;
                }
                break;
            case RotateToFirstBeacon:
                if(turnAroundPivotPoint(rotateRadius,0,.4,90,Direction.COUNTERCLOCKWISE,4)){
                    state=RobotState.LineUpAndPressBeacon;
                }
                break;
            case LineUpAndPressBeacon:
                if(alignWithAndPushCurrentBeacon(currentBeacon,beaconResult,Side.BLUE)){
                    state=RobotState.Stop;
                }
                break;
            case Stop:
                swerveDrive.drive(0,0,1,0);
                break;
        }


        telemetry.addData("BeaconResult",beaconResult);
        telemetry.addData("Confidence",thread.getConfidence());
        swerveDrive.update(true,15,true);
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
