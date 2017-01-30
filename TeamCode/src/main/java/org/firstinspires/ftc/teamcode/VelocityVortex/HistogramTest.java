package org.firstinspires.ftc.teamcode.VelocityVortex;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.teamcode.CameraStuff.FTCVuforia;
import org.firstinspires.ftc.teamcode.CameraStuff.HistogramAnalysisThread;

/**
 * Created by Justin on 1/27/2017.
 */
@Autonomous
public class HistogramTest extends OpMode {
    HistogramAnalysisThread thread;
    FTCVuforia vuforia;
    HistogramAnalysisThread.BeaconResult beaconResult;

    @Override
    public void init() {
        vuforia=new FTCVuforia(FtcRobotControllerActivity.getActivity());
        vuforia.addTrackables("FTC_2016-17.xml");
        vuforia.initVuforia();
        thread=new HistogramAnalysisThread(vuforia);
        thread.start();
    }

    @Override
    public void loop() {
        beaconResult=thread.getBeaconResult();
        telemetry.addData("BeaconResult",beaconResult);
        telemetry.addData("Confidence",thread.getConfidence());
    }

    public void stop(){
        thread.kill();
        try {
            vuforia.destroy();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
