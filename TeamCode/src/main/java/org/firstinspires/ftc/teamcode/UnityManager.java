//package org.firstinspires.ftc.teamcode;
//
//import android.content.res.Configuration;
//import android.util.Log;
//import android.widget.FrameLayout;
////a
//
//import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
//
///**
// * Created by Justin on 9/9/2016.
// */
//public class UnityManager implements FtcRobotControllerActivity.UIListener{
//    UnityPlayer mUnityPlayer;
//
//    private Runnable initRunnable=new Runnable() {
//        @Override
//        public void run() {
//            if(mUnityPlayer==null) {
//                mUnityPlayer = new UnityPlayer(FtcRobotControllerActivity.getActivity());
//                int glesMode = mUnityPlayer.getSettings().getInt("gles_mode", 1);
//                mUnityPlayer.init(glesMode, false);
//                FrameLayout f = (FrameLayout) FtcRobotControllerActivity.getActivity().findViewById(com.qualcomm.ftcrobotcontroller.R.id.frameLayout);
//                f.addView(mUnityPlayer);
//            }
//            mUnityPlayer.requestFocus();
//            mUnityPlayer.resume();
//        }
//    };
//    public void init(){
//        FtcRobotControllerActivity.getActivity().runOnUiThread(initRunnable);
//    }
//    private Runnable stopRunnable=new Runnable() {
//        @Override
//        public void run() {
//            mUnityPlayer.pause();
//        }
//    };
//    public void stop(){
//        FtcRobotControllerActivity.getActivity().runOnUiThread(stopRunnable);
//    }
//
//    @Override
//    public void onWindowFocusChanged(boolean focus) {
//        Log.d("Focus", "changed");
//        mUnityPlayer.windowFocusChanged(focus);
//    }
//    @Override
//    public void onConfigurationChanged(Configuration c){
//        Log.d("Configuration","Changed");
//        mUnityPlayer.configurationChanged(c);
//    }
//}
