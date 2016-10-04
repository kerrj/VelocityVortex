package org.firstinspires.ftc.teamcode.RESQ5795;

/**
 * Created by hunai on 10/3/2016.
 */

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by jboerger on 10/2/2015.
 */
public class TeleOp2 extends OpMode {

    DcMotor R;
    DcMotor L;
    DcMotor sponge;
    DcMotor lift;
    DcMotor rope;

    Servo left;
    Servo right;
    Servo belt;
    Servo plowR;
    Servo plowL;
    Servo left2;
    Servo right2;
    Servo blueClimber;
    Servo redClimber;
    Servo wheelL;
    Servo wheelR;

    double leftPosition;
    double rightPosition;
    double beltPosition;
    double plowRPosition;
    double plowLPosition;
    double left2Position;
    double right2Position;
    double blueClimberPosition;
    double redClimberPosition;
    double wheelLposition;
    double wheelRposition;

    double leftChange=.1;
    double rightChange=.1;
    double plowRChange=.05;
    double plowLChange=.05;
    double BELTChange=.1;

    final static double LEFT_MIN_RANGE  = .35;
    final static double LEFT_MAX_RANGE  = .95;
    final static double RIGHT_MIN_RANGE  = 0.24;
    final static double RIGHT_MAX_RANGE  = 0.85;
    final static double BELT_MIN_RANGE  = 0.10;
    final static double BELT_MAX_RANGE  = 0.9;
    final static double PLOWR_MAX_RANGE = .97;
    final static double PLOWR_MIN_RANGE = .75;
    final static double PLOWL_MAX_RANGE = .3;
    final static double PLOWL_MIN_RANGE = .12;
    final static double LEFT2_MIN_RANGE  = .3;
    final static double LEFT2_MAX_RANGE  = .9;
    final static double RIGHT2_MIN_RANGE  = .1;
    final static double RIGHT2_MAX_RANGE  = .8;
    final static double blueClimber_MIN_RANGE  = .95;
    final static double blueClimber_MAX_RANGE  = .3;
    final static double redClimber_MIN_RANGE  = .8;
    final static double redClimber_MAX_RANGE  = .1;
    final static double wheelL_MIN_RANGE  = 1;
    final static double wheelL_MAX_RANGE  = 0;
    final static double wheelR_MIN_RANGE  = 1;
    final static double wheelR_MAX_RANGE  = 0;

    public void init (){
        R = hardwareMap.dcMotor.get("R");
        L = hardwareMap.dcMotor.get("L");
        lift=hardwareMap.dcMotor.get("lift");
        rope=hardwareMap.dcMotor.get("rope");

        sponge=hardwareMap.dcMotor.get("sponge");
        left= hardwareMap.servo.get("left");
        right=hardwareMap.servo.get("right");
        belt=hardwareMap.servo.get("belt");
        plowR=hardwareMap.servo.get("plowR");
        plowL=hardwareMap.servo.get("plowL");
        left2= hardwareMap.servo.get("left2");
        right2=hardwareMap.servo.get("right2");
        blueClimber=hardwareMap.servo.get("blueClimber");
        redClimber=hardwareMap.servo.get("redClimber");
        wheelL=hardwareMap.servo.get("wheelL");
        wheelR=hardwareMap.servo.get("wheelR");


        leftPosition=LEFT_MAX_RANGE;
        rightPosition=RIGHT_MIN_RANGE;
        beltPosition=.5;
        plowLPosition=PLOWL_MIN_RANGE;
        plowRPosition=PLOWR_MAX_RANGE;
        left2Position=LEFT2_MAX_RANGE;
        right2Position=RIGHT2_MIN_RANGE;
        blueClimberPosition=blueClimber_MIN_RANGE;
        redClimberPosition=redClimber_MAX_RANGE;
        wheelLposition=wheelL_MIN_RANGE;
        wheelRposition=wheelR_MAX_RANGE;

        R.setDirection(DcMotor.Direction.FORWARD);
        L.setDirection(DcMotor.Direction.FORWARD);
    }


    public void loop () {
        R.setPower(gamepad1.right_stick_y);
        L.setPower(-gamepad1.left_stick_y);

        sponge.setPower(gamepad2.left_stick_y*3/4);

        if(gamepad1.y){
            lift.setPower(-.7);
        }
        else if(gamepad1.a){
            lift.setPower(.5);
        }
        else{
            lift.setPower(0);
        }

        if(gamepad1.right_bumper){
            rope.setPower(-1);
        }
        else if(gamepad1.left_bumper){
            rope.setPower(1);
        }
        else{
            rope.setPower(0);
        }

        if (gamepad1.left_trigger >= .9) {
            blueClimberPosition = blueClimber_MIN_RANGE;
        }
        if (gamepad1.right_trigger >= .9) {
            redClimberPosition = redClimber_MAX_RANGE;
        }
        if (gamepad1.left_trigger <= .3) {
            blueClimberPosition = blueClimber_MAX_RANGE;
        }
        if (gamepad1.right_trigger <= .3) {
            redClimberPosition = redClimber_MIN_RANGE;
        }


        if (gamepad2.dpad_right) {
            leftPosition += leftChange;
        }

        if (gamepad2.dpad_left) {
            leftPosition -= leftChange;
        }

        if (gamepad2.b) {
            rightPosition += rightChange;
        }
        if (gamepad2.x) {
            rightPosition -= rightChange;
        }
        if (gamepad2.left_bumper){
            BELTChange=.1;
            beltPosition -= BELTChange;
        }
        else if (gamepad2.right_bumper){
            BELTChange=.1;
            beltPosition+=BELTChange;
        }
        else{
            BELTChange = 0;
            beltPosition=.5;
        }
        if (gamepad2.a){
            plowLPosition= plowLPosition - plowLChange;
            plowRPosition= plowRPosition + plowRChange;
        }
        if (gamepad2.y){
            plowLPosition= plowLPosition + plowLChange;
            plowRPosition= plowRPosition - plowRChange;
        }

        if (gamepad2.left_trigger >= .9) {
            left2Position = LEFT2_MIN_RANGE;
        }
        if (gamepad2.right_trigger >= .9) {
            right2Position = RIGHT2_MAX_RANGE;
        }
        if (gamepad2.left_trigger <= .3) {
            left2Position = LEFT2_MAX_RANGE;
        }
        if (gamepad2.right_trigger <= .3) {
            right2Position = RIGHT2_MIN_RANGE;
        }
        if(gamepad1.dpad_up){
            wheelLposition=wheelL_MAX_RANGE;
            wheelRposition=wheelR_MIN_RANGE;
        }
        if(gamepad1.dpad_down){
            wheelLposition=wheelL_MIN_RANGE;
            wheelRposition=wheelR_MAX_RANGE;
        }


        leftPosition = Range.clip(leftPosition, LEFT_MIN_RANGE, LEFT_MAX_RANGE);
        rightPosition = Range.clip(rightPosition, RIGHT_MIN_RANGE, RIGHT_MAX_RANGE);
        beltPosition = Range.clip(beltPosition, BELT_MIN_RANGE, BELT_MAX_RANGE);
        plowLPosition = Range.clip(plowLPosition, PLOWL_MIN_RANGE, PLOWL_MAX_RANGE);
        plowRPosition = Range.clip(plowRPosition, PLOWR_MIN_RANGE, PLOWR_MAX_RANGE);
        left2Position = Range.clip(left2Position, LEFT2_MIN_RANGE, LEFT2_MAX_RANGE);
        right2Position = Range.clip(right2Position, RIGHT2_MIN_RANGE, RIGHT2_MAX_RANGE);
        // blueClimberPosition = Range.clip(blueClimberPosition, blueClimber_MIN_RANGE, blueClimber_MAX_RANGE);
        redClimberPosition = Range.clip(redClimberPosition, redClimber_MIN_RANGE, redClimber_MAX_RANGE);
//        wheelLposition = Range.clip(wheelLposition, wheelL_MIN_RANGE, wheelL_MAX_RANGE);
//        wheelRposition = Range.clip(wheelRposition, wheelR_MIN_RANGE, wheelR_MAX_RANGE);

        left.setPosition(leftPosition);
        right.setPosition(rightPosition);
        belt.setPosition(beltPosition);
        plowL.setPosition (plowLPosition);
        plowR.setPosition (plowRPosition);  //twistin dope lean and the fanta
        left2.setPosition(left2Position);
        right2.setPosition(right2Position);
        blueClimber.setPosition(blueClimberPosition);
        redClimber.setPosition(redClimberPosition);
        wheelL.setPosition(wheelLposition);
        wheelR.setPosition(wheelRposition);

        telemetry.addData("S Right: ", gamepad1.right_stick_y);
        telemetry.addData("S Left: ", gamepad1.left_stick_y);
        telemetry.addData("right2",gamepad2.right_trigger);
        telemetry.addData("left2", gamepad2.left_trigger);
    }

    public void stop () {

    }
}


