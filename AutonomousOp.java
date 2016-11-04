package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import java.util.Timer;


public class AutonomousOp extends BaseOp {@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Oscar: Autonomous Tank", group = "Oscar")

Timer time= new Timer();
    Timer varTime= new Timer();


    // public int rightFrontEncoderTarget= 5000;
    public int position = shooter.getCurrentPosition();
    public int tp = rightFront.getCurrentPosition();
    // if time - varTime is > than wait then run again, set varTime to new Time
    @Override
    public void init() {
        super.init();
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //sets the encoders to zero
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setAutoRunMode();
    }

    public void loop() {
        super.loop();
        forward(23.5);
        turnLeft(45);
        forward(16);
        autoShoot();
        autoLoad();
        autoShoot();
        forward(28);
        turnRight(60);
        forward(2);
        //detectbuton
        forward(49);
        //detectbutton
        turnRight(90);
        forward(12);
        turnRight(90);
        forward(81);
        turnRight(45);
        forward(-12);
        stopRobot();



    }
public void forward(double a) {
    final int move= (int)(a *46.667);
rightFront.setTargetPosition((int) rightFront.getCurrentPosition()+ move) ;
    leftFront.setTargetPosition((int) rightFront.getCurrentPosition()+ move) ;
    leftBack.setTargetPosition((int) rightFront.getCurrentPosition()+ move)  ;
    rightBack.setTargetPosition((int) rightFront.getCurrentPosition()+ move);
    if( rightFront.getCurrentPosition()!= rightFront.getTargetPosition())
    {
        rightFront.setPower(.75);
        rightBack.setPower(.75);
        leftFront.setPower(.75);
        leftBack.setPower(.75);

    }
    else{
        rightFront.setPower(0.0);
        rightBack.setPower(0.0);
        leftFront.setPower(0.0);
        leftBack.setPower(0.0);


    }
}
    public void turnRight(double a ){
        final int move= (int)(a *46.667 * .1351944);
        rightFront.setTargetPosition((int) rightFront.getCurrentPosition()+ move) ;
        leftFront.setTargetPosition((int) rightFront.getCurrentPosition()- move) ;
        rightBack.setTargetPosition((int) rightFront.getCurrentPosition() + move) ;
        leftBack.setTargetPosition((int) rightFront.getCurrentPosition()- move) ;
        if( rightFront.getCurrentPosition()!= rightFront.getTargetPosition())
        {
            rightFront.setPower(.75);
            rightBack.setPower(.75);
            leftFront.setPower(-.75);
            leftBack.setPower(-.75);

        }
        else{
            rightFront.setPower(0.0);
            rightBack.setPower(0.0);
            leftFront.setPower(0.0);
            leftBack.setPower(0.0);


        }
    }
    public void turnLeft(double a ){
        final int move= (int)(a *46.667 * .1351944);
        rightFront.setTargetPosition((int) rightFront.getCurrentPosition()- move) ;
        leftFront.setTargetPosition((int) rightFront.getCurrentPosition()+ move) ;
        rightBack.setTargetPosition((int) rightFront.getCurrentPosition()- move) ;
        leftBack.setTargetPosition((int) rightFront.getCurrentPosition()+ move) ;
        if( rightFront.getCurrentPosition()!= rightFront.getTargetPosition())
        {
            rightFront.setPower(-.75);
            rightBack.setPower(-.75);
            leftFront.setPower(.75);
            leftBack.setPower(.75);

        }
        else{
            rightFront.setPower(0.0);
            rightBack.setPower(0.0);
            leftFront.setPower(0.0);
            leftBack.setPower(0.0);


        }
    }





    public void autoShoot() {//shoots the catapult
        shooter.setTargetPosition(shooter.getCurrentPosition() + 900);
        if (shooter.getCurrentPosition() != shooter.getTargetPosition()) {
            shooter.setPower(.5);
        }
        shooter.setTargetPosition(shooter.getCurrentPosition() + 360);
        if (shooter.getCurrentPosition() != shooter.getTargetPosition()) {
            shooter.setPower(.5);
        }
        stopShooter();
    }
    public void autoLoad(){
        loader.setPosition(.15);
        loader.setPosition(.5);
        loader.setPosition(.15);

    }
    public void stopRobot(){
        rightFront.setPower(0.0);
        rightBack.setPower(0.0);
        leftFront.setPower(0.0);
        leftBack.setPower(0.0);

    }



    public void stopShooter() {
        shooter.setPower(0.0);
    }

    public void runToInit(){
        if (gamepad1.a || gamepad2.a){
            shooter.setPower(.5);
        }
        if (gamepad1.b || gamepad2.b){
            shooter.setPower(-.5);
        }
        if (gamepad1.start || gamepad2.start){ // zero encoder when 1 or 2 presses Start
            shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            isSet
        }
    }
}




