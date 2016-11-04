package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;

/**
 * Created by Ultra on 11/3/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Oscar: Autonomous Linear Tank", group = "Oscar")

public class LinearAutoOp extends LinearOpMode {
    DcMotor rightFront;
    DcMotor rightBack;  //names the motors
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor shooter;
    DcMotor collector;
    Servo loader;
    long time = ElapsedTime.MILLIS_IN_NANO;
    long waitTime = (long) 750;
    public boolean isReady=false;

    @Override
    public void runOpMode() throws InterruptedException {

        shooter = hardwareMap.dcMotor.get("shooter"); //tells the robot what the hardware is calling it
        shooter.setDirection(DcMotor.Direction.FORWARD); // sets the direction

        while(!isReady){
            isSet();
        }

        rightFront = hardwareMap.dcMotor.get("rightFront"); //tells the robot what the hardware is calling it
        rightFront.setDirection(DcMotor.Direction.REVERSE); // sets the direction

        collector = hardwareMap.dcMotor.get("collector"); //tells the robot what the hardware is calling it
        collector.setDirection(DcMotor.Direction.REVERSE); // sets the direction

        rightBack = hardwareMap.dcMotor.get("rightBack");
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftFront.setDirection(DcMotor.Direction.FORWARD);

        leftBack = hardwareMap.dcMotor.get("leftBack");
        leftBack.setDirection(DcMotor.Direction.FORWARD);

        loader = hardwareMap.servo.get("loader");
        loader.setDirection(Servo.Direction.FORWARD);

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //sets the encoders to zero
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION); // sets the encoders to the right mode, tells the motor to run to the position it is given
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (opModeIsActive() && isReady) {
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


    }

    public void forward(double a) {
        final int move = (int) (a * 46.667);
        rightFront.setTargetPosition((int) rightFront.getCurrentPosition() + move);
        leftFront.setTargetPosition((int) rightFront.getCurrentPosition() + move);
        leftBack.setTargetPosition((int) rightFront.getCurrentPosition() + move);
        rightBack.setTargetPosition((int) rightFront.getCurrentPosition() + move);
        while (rightFront.getCurrentPosition() != rightFront.getTargetPosition()) {
            rightFront.setPower(.75);
            rightBack.setPower(.75);
            leftFront.setPower(.75);
            leftBack.setPower(.75);
        }
        stopRobot();

    }

    public void turnRight(double a) {
        final int move = (int) (a * 46.667 * .1351944);
        rightFront.setTargetPosition((int) rightFront.getCurrentPosition() + move);
        leftFront.setTargetPosition((int) rightFront.getCurrentPosition() - move);
        rightBack.setTargetPosition((int) rightFront.getCurrentPosition() + move);
        leftBack.setTargetPosition((int) rightFront.getCurrentPosition() - move);
        while (rightFront.getCurrentPosition() != rightFront.getTargetPosition()) {
            rightFront.setPower(.75);
            rightBack.setPower(.75);
            leftFront.setPower(-.75);
            leftBack.setPower(-.75);
        }
        stopRobot();

    }

    public void turnLeft(double a) {
        final int move = (int) (a * 46.667 * .1351944);
        rightFront.setTargetPosition((int) rightFront.getCurrentPosition() - move);
        leftFront.setTargetPosition((int) rightFront.getCurrentPosition() + move);
        rightBack.setTargetPosition((int) rightFront.getCurrentPosition() - move);
        leftBack.setTargetPosition((int) rightFront.getCurrentPosition() + move);
        while (rightFront.getCurrentPosition() != rightFront.getTargetPosition()) {
            rightFront.setPower(-.75);
            rightBack.setPower(-.75);
            leftFront.setPower(.75);
            leftBack.setPower(.75);
        }
        stopRobot();

    }


    public void autoShoot() {//shoots the catapult
        shooter.setTargetPosition(shooter.getCurrentPosition() + (560 * 6));
        if (shooter.getCurrentPosition() != shooter.getTargetPosition()) {
            shooter.setPower(.99);
        } else
            stopShooter();
    }

    public void autoLoad() throws InterruptedException {
        loader.setPosition(.15);
        loader.wait((long) 500);
        loader.setPosition(.5);
        loader.setPosition(.15);

    }

    public void stopRobot() {
        rightFront.setPower(0.0);
        rightBack.setPower(0.0);
        leftFront.setPower(0.0);
        leftBack.setPower(0.0);

    }

    public void stopShooter() {
        shooter.setPower(0.0);
    }

    public void isSet() {
        if (gamepad1.a || gamepad2.a){
            shooter.setPower(.5);
        }
        if (gamepad1.b || gamepad2.b){
            shooter.setPower(-.5);
        }
        if (gamepad1.start || gamepad2.start) { // zero encoder when 1 or 2 presses Start
            shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            isReady = true;
        }


        }





}

