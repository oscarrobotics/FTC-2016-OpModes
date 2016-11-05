package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.BaseOp;

import java.util.Timer;
/**
 * Created by Banks on 11/5/2016.
 */


public class AutoTest extends BaseOp {

    public int position, encoderTarget;
    public double drivePower;
    public boolean didWeDoItYet = false;
    public boolean didWeDoItAgain = false;
    public int counter;
    public boolean didWeMoveYet= false;

    @Override
    public void init() {
        super.init();

        // Init variables
        position = shooter.getCurrentPosition();
        encoderTarget = 4950; // encoder target in native Ticks
        drivePower = 1.0;

        // Init other stuff
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // zeroes encoders
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // what purpose does this serve?
        rightFront.setTargetPosition((rightFront.getCurrentPosition()));
        rightBack.setTargetPosition((rightBack.getCurrentPosition())); // sets the target position of the encoders to the current position + target position
        leftFront.setTargetPosition(leftFront.getCurrentPosition());
        leftBack.setTargetPosition(leftBack.getCurrentPosition());
        buttonPress.setPosition(0.5);

        // Init commands
        setAutoRunMode();
        runToInit();
    }

    public void loop() {
        super.loop();

        telemetry.addData("ShooterEncPos", shooter.getCurrentPosition()); // telemetry for ShooterEncPos
        /*

       if (((rightFront.getCurrentPosition() < encoderTarget) && (leftFront.getCurrentPosition()) < encoderTarget)) { // if we aren't at target, go full power
            rightFront.setPower(drivePower);
            leftFront.setPower(drivePower);
            rightBack.setPower(drivePower);
            leftBack.setPower(drivePower);
        }
        else { // once we reach target, stop
            rightFront.setPower(0.0);
            leftFront.setPower(0.0);
            rightBack.setPower(0.0);
            leftBack.setPower(0.0);

        }

        if (rightFront.getCurrentPosition() >= 4950) { // if we are at target, turn
            rightFront.setTargetPosition(rightFront.getCurrentPosition() + 1650);
            rightBack.setTargetPosition(rightBack.getCurrentPosition() + 1650);


            rightFront.setPower(1.0);
            rightBack.setPower(1.0);
            leftBack.setPower(-1.0);
            leftFront.setPower(-1.0);
        }
        */
        telemetry.addData("1", didWeDoItYet);
        telemetry.update();
        if (!didWeMoveYet) {
            forward(23.5);
            /*
            turnLeft(45);
            forward(16);

           if( !weDidit) {
            if(particlesShot == 0) {
                chrisAutoShoot();
                particlesShot++;
            }
            chrisAutoLoad();

            if(particlesShot == 1) {

                chrisAutoShoot();
                weDidit = true;
            }

        }

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
            */
            stopDriving();
            didWeMoveYet = true;
        }


        if (!didWeDoItYet) {
            int a = 1;
            shootParticle();

            while (shooter.getCurrentPosition() > -3350) {
                a = 1;
            }

            shootParticle();
            didWeDoItYet = true;
        }


        //  stopR();
    }


    public void runToInit() {
        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION); // change to position closed loop
        shooter.setPower(.5);
        if (gamepad2.a) { // increment shooter tick position by 10 while held
            shooter.setTargetPosition(shooter.getCurrentPosition() + 10);
        }
        if (gamepad2.y) { // decrement shooter tick position by 10 while held
            shooter.setTargetPosition(shooter.getCurrentPosition() - 10);
        }
        if (gamepad1.start || gamepad2.start) { // zero encoder when 1 or 2 presses Start
            shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void stopShooter() { //checks to see if the target
        shooter.setPower(0.0);
    }
    public void forward(double a) {
        final int move = (int) (a * 44.8);
        rightFront.setTargetPosition((int) rightFront.getCurrentPosition() + move);
        leftFront.setTargetPosition((int) rightFront.getCurrentPosition() + move);
        leftBack.setTargetPosition((int) rightFront.getCurrentPosition() + move);
        rightBack.setTargetPosition((int) rightFront.getCurrentPosition() + move);
        rightFront.setPower(.75);
        rightBack.setPower(.75);
        leftFront.setPower(.75);
        leftBack.setPower(.75);
        while (rightFront.getCurrentPosition() != rightFront.getTargetPosition()) {

        }
        stopDriving();

    }

    public void turnRight(double a) {
        final int move = (int) (a * 44.8 * .1351944);
        rightFront.setTargetPosition((int) rightFront.getCurrentPosition() + move);
        leftFront.setTargetPosition((int) rightFront.getCurrentPosition() - move);
        rightBack.setTargetPosition((int) rightFront.getCurrentPosition() + move);
        leftBack.setTargetPosition((int) rightFront.getCurrentPosition() - move);
        rightFront.setPower(.75);
        rightBack.setPower(.75);
        leftFront.setPower(-.75);
        leftBack.setPower(-.75);
        while (rightFront.getCurrentPosition() != rightFront.getTargetPosition()) {

        }
        stopDriving();

    }

    public void turnLeft(double a) {
        final int move = (int) (a * 44.8 * .1351944);
        rightFront.setTargetPosition((int) rightFront.getCurrentPosition() - move);
        leftFront.setTargetPosition((int) rightFront.getCurrentPosition() + move);
        rightBack.setTargetPosition((int) rightFront.getCurrentPosition() - move);
        leftBack.setTargetPosition((int) rightFront.getCurrentPosition() + move);
        rightFront.setPower(-.75);
        rightBack.setPower(-.75);
        leftFront.setPower(.75);
        leftBack.setPower(.75);
        while (rightFront.getCurrentPosition() != rightFront.getTargetPosition()) {

        }
        stopDriving();

    }


    public void stopDriving() {
        rightFront.setPower(0.0);
        rightBack.setPower(0.0);
        leftFront.setPower(0.0);
        leftBack.setPower(0.0);

    }





}



