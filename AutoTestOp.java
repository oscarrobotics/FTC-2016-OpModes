

/**
 * Created by Chris on 11/7/2016.
 */

package org.firstinspires.ftc.teamcode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.util.Range;

        import java.util.Timer;
/**
 * Created by Banks on 11/5/2016.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Oscar: Autonomous Test Tank", group = "Oscar")

public class AutoTestOp extends BaseOp {

    public int position, encoderTarget;
    public boolean didWeMoveYet= false;
    public boolean weDidit= false;
    @Override
    public void init() {
        super.init();

        // Init variables
        loader.setPosition(.15);
        // Init other stuff
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // zeroes encoders
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setTargetPosition((rightFront.getCurrentPosition()));
        rightBack.setTargetPosition((rightBack.getCurrentPosition())); // sets the target position of the encoders to the current position + target position
        leftFront.setTargetPosition(leftFront.getCurrentPosition());
        leftBack.setTargetPosition(leftBack.getCurrentPosition());
        buttonPress.setPosition(0.5);

        // Init commands
        setAutoRunMode();
        init_loop();
    }

    public void loop() {
        super.loop();

        telemetry.addData("ShooterEncPos", shooter.getCurrentPosition()); // telemetry for ShooterEncPos
        telemetry.update();
        if(!didWeMoveYet){
           auto2ball();
            turnLeft(90);
            turnRight(45);
            forward(18);
            //detectbutton
            //move to button
            //button press
            forward(18);
            //detectbutton
            //move to button
            //button press
            turnRight(118);
            forward(50);
            stopDriving();
            didWeMoveYet=true;
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

        while (rightFront.getCurrentPosition() != rightFront.getTargetPosition()) {
            rightFront.setPower(.75);
            rightBack.setPower(.75);
            leftFront.setPower(.75);
            leftBack.setPower(.75);
        }
        stopDriving();

    }

    public void turnRight(double a) {
        final int move = (int) (a * 44.8 * .1351944);
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
        stopDriving();

    }

    public void turnLeft(double a) {
        final int move = (int) (a * 44.8 * .1351944);
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
        stopDriving();

    }


    public void stopDriving() {
        rightFront.setPower(0.0);
        rightBack.setPower(0.0);
        leftFront.setPower(0.0);
        leftBack.setPower(0.0);

    }

    public void chrisAutoShoot() {
        shooter.setPower(1.0);
        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shooterTargetPosition -= 3360;
        do {
            shooter.setTargetPosition(shooterTargetPosition);
        } while (shooter.getCurrentPosition() >= shooterTargetPosition + 10);
    }

    public void auto2ball(){

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


    }

    public void chrisAutoLoad(){ // make sure servo is initialized to .15
        if(shooter.getCurrentPosition() >= shooterTargetPosition) {
            loader.setPosition(.15);
            do {
                loader.setPosition(.5);
            } while (loader.getPosition() != .5);
        }
    }




}
