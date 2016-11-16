package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.Auto3Ball.State.*;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Oscar: Auto 3 Ball", group = "Oscar")
public class Auto3Ball extends BaseOp {
    public int position, encoderTarget, particlesShot, particlesLoaded, loopCounter;
    public double drivePower;
    public boolean doneShooting;

    public enum State { // Ideally, these stay in order of how we use them
        STATE_INITIAL,
        STATE_FIRST_SHOT,
        STATE_FIRST_LOAD,
        STATE_SECOND_SHOT,
        STATE_DRIVE_WHITE_LINE1,
        STATE_DRIVE_WHITE_LINE2,
        STATE_GO_TO_BEACON1,
        STATE_PRESS_BUTTON,
        STATE_CENER_WHITE_LINE1,
        STATE_STOP
    }

    private ElapsedTime mStateTime = new ElapsedTime();  // Time into current state
    private State mCurrentState;

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        resetStartTime();
        newState(State.STATE_INITIAL);
    }


    @Override
    public void init() {
        super.init();


        // Init variables
        position = shooter.getCurrentPosition();
        encoderTarget = 4950; // encoder target in native Ticks
        drivePower = 1.0;
        loader.setPosition(.15); // added to resolve loader servo not initialized properly - george
        particlesShot = 0;
        doneShooting = false;
        loopCounter = 0;

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
        beaconPress.setPosition(0.5);

        // Init commands
        setAutoRunMode();
        init_loop();
    }

    public void loop() {
        loopCounter++;

        super.loop();

        // Telemetry block
        telemetry.addData("0", "Loop count " + loopCounter);
        telemetry.update(); // update telemetry

        switch (mCurrentState) {
            case STATE_INITIAL:
                // do nothing (for now)
                newState(STATE_FIRST_SHOT);
                break;

            case STATE_FIRST_SHOT:
                if (particlesShot == 0) {
                    chrisAutoShoot();
                    particlesShot++;
                } else {
                    newState(STATE_FIRST_LOAD);
                }
                break;

            case STATE_FIRST_LOAD:
                if (particlesLoaded == 0) {
                    chrisAutoLoad();
                    particlesLoaded++;
                } else {
                    newState(STATE_SECOND_SHOT);
                }
                break;

            case STATE_SECOND_SHOT:
                if (particlesShot == 1) {
                    chrisAutoShoot();
                } else {
                    newState(STATE_DRIVE_WHITE_LINE1);
                    odSensor.enableLed(true);
                    //Drive to line here
                }
                break;
            case STATE_DRIVE_WHITE_LINE1:
                if (odSensor.getLightDetected() >= .5) {
                    stopDriving();
                    //Add drive right code
                    newState(STATE_GO_TO_BEACON1);
                }
        }
    }

    public void chrisAutoShoot() {
        shooter.setPower(1.0);
        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shooterTargetPosition -= 3360;
        do {
            shooter.setTargetPosition(shooterTargetPosition);
        } while (shooter.getCurrentPosition() >= shooterTargetPosition + 10);
    }


    public void chrisAutoLoad() { // make sure servo is initialized to .15
        do {
            loader.setPosition(.5);
        }
        while (loader.getPosition() != .5);
    }

    private void newState(State newState) {
        // Reset the state time, and then change to next state.
        mStateTime.reset();
        mCurrentState = newState;
    }


}




