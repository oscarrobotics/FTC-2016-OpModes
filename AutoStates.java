package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.AutoStates.State.*;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Oscar: AutoStates", group = "Oscar")
public class AutoStates extends BaseOp {
    public int position, encoderTarget, particlesShot, particlesLoaded, loopCounter, timesMoved;
    public double drivePower;
    public boolean doneShooting;
    public double startTime;
    public double target1;
    public double speed;
    public double direction;

    public enum State { // Ideally, these stay in order of how we use them
        STATE_INITIAL,
        STATE_MOVE_FROM_WALL,
        STATE_WAIT_FOR_FIRST_DRIVE,
        STATE_FIRST_SHOT,
        STATE_FIRST_LOAD,
        STATE_WAIT_FOR_LOAD,
        STATE_SECOND_SHOT,
        STATE_DRIVE_BACKWARDS1,
        STATE_TURN_45_1,
        STATE_DRIVE_BACKWARDS2,
        STATE_TURN_45_2,
        STATE_DRIVE_WHITE_LINE1,
        STATE_DRIVE_WHITE_LINE2,
        STATE_GO_TO_BEACON1,
        STATE_PRESSING_BEACON1,
        STATE_CENTER_WHITE_LINE1,
        STATE_DETECT_COLOR,
        STATE_PUSHING_LEFT,
        STATE_PUSHING_RIGHT,
        STATE_CENTER_WHITE_LINE2,
        STATE_GO_TO_BEACON2,
        STATE_DETECT_COLOR_2,
        STATE_PRESSING_BEACON2,
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
        beaconPress.setPosition(0.5);
        timesMoved = 0;

        // Init commands TODO: what am i?
        // init_loop();
    }

    public void loop() {
        loopCounter++;

        super.loop();
        // Telemetry block
        telemetry.addLine()
                .addData("Loop count", loopCounter)
                .addData("State", mCurrentState);

        switch (mCurrentState) {
            case STATE_INITIAL:
                // do nothing (for now)
                newState(STATE_MOVE_FROM_WALL);
                break;

            case STATE_MOVE_FROM_WALL:
                speed = 0.2;
                if (MecanumDrive(speed, rightMove(speed), rotationComp(), -1000)) {
                    newState(STATE_FIRST_SHOT);
                    MecanumDrive(0, 0, rotationComp(), 0);
                }
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
                    startTime = System.currentTimeMillis();
                    newState(STATE_WAIT_FOR_LOAD);
                }
                break;

            case STATE_WAIT_FOR_LOAD:
                if (System.currentTimeMillis() > startTime + 500)
                    newState(STATE_SECOND_SHOT);
                break;

            case STATE_SECOND_SHOT:
                if (particlesShot == 1) {
                    chrisAutoShoot();
                    particlesShot++;
                } else {
                    newState(STATE_DRIVE_BACKWARDS1);
                }
                break;

            case STATE_DRIVE_BACKWARDS1:
                speed = 0.5;
                if (MecanumDrive(speed, backwardMove(speed), rotationComp(), 2000)) {
                    newState(STATE_TURN_45_1);
                    MecanumDrive(0, 0, rotationComp(), 0);
                }


                break;

            case STATE_TURN_45_1:
                targetHeading = 317;
                MecanumDrive(0, 0, rotationComp(), 0);
                if (gyroCloseEnough(3)) {
                    newState(STATE_DRIVE_BACKWARDS2);
                }
                break;

            case STATE_DRIVE_BACKWARDS2:
                if (MecanumDrive(speed, backwardMove(speed), rotationComp(), 4000)) {
                    newState(STATE_TURN_45_2);
                    MecanumDrive(0, 0, rotationComp(), 0);
                }
                break;

            case STATE_TURN_45_2:
                targetHeading = 270;
                MecanumDrive(0, 0, rotationComp(), 0);
                if (gyroCloseEnough(3)) {
                    newState(STATE_DRIVE_WHITE_LINE1);
                }
                break;


            case STATE_DRIVE_WHITE_LINE1:
                if (odSensor.getLightDetected() >= .5) {
                    //MecanumDrive(0, 0, 0, 0);
                    newState(STATE_CENTER_WHITE_LINE1);
                }
                break;

            case STATE_CENTER_WHITE_LINE1:
                newState(STATE_GO_TO_BEACON1);
                break;

            case STATE_GO_TO_BEACON1:
                //add code to drive left
                newState(STATE_DETECT_COLOR);
                break;

            case STATE_DETECT_COLOR:
                if (redBlueSensor.blue() >= redBlueSensor.red() + 50)
                    beaconPress.setPosition(.75);
                else
                    beaconPress.setPosition(.25);
                newState(STATE_DRIVE_WHITE_LINE2);
                break;

            case STATE_DRIVE_WHITE_LINE2:
                //Drive forward code goes here
                newState(STATE_CENTER_WHITE_LINE2);

                break;

            case STATE_CENTER_WHITE_LINE2:
                newState(STATE_GO_TO_BEACON2);
                break;

            case STATE_GO_TO_BEACON2:
                //drive left code goes here
                newState(STATE_DETECT_COLOR_2);

                break;

            case STATE_DETECT_COLOR_2:
                if (redBlueSensor.blue() >= redBlueSensor.red() + 50)
                    beaconPress.setPosition(.75);
                else
                    beaconPress.setPosition(.25);
                break;


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

    public void driveSideways(int target) {
        speed = 0.5;

        //MecanumDrive(speed, direction, rotation, target);
    }

    public boolean gyroCloseEnough(double epsilon) {
        return Math.abs(currentGyroHeading - targetHeading) < epsilon;
    }
}




