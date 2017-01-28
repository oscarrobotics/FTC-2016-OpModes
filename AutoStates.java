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
    public int whiteLineFirstEdge;
    public int whiteLineBackEdge;
    public int distanceWhiteLineCenter;


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
        STATE_MOVE_LEFT,
        STATE_DRIVE_FOR_BEACON1,
        STATE_DRIVE_IN_BETWEEN_BEACONS,
        STATE_DRIVE_FOR_BEACON2,
        STATE_CAP_DRIVE1,
        STATE_CAP_TURN1,
        STATE_CAP_DRIVE2,
        STATE_STOP
    }

    private ElapsedTime mStateTime = new ElapsedTime();  // Time into current state
    private State mCurrentState;
    private boolean yPressed = false;

    public enum autoMode {
        MODE_DRIVE_BEACONS,
        MODE_PUSH_CAP_BALL,
        MODE_SHOOT_ONLY,
        MODE_COLOR_TEST,
        MODE_WHITE_LINE_TEST
    }

    public autoMode currentMode = autoMode.MODE_DRIVE_BEACONS;

    @Override
    public void init_loop() {
        super.init_loop();

        if (gamepad2.y && !yPressed) {
            switch (currentMode) {
                case MODE_DRIVE_BEACONS:
                    currentMode = autoMode.MODE_PUSH_CAP_BALL;
                    break;
                case MODE_PUSH_CAP_BALL:
                    currentMode = autoMode.MODE_SHOOT_ONLY;
                    break;
                case MODE_SHOOT_ONLY:
                    currentMode = autoMode.MODE_COLOR_TEST;
                    break;
                case MODE_COLOR_TEST:
                    currentMode = autoMode.MODE_WHITE_LINE_TEST;
                    break;
                case MODE_WHITE_LINE_TEST:
                    currentMode = autoMode.MODE_DRIVE_BEACONS;
                    break;
            }

        }

        yPressed = gamepad2.y;

        if (gamepad2.b) isRed = true;
        if (gamepad2.x) isRed = false;
        telemetry.addData("2", currentMode);
        telemetry.addData("3", isRed ? "RedSide" : "BlueSide");
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
        beaconPress.setPosition(servoIn);
        timesMoved = 0;

        // Init commands TODO: what am i?
        // init_loop();
    }

    public void loop() {
        loopCounter++;
        boolean extendBeaconPress = (isRed && redBlueSensor.blue() + colorSensorMargin < redBlueSensor.red() ||
                (!isRed && redBlueSensor.blue() > redBlueSensor.red() + colorSensorMargin));
        if (extendBeaconPress) {
            beaconPress.setPosition(servoExtend);
            bringBackInAt = System.currentTimeMillis() + 1000;
        } else if (bringBackInAt < System.currentTimeMillis()) {
            beaconPress.setPosition(servoIn);
        }
        super.loop();
        // Telemetry block
        telemetry.addLine()
                .addData("Loop count", loopCounter)
                .addData("State", mCurrentState);
        telemetry.addData("3", "Light detected: " + odSensor.getLightDetected());
        switch (mCurrentState) {
            case STATE_INITIAL:
                newState(STATE_FIRST_SHOT);
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
                if (System.currentTimeMillis() > startTime + 1250)
                    newState(STATE_SECOND_SHOT);
                break;

            case STATE_SECOND_SHOT:
                loader.setPosition(.15);
                if (particlesShot == 1) {
                    chrisAutoShoot();
                    particlesShot++;
                } else if (currentMode == autoMode.MODE_DRIVE_BEACONS) {
                    newState(STATE_DRIVE_BACKWARDS1);
                } else {
                    newState(STATE_STOP);
                }
                break;


            case STATE_DRIVE_BACKWARDS1:
                speed = 0.75;
                if (MecanumDrive(speed, isRed ? forwardMove(speed) : backwardMove(speed), rotationComp(), isRed ? -700 : 1100)) {
                    newState(STATE_TURN_45_1);
                    MecanumDrive(0, 0, rotationComp(), 0);
                }

                break;

            case STATE_TURN_45_1:
                targetHeading = isRed ? 45 : 313;
                MecanumDrive(0, 0, rotationComp(), 0);
                if (gyroCloseEnough(3)) {
                    newState(STATE_DRIVE_BACKWARDS2);
                }
                break;

            case STATE_DRIVE_BACKWARDS2:
                speed = isRed ? 0.75 : 1;
                if (MecanumDrive(speed, isRed ? forwardMove(speed) : backwardMove(speed), rotationComp(), isRed ? -4800 : 5000)) {
                    newState(STATE_TURN_45_2);
                    MecanumDrive(0, 0, rotationComp(), 0);
                }
                break;

            case STATE_TURN_45_2:
                targetHeading = isRed ? 90 : 270;
                MecanumDrive(0, 0, rotationComp(), 0);
                if (gyroCloseEnough(3)) {
                    newState(STATE_MOVE_LEFT);
                }
                break;

            case STATE_MOVE_LEFT:
                speed = .7;
                if (MecanumDrive(speed, isRed ? rightMove(speed) : leftMove(speed), rotationComp(), isRed ? -500 : 1000)) {
                    MecanumDrive(0, 0, 0, 0);
                    newState(STATE_DRIVE_FOR_BEACON1);
                }
                break;

            case STATE_DRIVE_FOR_BEACON1:
                speed = .3;
                if (MecanumDrive(speed, isRed ? forwardMove(speed) : backwardMove(speed), rotationComp(), 1500)) {
                    newState(STATE_DRIVE_IN_BETWEEN_BEACONS);
                }
                break;

            case STATE_DRIVE_IN_BETWEEN_BEACONS:
                speed = 1;
                if (MecanumDrive(speed, isRed ? forwardMove(speed) : backwardMove(speed), rotationComp(), 2750)) {
                    newState(STATE_DRIVE_FOR_BEACON2);
                }
                break;

            case STATE_DRIVE_FOR_BEACON2:
                speed = .3;
                if (MecanumDrive(speed, isRed ? forwardMove(speed) : backwardMove(speed), rotationComp(), 1750)) {
                    newState(STATE_CAP_TURN1);
                }
                break;

            case STATE_CAP_TURN1:
                targetHeading = isRed ? 45 : 320;
                MecanumDrive(0, 0, rotationComp(), 0);
                newState(STATE_CAP_DRIVE1);
                break;

            case STATE_CAP_DRIVE1:
                speed = 1;
                if (MecanumDrive(speed, isRed ? backwardMove(speed) : forwardMove(speed), rotationComp(), isRed ? 7500 : -7500)) {
                    MecanumDrive(0, 0, rotationComp(), 0);
                    newState(STATE_STOP);
                }
                break;

            case STATE_STOP:
                MecanumDrive(0, 0, 0, 0);
                break;
        }


    }

    public void chrisAutoShootOld() {
        shooter.setPower(1.0);
        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shooterTargetPosition -= 3360;
        do {
            shooter.setTargetPosition(shooterTargetPosition);
        } while (shooter.getCurrentPosition() >= shooterTargetPosition + 10);
    }

    public void chrisAutoShoot() {
        shooter.setPower(1.0);
        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shooterTargetPosition -= 3360;
        shooter.setTargetPosition(shooterTargetPosition);

        while (shooter.isBusy()) ;
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




