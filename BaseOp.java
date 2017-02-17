package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by Ultra on 9/29/2016
 * Edited by Jonathan on 11/4/2016
 * Edited by Banks on 11/4/2016
 * QUALIFIER 1: Duct Ties & Zip Tape
 * Edited by George on 11/5/2016
 * Edited by Chris on 11/5/2016
 * Edited by Banks on 11/5/2016
 * Edited by Banks on 11/9/2016
 */

public class BaseOp extends OpMode {

    // Controllers
    DcMotorController frontController; // Motor Controller 1
    DcMotorController backController; // Motor Controller 2
    DcMotorController shooterController; // Shooter Controller
    ServoController servoController; // Servo Controller 1
    DeviceInterfaceModule cdi; // Core Device Interface Module

    // Motors
    DcMotor rightFront;
    DcMotor rightBack;
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor shooter; // catapult motor
    DcMotor collector; // spinny foam intake

    // Servos
    Servo loader; // shooter loader servo
    Servo beaconPress; // beacon presser servo

    // Sensors
    ColorSensor redBlueSensor; // Adafruit RGBW sensor
    //OpticalDistanceSensor odSensor; // Modern Robotics RGBW sensor
    BNO055IMU imu; // Gyro
    Orientation angles; // Gyro angles

    // Variables
    public int shooterTargetPosition = 0;
    public boolean shooting = false;
    public boolean loading = false;
    public boolean isLeftHandDrive = true;
    double lastKnownRotJoy = 0.0;
    double targetHeading = 0.0;
    double currentGyroHeading = 0.0;
    double rotStickX = 0;
    double driveStickY = 0;
    double driveStickX = 0;
    public final double servoIn = .85;
    public final double servoExtend = .56;
    public final double servoOpposite = .28;
    public final double servoOppositeIn = .09;
    public final double colorSensorMargin = 150;
    protected boolean isRed = false;
    protected boolean nearBeacon = false;
    public long bringBackInAt = 0;
    public boolean lookingForRed = false; // 0 for blue, 1 for red
    public boolean toggleDebug = false; //toggles through the button presser states
    public boolean leftBumperPressed = false;
    public static final int retractDelay = 200;
    public boolean zeroWasAdjusted = false;
    public boolean beaconEnabled = false;
    public double safeServoPos = servoIn;
    public double extendServoPos = servoExtend;
    public boolean servoFlipped = false;
    // Mecanum variables
    double speed = 0;
    double direction = 0;
    double rotation = 0;
    int target = 0;
    boolean doneDriving = false;
    int targetDestination = 0;


    public double forwardMove(double speed) {
        return Math.atan2(-speed, 0) - Math.PI / 4;
    }

    public double backwardMove(double speed) {
        return Math.atan2(speed, 0) - Math.PI / 4;
    }

    public double rightMove(double speed) {
        return Math.atan2(0, -speed) - Math.PI / 4;
    }

    public double leftMove(double speed) {
        return Math.atan2(0, speed) - Math.PI / 4;
    }



    public void init() { // runs when any OpMode is initalized, sets up everything needed to run robot

        // Controller/Module block
        frontController = hardwareMap.dcMotorController.get("Motor Controller 1"); // serial AL00VXVX
        backController = hardwareMap.dcMotorController.get("Motor Controller 2"); // serial AL00VXRV
        shooterController = hardwareMap.dcMotorController.get("Shooter Controller"); // TODO: Get serial and put it here

        servoController = hardwareMap.servoController.get("Servo Controller 1"); // TODO: Get serial and put it here

        cdi = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1"); // TODO: Get serial and put it here
        //cdi.setLED(1, true); // turn on Blue LED TODO: does this work?


        // Motor block
        shooter = hardwareMap.dcMotor.get("shooter"); // Shooter Motor
        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterTargetPosition = 0;
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shooter.setPower(1.0);
        shooter.setTargetPosition(0);

        collector = hardwareMap.dcMotor.get("collector"); // Spinny Foam Thing Motor
        collector.setDirection(DcMotor.Direction.REVERSE);
        collector.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightBack = hardwareMap.dcMotor.get("rightBack");
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBack = hardwareMap.dcMotor.get("leftBack");
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Servo block
        loader = hardwareMap.servo.get("loader"); // shooter loader servo
        loader.setPosition(0.0); // TODO: Find right value for this

        beaconPress = hardwareMap.servo.get("beaconPress"); // beacon presser servo
        beaconPress.setPosition(servoIn);

        // Sensor block

        /*redBlueSensor = (AMSColorSensorImpl)hardwareMap.colorSensor.get("redBlueSensor");
        AMSColorSensor.Parameters params = redBlueSensor.getParameters();
        redBlueSensor.initialize(params);
*/
        //odSensor = hardwareMap.opticalDistanceSensor.get("odSensor");
        redBlueSensor = hardwareMap.colorSensor.get("redBlueSensor");

        // Gyro block
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = false;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "gyro");

        telemetry.addData("1", "init");
        imu.initialize(parameters);
        telemetry.addData("1", "done");
    }

    @Override
    public void init_loop() { // runs after pressing INIT and loops until START pressed
        super.init_loop();
        if (gamepad2.dpad_up) { // bring shooter up by 25 ticks
            shooterTargetPosition = shooter.getCurrentPosition() + 25;
            shooter.setTargetPosition(shooterTargetPosition);
        }
        if (gamepad2.dpad_down) { // bring shooter down by 25 ticks
            shooterTargetPosition = shooter.getCurrentPosition() - 25;
            shooter.setTargetPosition(shooterTargetPosition);
        }
        if ((!gamepad2.dpad_down && !gamepad2.dpad_up) && zeroWasAdjusted) {
            shooterTargetPosition = 0;
            shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shooter.setTargetPosition(shooterTargetPosition);
        }
        zeroWasAdjusted = (gamepad2.dpad_down || gamepad2.dpad_up);
        telemetry.addData("Shooter Position", shooter.getCurrentPosition());

        nearBeacon = false;
    }

    public void setRunMode() { // sets things specific to autonomous

    }


    public void loop() { // constantly running code
      //  telemetry.addLine()
                //.addData("ShooterPos", shooter.getCurrentPosition())
                //.addData("Target", shooterTargetPosition);

        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        currentGyroHeading = Math.abs(angles.firstAngle % 360.0);
        ////telemetry.update();
    }

    public boolean shooterReady() { // DO NOT TOUCH
        return !shooter.isBusy();
        //return shooter.getCurrentPosition() <= shooterTargetPosition + 10; // DO NOT TOUCH
    }

    public void manualFire() { // Teleop particle shooting
        shooter.setPower(1.0);
       // shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (!shooting && shooterReady()) { // if NOT SHOOTING but READY TO SHOOT
            shooterTargetPosition -= 3360; // make our target to 1 full rotation
            shooter.setTargetPosition(shooterTargetPosition); // set that target as above
            shooting = true;
            loading = false;
            telemetry.addData("getting ready to shoot", "");
        } else if (shooting && shooterReady() && !loading) { // IF SHOOTING and READY TO SHOOT
            loading = true;
            loader.setPosition(0.5);
            telemetry.addData("Trying to Load", "");
        } else if (loading && loader.getPosition() <= 0.49) { // IF LOADING and LOADER NOT THERE
            loading = false;
            shooting = false;
            loader.setPosition(0.15);
            telemetry.addData("else done shooting", "");
        } else {
            shooting = false;
            loading = false;
            telemetry.addData("None of the Above", "");
        }
    }

    protected void MecanumGamepadDrive() {
        target = 0;
        double maxIncrement = 100;

        if (isLeftHandDrive) {
            rotStickX = gamepad1.right_stick_x;
            driveStickX = gamepad1.left_stick_x;
            driveStickY = gamepad1.left_stick_y;
        } else {
            rotStickX = gamepad1.left_stick_x;
            driveStickX = gamepad1.right_stick_x;
            driveStickY = gamepad1.right_stick_y;
        }
        rotStickX = Math.pow(rotStickX, 3);


        if (rotStickX == 0 && lastKnownRotJoy != 0.0) {
            targetHeading = currentGyroHeading;
        }

        lastKnownRotJoy = rotStickX;

        if (rotStickX != 0) {
            targetHeading = (currentGyroHeading + (maxIncrement * rotStickX)) % 360;
        }


        // DPad drive
        if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
            if (gamepad1.dpad_up) { // forwards
                speed = gamepad1.right_bumper?1.0: 0.3;
                direction = Math.atan2(-speed, 0) - Math.PI / 4;
            } else if (gamepad1.dpad_right) { // right
                speed = 0.7;
                direction = Math.atan2(0, -speed) - Math.PI / 4;
            } else if (gamepad1.dpad_down) { // backwards
                speed = gamepad1.right_bumper?1.0:.3;
                direction = Math.atan2(speed, 0) - Math.PI / 4;
            } else { // left
                speed = 0.7;
                direction = Math.atan2(0, speed) - Math.PI / 4;
            }
        }

        // Joystick drive
        else {
            speed = Math.hypot(driveStickX, driveStickY);
            direction = Math.atan2(driveStickY, -driveStickX) - Math.PI / 4;
        }

        MecanumDrive(speed, direction, rotation, target);
    }

    protected boolean MecanumDrive(double speed, double direction, double rotation, int target) {

        if (rotation == 0) {
            rotation = rotationComp();
        }


        if (target != 0 && targetDestination == 0) {
            targetDestination = leftFront.getCurrentPosition() + target;
        }

        /*telemetry.addLine()
                .addData("3", "TargetHeading", targetHeading)
                .addData("3", "ActualHeading", currentGyroHeading);
        telemetry.addLine()
                .addData("4", "rotation", rotation);*/

       // telemetry.addLine()
                //.addData("Actual", leftFront.getCurrentPosition())
                //.addData("Dest", targetDestination);


        final double v1 = speed * Math.cos(direction) + rotation;
        final double v2 = speed * Math.sin(direction) - rotation;
        final double v3 = speed * Math.sin(direction) + rotation;
        final double v4 = speed * Math.cos(direction) - rotation;

        leftFront.setPower(v1);
        rightFront.setPower(v2);
        leftBack.setPower(v3);
        rightBack.setPower(v4);

        if (targetDestination != 0 && Math.abs(leftFront.getCurrentPosition() - targetDestination) < 100) {
            targetDestination = 0;
            return true;
        } else {
            return false;
        }
    }

    protected double rotationComp() {
        double rotation = 0.0;
        double gyro = currentGyroHeading;
        double target = targetHeading;
        double posError = gyro - target;
        double epsilon = 2; // was 4
        double minSpeed = 0.12;
        double maxSpeed = 1;

        if (Math.abs(posError) > 180) {
            posError = -360 * Math.signum(posError) + posError;
        }
        if (Math.abs(posError) > epsilon) {
            rotation = minSpeed + (Math.abs(posError) / 180) * (maxSpeed - minSpeed);
            rotation = rotation * Math.signum(posError);
        }
        return rotation;
    }
}