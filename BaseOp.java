package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.ams.AMSColorSensorImpl;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
    AMSColorSensorImpl redBlueSensor;
    // ColorSensor redBlueSensor; // Adafruit RGBW sensor
    OpticalDistanceSensor odSensor; // Modern Robotics RGBW sensor
    BNO055IMU imu; // Gyro
    Orientation angles; // Gyro angles

    // Variables
    public int shooterTargetPosition = 0;
    public boolean shooting = false;
    public boolean loading = false;
    double lastKnownRotJoy = 0.0;
    double targetHeading = 0.0;
    double currentGyroHeading = 0.0;

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
        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setPower(1.0);

        collector = hardwareMap.dcMotor.get("collector"); // Spinny Foam Thing Motor
        collector.setDirection(DcMotor.Direction.REVERSE);
        collector.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightBack = hardwareMap.dcMotor.get("rightBack");
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBack = hardwareMap.dcMotor.get("leftBack");
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Servo block
        loader = hardwareMap.servo.get("loader"); // shooter loader servo
        loader.setPosition(0.0); // TODO: Find right value for this

        beaconPress = hardwareMap.servo.get("beaconPress"); // beacon presser servo
        beaconPress.setPosition(0.5); // TODO: Find right value for this

        // Sensor block

        /*redBlueSensor = (AMSColorSensorImpl)hardwareMap.colorSensor.get("redBlueSensor");
        AMSColorSensor.Parameters params = redBlueSensor.getParameters();
        redBlueSensor.initialize(params);
*/
        odSensor = hardwareMap.opticalDistanceSensor.get("odSensor");

        // Gyro block
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = false;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "gyro");
        imu.initialize(parameters);

    }

    @Override
    public void init_loop() { // runs after pressing INIT and loops until START pressed
        super.init_loop();

        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (gamepad2.dpad_up) { // bring shooter up by 25 ticks
            shooter.setTargetPosition(shooter.getCurrentPosition() + 25);
        }
        if (gamepad2.dpad_down) { // bring shooter down by 25 ticks
            shooter.setTargetPosition(shooter.getCurrentPosition() - 25);
        }

        if (gamepad2.start || gamepad1.start) {
            shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        telemetry.addData("Shooter Position", shooter.getCurrentPosition());
        //telemetry.update();
    }

    public void setAutoRunMode() { // sets things specific to autonomous
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION); // sets the encoders to the right mode, tells the motor to run to the position it is given
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setTeleRunMode() { // sets things specific to teleop
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // tells the motors to use encoders
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shooter.setPower(1.0);
    }

    public void loop() { // constantly running code
        telemetry.addLine()
                .addData("ShooterPos", shooter.getCurrentPosition())
                .addData("Target", shooterTargetPosition);

        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        currentGyroHeading = Math.abs(angles.firstAngle % 360.0);
        ////telemetry.update();
    }

    public boolean shooterReady() { // DO NOT TOUCH
        return shooter.getCurrentPosition() <= shooterTargetPosition + 10; // DO NOT TOUCH
    }

    public void manualFire() { // Teleop particle shooting
        shooter.setPower(1.0);
        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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

    protected void stopDriving() {
        leftBack.setPower(0.0);
        leftFront.setPower(0.0);
        rightBack.setPower(0.0);
        rightFront.setPower(0.0);
    }


    protected void MecanumGamepadDrive() {
        double speed = 0;
        double direction = 0;
        double rotation = 0;
        double maxIncrement = 179;

        if (gamepad1.left_stick_x == 0 && lastKnownRotJoy != 0.0) {
            targetHeading = currentGyroHeading;
        }

        lastKnownRotJoy = gamepad1.left_stick_x;

        if (gamepad1.left_stick_x != 0) {
            targetHeading = (currentGyroHeading + (maxIncrement * gamepad1.left_stick_x)) % 360;
        }


        // DPad drive
        if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
            if (gamepad1.dpad_up) { // forwards
                speed = 0.3;
                direction = Math.atan2(-speed, 0) - Math.PI / 4;
            } else if (gamepad1.dpad_right) { // right
                speed = 0.7;
                direction = Math.atan2(0, -speed) - Math.PI / 4;
            } else if (gamepad1.dpad_down) { // backwards
                speed = 0.3;
                direction = Math.atan2(speed, 0) - Math.PI / 4;
            } else { // left
                speed = 0.7;
                direction = Math.atan2(0, speed) - Math.PI / 4;
            }

        }
        // Joystick drive
        else {
            // left hand drive
            // speed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            // direction = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            // rotation = -gamepad1.right_stick_x;

            // right hand drive
            speed = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
            direction = Math.atan2(gamepad1.right_stick_y, -gamepad1.right_stick_x) - Math.PI / 4;
            //rotation = -gamepad1.left_stick_x;
        }
        rotation = rotationComp();
        MecanumDrive(speed, direction, rotation);
    }

    private double rotationComp() {
        double rotation = 0.0;
        double gyro = currentGyroHeading;
        double target = targetHeading;
        double posError = gyro - target;
        double epsilon = 2;
        double minSpeed = 0.16;
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

    protected void MecanumDrive(double speed, double direction, double rotation) {
        telemetry.addLine()
                .addData("TargetHeading", targetHeading)
                .addData("ActualHeading", currentGyroHeading);
        telemetry.addLine()
                .addData("rotation", rotation);

        final double v1 = speed * Math.cos(direction) + rotation;
        final double v2 = speed * Math.sin(direction) - rotation;
        final double v3 = speed * Math.sin(direction) + rotation;
        final double v4 = speed * Math.cos(direction) - rotation;

        leftFront.setPower(v1);
        rightFront.setPower(v2);
        leftBack.setPower(v3);
        rightBack.setPower(v4);
    }
}


