package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

@Autonomous(name="GyroTestingImu", group="PushBot")

public class GyroSensorAutoTesting extends LinearOpMode{
    // Use this code when starting at depot
    // This a depot code that you can revise

    /* Public OpMode members. */
    MasterXHardware robot = new MasterXHardware();
    private ElapsedTime runtime  = new ElapsedTime();
    SamplingOrderDetector detector = new SamplingOrderDetector();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: AndyMark NeveRest Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     FORWARD_SPEED   = 0.6;
    static final double     TURN_SPEED      = 0.55;
    static final double     SUPERSONICSPEED      = 1.0;
    double sample_speed = 0.3;

    double boneDispenser_up_position = 1.0; // 1=near phone case
    double boneDispenser_down_position = 0.0; // 0 = drop outside

    double latch_up_speed = 0.5;
    double latch_down_speed = 0.5;

    long sleepTime = 1000;


    BNO055IMU imu;
    // State used for updating telemetry
    Orientation angles = new Orientation();




    @Override
    public void runOpMode()throws InterruptedException{

        robot.init(hardwareMap);
        robot.boneDispenser.setPosition(boneDispenser_up_position);
        detector = new SamplingOrderDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), -1, false);
        detector.useDefaults();


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // Optional Tuning
       // detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned.(Represented by green bars in the preview)
       // detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        // ^ are commented because they threw up an error when I imported SamplingOrderDetector
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;
        robot.phoneServo.setPosition(.47);// 1=vertical, 0=horizontal


        telemetry.addData("Status", "Ready to run");
        telemetry.update();
        waitForStart();

        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        runtime.reset();

        moveForward(.5,1);

        TurnLeftAccurately(90);



    }

    public void moveForward(double forwardSpeed, double time){
        // DON'T CHANGE THIS CODE DAMIEN! IT WORKS!
        runtime.reset();
        while(opModeIsActive() && runtime.seconds()<time) {
            robot.leftFrontMotor.setPower(-forwardSpeed);
            robot.rightFrontMotor.setPower(-forwardSpeed);
            robot.leftBackMotor.setPower(-forwardSpeed);
            robot.rightBackMotor.setPower(-forwardSpeed);
        }
    }
    public void turnLeft(double turnSpeed, double time) {
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < time) {
            robot.leftFrontMotor.setPower(-turnSpeed);
            robot.rightFrontMotor.setPower(turnSpeed);
            robot.leftBackMotor.setPower(-turnSpeed);
            robot.rightBackMotor.setPower(turnSpeed);
        }
    }
    public void turnRight(double turnSpeed, double time) {
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < time) {
            robot.leftFrontMotor.setPower(turnSpeed);
            robot.rightFrontMotor.setPower(-turnSpeed);
            robot.leftBackMotor.setPower(turnSpeed);
            robot.rightBackMotor.setPower(-turnSpeed);
        }
    }
    public void moveBackward(double backwardSpeed, double time) {
        // DON'T CHANGE THIS CODE DAMIEN! IT WORKS!
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < time) {
            robot.leftFrontMotor.setPower(backwardSpeed);
            robot.rightFrontMotor.setPower(backwardSpeed);
            robot.leftBackMotor.setPower(backwardSpeed);
            robot.rightBackMotor.setPower(backwardSpeed);
        }
    }
    public void strafeLeft(double forwardSpeed, double time){
        runtime.reset();
        while(opModeIsActive() && runtime.seconds()<time) {
            robot.leftFrontMotor.setPower(forwardSpeed);
            robot.rightFrontMotor.setPower(-forwardSpeed);
            robot.leftBackMotor.setPower(-forwardSpeed);
            robot.rightBackMotor.setPower(forwardSpeed);
        }
    }
    public void strafeRight(double forwardSpeed, double time) {
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < time) {
            robot.leftFrontMotor.setPower(-forwardSpeed);
            robot.rightFrontMotor.setPower(forwardSpeed);
            robot.leftBackMotor.setPower(forwardSpeed);
            robot.rightBackMotor.setPower(-forwardSpeed);
        }
    }

    public void encoderDriveBackward(double speed, double inches, double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.leftFrontMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFrontMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newLeftBackTarget = robot.leftBackMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightBackMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

            robot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            robot.rightFrontMotor.setTargetPosition(newRightFrontTarget);
            robot.leftBackMotor.setTargetPosition(newLeftBackTarget);
            robot.rightBackMotor.setTargetPosition(newRightBackTarget);
            // Turn On RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftFrontMotor.setPower(Math.abs(speed));
            robot.rightFrontMotor.setPower(Math.abs(speed));
            robot.leftBackMotor.setPower(Math.abs(speed));
            robot.rightBackMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy()) && robot.rightBackMotor.isBusy() && robot.leftBackMotor.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftFrontMotor.getCurrentPosition(),
                        robot.rightFrontMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftFrontMotor.setPower(0);
            robot.rightFrontMotor.setPower(0);
            robot.leftBackMotor.setPower(0);
            robot.rightBackMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    public void encoderDriveForward(double speed, double inches, double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.leftFrontMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFrontMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            newLeftBackTarget = robot.leftBackMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightBackMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);

            robot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            robot.rightFrontMotor.setTargetPosition(newRightFrontTarget);
            robot.leftBackMotor.setTargetPosition(newLeftBackTarget);
            robot.rightBackMotor.setTargetPosition(newRightBackTarget);
            // Turn On RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftFrontMotor.setPower(Math.abs(speed));
            robot.rightFrontMotor.setPower(Math.abs(speed));
            robot.leftBackMotor.setPower(Math.abs(speed));
            robot.rightBackMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy()) && robot.rightBackMotor.isBusy() && robot.leftBackMotor.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftFrontMotor.getCurrentPosition(),
                        robot.rightFrontMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftFrontMotor.setPower(0);
            robot.rightFrontMotor.setPower(0);
            robot.leftBackMotor.setPower(0);
            robot.rightBackMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    public void encoderDriveRight(double speedFront, double speedBack, double inches, double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller

            newLeftFrontTarget = robot.leftFrontMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFrontMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            newLeftBackTarget = robot.leftBackMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightBackMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            robot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            robot.rightFrontMotor.setTargetPosition(newRightFrontTarget);
            robot.leftBackMotor.setTargetPosition(newLeftBackTarget);
            robot.rightBackMotor.setTargetPosition(newRightBackTarget);
            // Turn On RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftFrontMotor.setPower(Math.abs(speedFront));
            robot.rightFrontMotor.setPower(Math.abs(speedFront));
            robot.leftBackMotor.setPower(Math.abs(speedBack));
            robot.rightBackMotor.setPower(Math.abs(speedBack));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy()) && robot.rightBackMotor.isBusy() && robot.leftBackMotor.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftFrontMotor.getCurrentPosition(),
                        robot.rightFrontMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftFrontMotor.setPower(0);
            robot.rightFrontMotor.setPower(0);
            robot.leftBackMotor.setPower(0);
            robot.rightBackMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    public void encoderDriveLeft(double speedFront, double speedBack, double inches, double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.leftFrontMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFrontMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newLeftBackTarget = robot.leftBackMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightBackMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);

            robot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            robot.rightFrontMotor.setTargetPosition(newRightFrontTarget);
            robot.leftBackMotor.setTargetPosition(newLeftBackTarget);
            robot.rightBackMotor.setTargetPosition(newRightBackTarget);
            // Turn On RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftFrontMotor.setPower(Math.abs(speedFront));
            robot.rightFrontMotor.setPower(Math.abs(speedFront));
            robot.leftBackMotor.setPower(Math.abs(speedBack));
            robot.rightBackMotor.setPower(Math.abs(speedBack));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy()) && robot.rightBackMotor.isBusy() && robot.leftBackMotor.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftFrontMotor.getCurrentPosition(),
                        robot.rightFrontMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftFrontMotor.setPower(0);
            robot.rightFrontMotor.setPower(0);
            robot.leftBackMotor.setPower(0);
            robot.rightBackMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    public void encoderDriveTurnLeft(double speed, double inches, double timeoutS)  {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.leftFrontMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFrontMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            newLeftBackTarget = robot.leftBackMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightBackMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);

            robot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            robot.rightFrontMotor.setTargetPosition(newRightFrontTarget);
            robot.leftBackMotor.setTargetPosition(newLeftBackTarget);
            robot.rightBackMotor.setTargetPosition(newRightBackTarget);
            // Turn On RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftFrontMotor.setPower(Math.abs(speed));
            robot.rightFrontMotor.setPower(Math.abs(speed));
            robot.leftBackMotor.setPower(Math.abs(speed));
            robot.rightBackMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy()) && robot.rightBackMotor.isBusy() && robot.leftBackMotor.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftFrontMotor.getCurrentPosition(),
                        robot.rightFrontMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftFrontMotor.setPower(0);
            robot.rightFrontMotor.setPower(0);
            robot.leftBackMotor.setPower(0);
            robot.rightBackMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    public void encoderDriveTurnRight(double speed, double inches, double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.leftFrontMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFrontMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newLeftBackTarget = robot.leftBackMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightBackMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

            robot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            robot.rightFrontMotor.setTargetPosition(newRightFrontTarget);
            robot.leftBackMotor.setTargetPosition(newLeftBackTarget);
            robot.rightBackMotor.setTargetPosition(newRightBackTarget);
            // Turn On RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftFrontMotor.setPower(Math.abs(speed));
            robot.rightFrontMotor.setPower(Math.abs(speed));
            robot.leftBackMotor.setPower(Math.abs(speed));
            robot.rightBackMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy()) && robot.rightBackMotor.isBusy() && robot.leftBackMotor.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftFrontMotor.getCurrentPosition(),
                        robot.rightFrontMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftFrontMotor.setPower(0);
            robot.rightFrontMotor.setPower(0);
            robot.leftBackMotor.setPower(0);
            robot.rightBackMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }

    }

    public void unlatch (double timeoutS){
        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < timeoutS)){
            robot.latchingMotor.setPower(-1.0);
        }
        robot.latchingMotor.setPower(0.0);
    }
    public void latch (double timeoutS){
        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < timeoutS)){
            robot.latchingMotor.setPower(1.0);
        }
        robot.latchingMotor.setPower(0.0);
    }

    public void markerUp(){
        robot.boneDispenser.setPosition(boneDispenser_up_position);
    }
    public void markerDown(){
        robot.boneDispenser.setPosition(boneDispenser_down_position);
    }

    public void CheckAllign(){
        detector.enable();
        StopRobot();
        sleep(2000);

     /*  while(opModeIsActive() && detector.getAligned() != true){
        // This while loop will move the robot backwards until it find the gold mineral
            robot.rightBackMotor.setPower(-.15);
            robot.rightFrontMotor.setPower(-.15);
            robot.leftBackMotor.setPower(-.15);
            robot.leftFrontMotor.setPower(-.15);
        }
*/ //This is commented because it threw up an error when I imported SamplingOrderDetector
        telemetry.addData("Testing", "Found gold");
        telemetry.update();
        StopRobot();
        sleep(1000);
    //    strafeLeft(0.3, 2.2);
        turnRight(.3,2.2);
        moveForward(.3,2.0);
    }

    public void pauseRobot(double timeoutS){
        while (opModeIsActive() && (runtime.seconds() < timeoutS)){
            robot.rightBackMotor.setPower(0.00);
            robot.leftBackMotor.setPower(0.00);
            robot.rightFrontMotor.setPower(0.00);
            robot.leftFrontMotor.setPower(0.00);
        }
    }
    public void StopRobot(){
        robot.rightBackMotor.setPower(0.00);
        robot.leftBackMotor.setPower(0.00);
        robot.rightFrontMotor.setPower(0.00);
        robot.leftFrontMotor.setPower(0.00);
    }

    public void TurnLeftAccurately(double rightThing){

        while (angles.firstAngle < rightThing ){

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            robot.leftFrontMotor.setPower(1.0);
            robot.rightFrontMotor.setPower(-1.0);
            robot.leftBackMotor.setPower(1.0);
            robot.rightBackMotor.setPower(-1.0);
            telemetry.addData("Angle: ", angles.firstAngle);
            telemetry.update();
        }
        while (angles.firstAngle > rightThing){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            robot.leftFrontMotor.setPower(-1.0);
            robot.rightFrontMotor.setPower(1.0);
            robot.leftBackMotor.setPower(-1.0);
            robot.rightBackMotor.setPower(1.0);
        }
        StopRobot();
    }




}