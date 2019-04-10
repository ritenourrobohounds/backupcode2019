package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;


/*
Hardware Configuration on Phones

REV Expansion Hub 1:
Motor Port 0: rmf
Motor Port 1: lmf
Motor Port 2: rmb
Motor Port 3: lmb
Servo Port 4: arm


*/
public class ShovelBoiHardware {
    /* Public OpMode members. */
    public DcMotor  leftFrontMotor   = null;
    public DcMotor  leftBackMotor    = null;
    public DcMotor  rightFrontMotor  = null;
    public DcMotor  rightBackMotor   = null;
    public Servo    arm      = null;
//    public DcMotor  armLifter        = null;

    /* Local OpMode members. */
    HardwareMap hwMap  = null;
    private ElapsedTime period  = new ElapsedTime();


    /* Constructor */
    public ShovelBoiHardware() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFrontMotor   = hwMap.get(DcMotor.class, "lmf");
        leftBackMotor    = hwMap.get(DcMotor.class, "lmb");
        rightFrontMotor  = hwMap.get(DcMotor.class, "rmf");
        rightBackMotor   = hwMap.get(DcMotor.class, "rmb");
        arm      = hwMap.get(Servo.class, "arm");
//        armLifter        = hwMap.get(DcMotor.class, "lifter");
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        leftFrontMotor.setPower(0.0);
        leftBackMotor.setPower(0.0);
        rightFrontMotor.setPower(0.0);
        rightBackMotor.setPower(0.0);
//        armLifter.setPower(0.0);

//        armExtender.setPosition(0.5);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
