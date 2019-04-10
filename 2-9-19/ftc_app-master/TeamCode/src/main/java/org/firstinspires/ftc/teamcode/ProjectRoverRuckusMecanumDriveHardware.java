/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
//import org.firstinspires.ftc.teamcode.GoldAlignExample;

public class ProjectRoverRuckusMecanumDriveHardware
{
    /* Public OpMode members. */
    public DcMotor leftDriveBack = null;
    public DcMotor rightDriveBack  = null;
    public DcMotor leftDriveFront  = null;
    public DcMotor rightDriveFront  = null;
    public DcMotor LatchUp = null;
    public Servo boneDispenser = null;
    public DcMotor KermitCollector = null;
    public DcMotor LinearSlide = null;
    public DcMotor MineralDepot = null;
    public Servo KermmitMover = null;

    /* Local OpMode members. */
    HardwareMap hwMap  = null;
    private ElapsedTime period  = new ElapsedTime();

    public static final double   COUNTS_PER_MOTOR_REV    = 1120 ;  // eg: AndyMark NeveRest Motor Encoder
    public static final double   DRIVE_GEAR_REDUCTION    = 0.25 ;  // This is < 1.0 if geared UP
    public static final double   WHEEL_DIAMETER_INCHES   = 4.0 ;   // For figuring circumference
    public static final double   COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    /* Constructor */
    public ProjectRoverRuckusMecanumDriveHardware() { }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDriveBack = hwMap.get(DcMotor.class, "lmb");
        leftDriveBack.setDirection(DcMotor.Direction.REVERSE);
        rightDriveBack = hwMap.get(DcMotor.class, "rmb");
        rightDriveFront = hwMap.get(DcMotor.class, "rmf");
        leftDriveFront  = hwMap.get(DcMotor.class, "lmf");
        leftDriveFront.setDirection(DcMotor.Direction.REVERSE);
        LinearSlide = hwMap.get(DcMotor.class,"l");
        KermitCollector = hwMap.get(DcMotor.class,"kc");
        LatchUp = hwMap.get(DcMotor.class,"lu");
        MineralDepot = hwMap.get(DcMotor.class, "mineral");
        boneDispenser = hwMap.get(Servo.class, "b");
        KermmitMover = hwMap.get(Servo.class,"km");
//        KermitExtender = hwMap.get(DcMotor.class, "ke");

        // Set all motors to zero power
        leftDriveBack.setPower(0);
        rightDriveBack.setPower(0);
        leftDriveFront.setPower(0);
        rightDriveFront.setPower(0);
        MineralDepot.setPower(0);
        LinearSlide.setPower(0);
        KermitCollector.setPower(0);
        LatchUp.setPower(0);
//        KermitExtender.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        KermitCollector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LatchUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MineralDepot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}