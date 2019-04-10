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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
/*
Hardware Configuration on Phones

REV Expansion Hub 1:

Motor Port 0: rmf
Motor Port 1: rmb
Motor Port 2: latching
Motor Port 3: mover2



REV Expansion Hub 2:

Motor Port 0: lmf
Motor Port 1: lmb
Motor Port 2: mover1
Motor Port 3: extendor
Servo Port 5: collector
Servo Port 4: dumper
Servo port 3: bone
Servo Port 2: soul
Servo Port 0: craterclaim



*/

public class MasterXHardware
{
    /* Public OpMode members. */
    public DcMotor  leftFrontMotor          = null;
    public DcMotor  leftBackMotor           = null;
    public DcMotor  rightFrontMotor         = null;
    public DcMotor  rightBackMotor          = null;
    public DcMotor  latchingMotor           = null;
    public Servo    moveMineralBox          = null;
    public Servo    phoneServo              = null;
    public Servo    mineralCollectorServo   = null;
    public DcMotor  moveMineralCollector1   = null;
    public DcMotor  moveMineralCollector2   = null;
    public DcMotor  extendMineralCollector  = null;
    public Servo    craterclaim              = null;
    public Servo boneDispenser = null;

    public ColorSensor color = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public MasterXHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map

        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFrontMotor  = hwMap.get(DcMotor.class, "lmf");
        leftBackMotor = hwMap.get(DcMotor.class, "lmb");
        rightFrontMotor    = hwMap.get(DcMotor.class, "rmf");
        rightBackMotor    = hwMap.get(DcMotor.class, "rmb");
        latchingMotor = hwMap.get(DcMotor.class, "latching");
        mineralCollectorServo = hwMap.get(Servo.class, "collector");
        moveMineralBox = hwMap.get(Servo.class, "dumper");
        moveMineralCollector1 = hwMap.get(DcMotor.class, "mover1");
        moveMineralCollector2 = hwMap.get(DcMotor.class, "mover2");
        extendMineralCollector = hwMap.get(DcMotor.class, "extendor");
        phoneServo = hwMap.get(Servo.class, "soul");
        craterclaim = hwMap.get(Servo.class, "craterclaim");

        boneDispenser = hwMap.get(Servo.class, "bone");

        color = hwMap.get(ColorSensor.class, "depotSensor");
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
//        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
//        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);
        latchingMotor.setDirection(DcMotor.Direction.REVERSE);


        // Set all motors to zero power
        leftFrontMotor.setPower(0.0);
        leftBackMotor.setPower(0.0);
        rightFrontMotor.setPower(0.0);
        rightBackMotor.setPower(0.0);
        latchingMotor.setPower(0.0);

        moveMineralCollector1.setPower(0.0);
        moveMineralCollector2.setPower(0.0);
        extendMineralCollector.setPower(0.0);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        latchingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        phoneServo.setPosition(.5);
        craterclaim.setPosition(0.002);
    }
 }