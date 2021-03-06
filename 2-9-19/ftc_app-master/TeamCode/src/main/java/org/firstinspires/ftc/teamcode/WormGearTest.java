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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Worm Gear Test", group="MASTERX")
//@Disabled
public class WormGearTest extends LinearOpMode {

    /* Declare OpMode members. */
    MasterXHardware robot = new MasterXHardware();
    private ElapsedTime runtime  = new ElapsedTime();
    @Override
    public void runOpMode() {

        double gamepad1LeftY;
        double gamepad1LeftX;
        double gamepad1RightX;

        double LATCH_UP = -1.0;
        double LATCH_DOWN = 1.0;
        double LATCH_STAY = 1.0;
        double LATCH_STOP = 0.0;

        boolean LATCHHELP = false;
        boolean COLLECTOR_STATUS = false;

        double SPEED_MODIFIER = 1.00;
        double SPEED_MODIFIER_INTERVAL = 0.25;
        double camera_position = 0.7;
        double camera_down = 0.05;

        String LatchStatus = "Off";

        robot.init(hardwareMap);
        robot.phoneServo.setPosition(camera_down);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Welcome to Rover Ruckus Mecanum.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive())
        {
            robot.moveMineralCollector1.setPower(gamepad1.left_stick_y);
            robot.moveMineralCollector2.setPower(gamepad1.right_stick_y);
            // Pause for 40 mS each cycle = update 25 times a second.
            sleep(40);
        }
    }
}