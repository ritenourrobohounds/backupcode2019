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

@TeleOp(name="Sir Andy Nolen", group="MASTERX")
//@Disabled
public class MecanumTeleOpWithLatching extends LinearOpMode {

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
        boolean onSwitchCollector = false;

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
            robot.color.enableLed(false);

            gamepad1LeftY = gamepad1.left_stick_y;
            gamepad1LeftX = gamepad1.left_stick_x;
            gamepad1RightX = -gamepad1.right_stick_x;
            telemetry.addData("Blue: ", robot.color.blue());
            telemetry.addData("Red: ", robot.color.red());
            telemetry.addData("LeftY: ", gamepad1LeftY);
            telemetry.addData("LeftX: ", gamepad1LeftX);
            telemetry.addData("RightX: ", gamepad1RightX);
            telemetry.addData("LatchHelperStatus: ", LatchStatus);

            telemetry.update();

            //Mecanum Formulas
            final double v1 = ((gamepad1LeftY - gamepad1LeftX + gamepad1RightX)*SPEED_MODIFIER);
            final double v2 = ((gamepad1LeftY + gamepad1LeftX - gamepad1RightX)*SPEED_MODIFIER);
            final double v3 = ((gamepad1LeftY + gamepad1LeftX + gamepad1RightX)*SPEED_MODIFIER);
            final double v4 = ((gamepad1LeftY - gamepad1LeftX - gamepad1RightX)*SPEED_MODIFIER);

            //Write the values to the Motor`
            robot.leftFrontMotor.setPower(v1);
            robot.leftBackMotor.setPower(v3);
            robot.rightFrontMotor.setPower(v2);
            robot.rightBackMotor.setPower(v4);



            if (gamepad1.dpad_up){
                robot.latchingMotor.setPower(LATCH_UP);
            }
            else if (gamepad1.dpad_down){
                robot.latchingMotor.setPower(LATCH_DOWN);
            }
            else {
                robot.latchingMotor.setPower(LATCH_STOP);
            }

            /**
            if (gamepad2.x){
                robot.mineralCollectorServo.setPosition(1);
            }
            else if(gamepad2.b){
                robot.mineralCollectorServo.setPosition(0);
            }
            else{
                robot.mineralCollectorServo.setPosition(.5);
            }
            **/


            if (gamepad2.x){
                robot.mineralCollectorServo.setPosition(1);
            }
            else{
                robot.mineralCollectorServo.setPosition(0.5);
            }

            if (gamepad2.y){
                robot.moveMineralBox.setPosition(1);
            }
            else if (gamepad2.b){
                robot.moveMineralBox.setPosition(.9);
            }
            else if (gamepad2.a){
                robot.moveMineralBox.setPosition(.3);
            }


            if(gamepad2.right_bumper){
                robot.extendMineralCollector.setPower(0.5);
            }
            else if(gamepad2.left_bumper) {
                robot.extendMineralCollector.setPower(-1.0);
            }
            else{
                robot.extendMineralCollector.setPower(0);
            }

            if (gamepad1.left_bumper){
                if (SPEED_MODIFIER > 0) {
                    SPEED_MODIFIER -= SPEED_MODIFIER_INTERVAL;
                }
            }
            if (gamepad1.right_bumper){
                if (SPEED_MODIFIER < 1) {
                    SPEED_MODIFIER += SPEED_MODIFIER_INTERVAL;
                }
            }

            robot.moveMineralCollector1.setPower(gamepad2.left_stick_y);
            robot.moveMineralCollector2.setPower(gamepad2.left_stick_y);

            // Pause for 40 mS each cycle = update 25 times a second.
            sleep(40);
        }
    }
}
