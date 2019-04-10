package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.ShovelBoiHardware;
import com.qualcomm.ftccommon.SoundPlayer;

@TeleOp(name="ShovelBoi", group="Pushbot")
//@Disabled
public class ShovelBoi extends LinearOpMode {
    /* Declare OpMode members. */
    ShovelBoiHardware robot = new ShovelBoiHardware();
    private boolean hiFound;

    @Override
    public void runOpMode() {

        double gamepad1LeftY;
        double gamepad1LeftX;
        double gamepad1RightX;
        double gamepad1RightY;

        double SPEED_MODIFIER = 1.00;
        double SPEED_MODIFIER_INTERVAL = 0.25;
        robot.init(hardwareMap);
        int Hi = hardwareMap.appContext.getResources().getIdentifier("hi", "raw", hardwareMap.appContext.getPackageName());
        int ritIsLit = hardwareMap.appContext.getResources().getIdentifier("ritislit", "raw", hardwareMap.appContext.getPackageName());
        int fatBoiTaint = hardwareMap.appContext.getResources().getIdentifier("fatboitaint", "raw", hardwareMap.appContext.getPackageName());
        int mong = hardwareMap.appContext.getResources().getIdentifier("mong", "raw", hardwareMap.appContext.getPackageName());
        if (Hi != 0)
            hiFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, Hi);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "ShovelBoi is ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            gamepad1LeftY = -gamepad1.left_stick_y;
            gamepad1LeftX = -gamepad1.left_stick_x;
            gamepad1RightX = gamepad1.right_stick_x;
            gamepad1RightY = -gamepad1.right_stick_y;
            telemetry.addData("LeftY: ", gamepad1LeftY);
            telemetry.addData("LeftX: ", gamepad1LeftX);
            telemetry.addData("RightX: ", gamepad1RightX);
            telemetry.update();

            //Mecanum Formulas
            final double v1 = ((gamepad1LeftY - gamepad1LeftX + gamepad1RightX) * SPEED_MODIFIER);
            final double v2 = ((gamepad1LeftY + gamepad1LeftX - gamepad1RightX) * SPEED_MODIFIER);
            final double v3 = ((gamepad1LeftY + gamepad1LeftX + gamepad1RightX) * SPEED_MODIFIER);
            final double v4 = ((gamepad1LeftY - gamepad1LeftX - gamepad1RightX) * SPEED_MODIFIER);

            //Write the values to the Motor`
            if (v1 != 0 || v2 != 0 || v3 != 0 || v4 != 0) {
                robot.leftFrontMotor.setPower(v3);
                robot.leftBackMotor.setPower(v1);
                robot.rightFrontMotor.setPower(v4);
                robot.rightBackMotor.setPower(v2);
            } else {
                robot.leftFrontMotor.setPower(0.00);
                robot.leftBackMotor.setPower(0.00);
                robot.rightFrontMotor.setPower(0.00);
                robot.rightBackMotor.setPower(0.00);
            }

            if (gamepad1.left_bumper) {
                if (SPEED_MODIFIER > 0) {
                    SPEED_MODIFIER -= SPEED_MODIFIER_INTERVAL;
                }
            }
            if (gamepad1.right_bumper) {
                if (SPEED_MODIFIER < 1) {
                    SPEED_MODIFIER += SPEED_MODIFIER_INTERVAL;
                }
            }
            if (gamepad1.x) {
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, Hi);
                telemetry.addData("Playing", "Robot Saying Hi");
                telemetry.update();
            }
            if (gamepad1.b) {
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, ritIsLit);
                telemetry.addData("Playing", "Robot Saying hello i am shovle boi");
                telemetry.update();
            }

            if (gamepad1.dpad_up) {
                robot.arm.setPosition(1);
            }
            if (gamepad1.dpad_down) {
                robot.arm.setPosition(0);
            }
                    //right joystick y = arm motor
/*            if(gamepad1RightY > 1){
                robot.armLifter.setPower(0.5);
            }
            else if(gamepad1RightY < 1){
                robot.armLifter.setPower(-0.5);
            }
            else{
                robot.armLifter.setPower(0.0);
            }
            //dpad up and down for arm servo
            if(gamepad1.dpad_up){
                robot.armExtender.setPosition(1);
            }
            if(gamepad1.dpad_down){
                robot.armExtender.setPosition(0);
            }
*/

                    // Pause for 40 mS each cycle = update 25 times a second.
            sleep(40);

            }
        }
    }


