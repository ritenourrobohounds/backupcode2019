package org.firstinspires.ftc.teamcode;
import android.view.FocusFinder;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.FormatFlagsConversionMismatchException;

@Autonomous(name="Robot Lower", group="PushBot")
//@Disabled
public class  RobotLatchLower extends LinearOpMode{

    /* Public OpMode members. */
    MasterXHardware robot = new MasterXHardware();
    private ElapsedTime runtime  = new ElapsedTime();


    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: AndyMark NeveRest Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     FORWARD_SPEED   = 0.6;
    static final double     TURN_SPEED      = 0.55;
    static final double     SUPERSONICSPEED      = 1.0;
    double sample_speed = 0.3;

    double boneDispenser_up_position = .69;
    double boneDispenser_down_position = 1;

    double camera_position = 0.37;

    double latch_up_speed = 0.5;
    double latch_down_speed = 0.5;

    long sleepTime = 1000;

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        // robot.boneDispenser.setPosition(boneDispenser_up_position);

        robot.phoneServo.setPosition(camera_position);// 1=vertical, 0=horizontal

        telemetry.addData("Status", "Ready to run");
        telemetry.update();
        waitForStart();

        ReLatch(1,6.6);

    }
    public void ReLatch(double Power, double Time){
        runtime.reset();
        while(opModeIsActive() && runtime.seconds()<Time) {
            robot.latchingMotor.setPower(Power);
        }
    }
}