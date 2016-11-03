package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="OpMode", group="Linear Opmode")  // @Autonomous(...) is the other common choice
public class OpMode extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    //these are the objects
    DcMotor l1 = null;
    DcMotor l2 = null;
    DcMotor r1 = null;
    DcMotor r2 = null;
    @Override
    public void runOpMode() throws InterruptedException {
        //This notifies the debugger that we have reached the "Initialised stage.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //This tells the phone that "l1" in the phone is "l1" here.
        l1 = hardwareMap.dcMotor.get("l1");
        l2 = hardwareMap.dcMotor.get("l2");
        r1 = hardwareMap.dcMotor.get("r1");
        r2 = hardwareMap.dcMotor.get("r2");
        //This sets the direction that the motors should spin normally.
        l1.setDirection(DcMotor.Direction.REVERSE);
        l2.setDirection(DcMotor.Direction.FORWARD);
        r1.setDirection(DcMotor.Direction.FORWARD);
        r2.setDirection(DcMotor.Direction.REVERSE);
        //Var for direction of robot
        boolean forward=true;
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            //This notifies the debugger that we have reached the "Initialised stage.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
            if(gamepad1.a){
                forward=true;
            }else if(gamepad1.b){
                forward=false;
            }

            if(forward) {
                //This sets the motors to joystick values.
                l1.setPower((gamepad1.left_stick_y)/2);
                l2.setPower((gamepad1.left_stick_y)/2);
                r1.setPower((gamepad1.right_stick_y)/2);
                r2.setPower((gamepad1.right_stick_y)/2);
            }else if(!forward){
                //This sets the motors to joystick values.
                l1.setPower((-gamepad1.right_stick_y)/2);
                l2.setPower((-gamepad1.right_stick_y)/2);
                r1.setPower((-gamepad1.left_stick_y)/2);
                r2.setPower((-gamepad1.left_stick_y)/2);
            }
            //idle is ALWAYS called at the end of the opmode thing.(Its a method)
            idle();
        }
    }
}

