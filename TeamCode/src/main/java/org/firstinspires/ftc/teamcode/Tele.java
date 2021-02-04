
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;

//import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.teamcode.Hardware.Bot;
import org.firstinspires.ftc.teamcode.Hardware.*;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Tele", group="Teleop")

//@Disabled
public class Tele extends OpMode {

    Bot bot = new Bot();
    int TankDrive = -1;
    double factor = 0.75;
    TankDrive drive;
    /*
    fr = 1
    fl = 2
    bl = 3
    br = 4

     */

    @Override
    public void init() {
        bot.init(hardwareMap, telemetry, false);
        drive = new TankDrive(bot);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        // Tank/Game drive initialization moved to init so it is not re-initialized every time
        double leftStickY = (double) -gamepad1.left_stick_y;
        double rightTrigger = (double) gamepad1.right_trigger;
        double leftTrigger = (double) gamepad1.left_trigger;
        double rightStickX = (double) gamepad1.right_stick_x;
        double rightStickY = (double) -gamepad1.right_stick_y;

 //       double leftTriggerIntake = (double) gamepad2.left_trigger;
 //       double rightTriggerIntake = (double) -gamepad2.right_trigger;
  //      double leftStickServo = (double) gamepad2.left_stick_x;
 //       double rightStickServo = (double) gamepad2.right_stick_x;

        drive.driveBot(leftStickY, rightStickX, rightTrigger, leftTrigger, rightStickY, 0.0, factor);

        bot.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if(gamepad2.right_trigger == 0.0) {
            bot.shooter.setPower(0);
        }
        else if(gamepad2.right_trigger == 0.5) {
            bot.shooter.setPower(-0.5);
        }
        else if(gamepad2.right_trigger == 1) {
            bot.shooter.setPower(-0.9);
        }

        bot.intake.setPower(-gamepad2.left_trigger);

        if(gamepad2.a) {
            bot.discPlacer.setPosition(0.1);
            telemetry.addData(">", "0.1");
            telemetry.update();
        }
        if(gamepad2.b) {
            bot.discPlacer.setPosition(0.9);
            telemetry.addData(">", "0.9");
            telemetry.update();
        }

        if(gamepad2.x) {
            bot.wobbleGrabber.setPosition(0);
            telemetry.addData(">", "0");
            telemetry.update();
        }
        if(gamepad2.y) {
            bot.wobbleGrabber.setPosition(0.9);
            telemetry.addData(">", "0.9");
            telemetry.update();
        }
        if(gamepad2.right_bumper) {
            bot.wobbleHook.setPosition(0);
            telemetry.addData(">", "0");
            telemetry.update();
        }
        if(gamepad2.left_bumper) {
            bot.wobbleHook.setPosition(1);
            telemetry.addData(">", "1");
            telemetry.update();
        }

    }

    @Override
    public void stop() {
        bot.stop();

    }
}