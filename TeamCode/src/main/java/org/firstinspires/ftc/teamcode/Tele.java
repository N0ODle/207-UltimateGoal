
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
    private double buffer = 0.15;
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

        // Update all gamepad2 mechanisms
        updateShooter();
        updateIntake();
        updateDiscPlacer();
        updateWobble();
    }


    // Update's shooter power
    private void updateShooter(){
        double minPower = 0.0;
        double lowPower = -0.5;
        double highPower = -0.9;

        bot.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // If right trigger is not pressed, do not run shooter (small buffer area of 0.15)
        if(gamepad2.right_trigger <= buffer) {
            bot.shooter.setPower(minPower);
        }
        // If right trigger is pressed anything in between 0 and full, set shooter power to 50%
        else if(gamepad2.right_trigger > buffer && gamepad2.right_trigger < 1) {
            bot.shooter.setPower(lowPower);
        }
        // If right trigger is pressed all the way, set shooter power to 90%
        else if(gamepad2.right_trigger == 1) {
            bot.shooter.setPower(highPower);
        }
    }

    // Updates intake power
    private void updateIntake(){
        // Max and min intake power
        double maxOuttakePower = 1.0;
        double minIntakePower = 0.0;

        // Set intake power whatever percent the left trigger is pushed down
        if(gamepad2.left_trigger > buffer) {
            bot.intake.setPower(-gamepad2.left_trigger);
        }

        // If x is pressed, outtake
        else if(gamepad2.x){
            bot.intake.setPower(maxOuttakePower);
        }

        else{
            bot.intake.setPower(minIntakePower);
        }
    }

    // Updates disc pusher
    private void updateDiscPlacer(){
        // If a is pressed, disc pusher is out (not pushing)
        if(gamepad2.a) {
            bot.discPlacer.setPosition(0.1);
            telemetry.addData(">", "0.1");
            telemetry.update();
        }

        // If b is pressed, disc pusher is pushing
        if(gamepad2.b) {
            bot.discPlacer.setPosition(0.9);
            telemetry.addData(">", "0.9");
            telemetry.update();
        }
    }

    // Updates wobble mechanism
    private void updateWobble(){
        // x-button on gamepad1 sets wobble grabber to forward position, and y-button sets to
        //     backwards position. Gamepad1 is used to give more space on gamepad2
        if(gamepad1.x) {
            bot.wobbleGrabber.setPosition(0);
            telemetry.addData(">", "0");
            telemetry.update();
        }

        // if y is pressed on gamepad1, the wobbleGrabber is in the backwards position
        if(gamepad1.y) {
            bot.wobbleGrabber.setPosition(0.9);
            telemetry.addData(">", "0.9");
            telemetry.update();
        }

        // Bumpers on pad 2 set position of small wobble hook that holds the wobble goal
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