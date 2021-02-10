
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

    // Buffer area for controller triggers
    private double buffer = 0.15;

    // Position for the disc pusher arm (-1 is out and 1 is pushing)
    private int armPos = -1;

    // Position for the wobble hook (-1 is in and 1 is out)
    private int hookPos = -1;

    // Position for the wobble arm (-1 is back and 1 is forward)
    private int wobbleArmPos = -1;
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

        // Update all gamepad2 mechanisms
        updateShooter();
        updateIntake();
        updateDiscPlacer();
        updateWobble();
    }


    // Update's shooter power
    private void updateShooter(){
        double minPower = 0.0;
        double goalPower = -0.5;
        double pegPower = -0.6;

        // If left bumper is not pressed, bot shooter power set to 0
        if(!gamepad2.left_bumper){
            bot.shooter.setPower(minPower);
        }
        // If left bumper is pressed, add enough power to shoot into the top goal
        else if(gamepad2.left_bumper){
            bot.shooter.setPower(goalPower);
        }
        // If right bumper not pressed, do not do anything
        if(!gamepad2.right_bumper){
            bot.shooter.setPower(minPower);
        }
        // if right bumper is pressed down, add enough power to shoot the peg
        else if(gamepad2.right_bumper){
            bot.shooter.setPower(pegPower);
        }
    }

    // Updates intake power
    private void updateIntake(){
        // Max and min intake power
        double minIntakePower = 0.0;

        // Set intake power whatever percent the left trigger is pushed down
        if(gamepad2.left_trigger > buffer) {
            bot.intake.setPower(-gamepad2.left_trigger);
        }
        // Set outtake power to whatever percent the right trigger is pushed down
        else if(gamepad2.right_trigger > buffer){
            bot.intake.setPower(gamepad2.right_trigger);
        }
        // If neither trigger is pushed down, set the intake power to 0
        else{
            bot.intake.setPower(minIntakePower);
        }
    }

    // Updates disc pusher
    private void updateDiscPlacer(){
        // if a is pressed once, placer goes to opposite position and the variable
        //     controlling it gets multiplied by -1
        // Arm goes to pushing position if a is pressed an it is in
        if(gamepad2.a && armPos == -1){
            bot.discPlacer.setPosition(0.9);
            telemetry.addData(">", "0.9");
            telemetry.update();
            armPos *= -1;
        }

        // Arm goes to rest position if a is pressed an it is pushing
        if(gamepad2.a && armPos == 1){
            bot.discPlacer.setPosition(0.1);
            telemetry.addData(">", "0.1");
            telemetry.update();
            armPos *= -1;
        }

//        // If b is pressed, disc pusher is pushing
//        if(gamepad2.b) {
//            bot.discPlacer.setPosition(0.9);
//            telemetry.addData(">", "0.9");
//            telemetry.update();
//        }
    }

    // Updates wobble mechanism
    private void updateWobble(){
        // x-button on gamepad2 sets wobble hook position to in/out
        if(gamepad2.x && hookPos == -1) {
            bot.wobbleGrabber.setPosition(0);
            telemetry.addData(">", "0");
            telemetry.update();
        }
        else if(gamepad2.x && hookPos == 1) {
            bot.wobbleGrabber.setPosition(0.9);
            telemetry.addData(">", "0.9");
            telemetry.update();
        }

        // y-button on gamepad2 sets wobble arm position to in/out
        if(gamepad2.y && wobbleArmPos == -1) {
            bot.wobbleHook.setPosition(0);
            telemetry.addData(">", "0");
            telemetry.update();
        }
        else if(gamepad2.y && wobbleArmPos == 1) {
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