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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Shamer: Teleop POV", group="Shamer")

public class ShamerTeleop extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareShamer robot           = new HardwareShamer();   // Use a Pushbot's hardware
                                                               // could also use HardwarePushbotMatrix class.
         // sets rate to move servo

    @Override
    public void runOpMode() {

        double left;
        double right;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            //****************** GAMEPAD 1 CONTROLS ********************//

            // sets left and right values to gamepad1 joysticks
            left = gamepad1.left_stick_y;
            right = gamepad1.right_stick_y;


            // power left motors if absolute value of the joystick is greater than the threshold value
            if((Math.abs(left)) > robot.threshold){
                robot.leftDrive(left);
            }

            // power right motors if absolute value of the joystick is greater than the threshold value
            if((Math.abs(right)) > robot.threshold){
                robot.rightDrive(right);
            }


            // Set's the control of the lift to GAMEPAD 1, left and right trigger

            //extend lift arm
            if(gamepad1.left_trigger > robot.threshold){
                robot.extendLift(gamepad1.left_trigger);
            }

            //retract lift arm
            if(gamepad1.right_trigger > robot.threshold){
                robot.retractLift(gamepad1.right_trigger);
            }





            //****************** GAMEPAD 2 CONTROLS ********************//

            // deploy the collection system with gamepad 2 button a
            if(gamepad2.a){
                robot.deployCollector(1.0);
            }

            //retract the collection system with gamepad 2 button b
            if(gamepad2.b){
                robot.retractCollector(-1.0);
            }

            //control for the collection system MINERALS IN - collection motor (halls of sadness)
            if(gamepad1.right_bumper){
                robot.collectorIn(1.0);
            }

            //control for the collection system MINERALS OUT - collection motor (halls of sadness)
            if(gamepad1.left_bumper){
                robot.collectorOut(-1.0);
            }

            //control for the collection system sorter



            //control for the mineral launching system



        }
    }
}
