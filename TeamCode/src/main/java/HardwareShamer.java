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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static android.os.SystemClock.sleep;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Shamer.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 *
 * RYAN THIS NEEDS UPDATED!!! RYAN THIS NEEDS UPDATED!!! RYAN THIS NEEDS UPDATED!!! RYAN THIS NEEDS UPDATED!!!
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareShamer
{

    private ElapsedTime runtime = new ElapsedTime();



    /* Public OpMode members. */

    //nameing drive system motors
    public DcMotor  leftDriveFront   = null;
    public DcMotor  leftDriveBack  = null;
    public DcMotor  rightDriveFront  = null;
    public DcMotor  rightDriveBack = null;


    //naming non-drive system motors
    public DcMotor collectionMotor = null;
    public DcMotor collectorDeployMotor = null;
    public DcMotor liftMotor = null;

    //naming the team marker servos
    public Servo markerArm = null;
    public Servo markerDrop = null;

    // Store and Release positions for the team marker Arm servo
    public double markerArmStore = 0;
    public double markerArmRelease = 1;

    // Store and Release positions for the team marker Drop servo
    public double markerDropStore = 0;
    public double markerDropRelease = 1;

    //threshold value for joysticks
    public double threshold = 0.1;



    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareShamer(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Drive Motors
        leftDriveFront  = hwMap.get(DcMotor.class, "left_drive1");
        leftDriveBack =  hwMap.get(DcMotor.class, "left_drive2");
        rightDriveFront = hwMap.get(DcMotor.class, "right_drive1");
        rightDriveBack = hwMap.get(DcMotor.class, "right_drive2");

        // define and initialize collector motor
        collectionMotor = hwMap.get(DcMotor.class, "collection_Motor");
        collectorDeployMotor = hwMap.get(DcMotor.class,"collector_deploy_motor");

        //initialize servos for the marker
        markerArm = hwMap.get(Servo.class, "marker_extend");
        markerDrop = hwMap.get(Servo.class, "marker_drop");

        //set direction of drive motors
        leftDriveFront.setDirection(DcMotor.Direction.FORWARD);
        leftDriveBack.setDirection(DcMotor.Direction.FORWARD);
        rightDriveFront.setDirection(DcMotor.Direction.REVERSE);
        rightDriveBack.setDirection(DcMotor.Direction.REVERSE);


        // Set all drive motors to zero power
        leftDriveFront.setPower(0);
        leftDriveBack.setPower(0);
        rightDriveFront.setPower(0);
        rightDriveBack.setPower(0);

        // set initial collector motors power to zero
        collectionMotor.setPower(0);
        collectorDeployMotor.setPower(0);

        // set team marker servo starting positions (store)
        markerArm.setPosition(markerArmStore);
        markerDrop.setPosition(markerDropStore);

    }

    //function to control the right side motors of the robot
    public void leftDrive(double power){
        leftDriveFront.setPower(power);
        leftDriveBack.setPower(power);
    }

    //function to control the right side motors of the robot
    public void rightDrive(double power){
        rightDriveFront.setPower(power);
        rightDriveBack.setPower(power);
    }

    //extend lift
    public void extendLift(double power){
        liftMotor.setPower(power);
    }

    //retract lift
    public void retractLift(double power){
        liftMotor.setPower(power);
    }

    //deploy the collector using collectorDeployMotor
    public void deployCollector(double power){
        collectorDeployMotor.setPower(power);
    }

    //retract the collector using collectorDeployMotor
    public void retractCollector(double power){
        collectorDeployMotor.setPower(power);
    }

    // moves the marker into the release position and then releases the marker
    public void releaseMarker(){
        markerArm.setPosition(markerArmRelease);
        sleep(250);
        markerDrop.setPosition(markerDropRelease);
    }

    // moves both marker servos into the stored position (ma
    public void storeMarker (){
        markerDrop.setPosition(markerDropStore);
        sleep(500);
        markerArm.setPosition(markerArmStore);
    }



}

