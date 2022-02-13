package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Constants;

import java.lang.Math;

public class Drive extends Subsystem{
    // Instantiate 4 TalonFX motors 
    private TalonFX r1master = new TalonFX(1);
    private TalonFX r2slave = new TalonFX(2);
    private TalonFX l1master = new TalonFX(3);
    private TalonFX l2slave = new TalonFX(4); 

    public Drive () {
        r2slave.set(ControlMode.Follower, 1);
        l2slave.set(ControlMode.Follower, 3);

        // config absolute encoders for Master talons 

        r1master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, Constants.kpidIDX, Constants.ktimeoutMs); 
        l1master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, Constants.kpidIDX, Constants.ktimeoutMs);
    }
    

    public void setOpenLoop (double throttle, double turn){
        // m is an instance variable that can change basically not a constant "k"
        double mRight_command = throttle + turn; 
        double mLeft_command = throttle - turn; 

        
        // a /= b will be --  new a value : a = a/b 
        // This block of code will make it so that the commanded motor velocity will not exceed 1.0 (maximum motor velocity)
        if (Math.abs(mRight_command)> 1.0 || Math.abs(mLeft_command)> 1.0){
            mRight_command /= Math.max(Math.abs(mRight_command), Math.abs(mLeft_command)); 
            mLeft_command /= Math.max(Math.abs(mRight_command), Math.abs(mLeft_command)); 

        }

        r1master.set(ControlMode.PercentOutput, mRight_command); 
        l1master.set(ControlMode.PercentOutput, mLeft_command); 
    }


    public void stop(){
        // only "commands" master motors to stop because the slave motors will copy the master
        l1master.set(ControlMode.PercentOutput, 0);
        r1master.set(ControlMode.PercentOutput, 0);
    }
}


