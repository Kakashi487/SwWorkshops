package frc.robot;

import edu.wpi.first.wpilibj.Timer; 


public class PIDF {
    private double mP,mI,mD,mF;
    private double mError, previous_error; 
    private double mSetpoint; 
    private double previous_time;
    double current_time  = Timer.getFPGATimestamp();
    


    public PIDF(double kP, double kI, double kD, double kF){
        this.mP = kP; 
        this.mI = kI; 
        this.mD = kD; 
        this.mF = kF; 
        
    }

    public double getError(){
        return mError;
    }

    public double getSetpoint(){
        return mSetpoint;  
    }
    
    public void setSetpoint(double setpoint){
        mSetpoint = setpoint ;
    }

    public double getP(){
        return mP; 
    }

    public void setP(double kP){
        mP = kP; 
    }
    public double getI(){
        return mI; 
    }

    public void setI(double kI){
        mI = kI; 
    }
    public double getD(){
        return mD; 
    }

    public void setD(double kD){
        mD = kD; 
    }
    public double getF(){
        return mF;
    }
    public void setF(double kF){
        mF = kF; 
    }


    
    public double update(double measured_value) throws InterruptedException{
        double integral = 0; 
        double derivative  = 0 ; 
        
        double output;
        double error = mSetpoint - measured_value;
        
        while (error != 0){ 
            long dt = (long)current_time - (long)previous_time; 
            error = mSetpoint - measured_value;
            integral = integral + error * dt ; 
            derivative = (error - previous_error) / dt; 
            output = mP * error + mI * integral + mD*derivative + mF * mSetpoint; 
            previous_error = error;
            wait(dt);
            previous_time = dt; 
            return output; 

        }

        return 0; 
        

    }






}

