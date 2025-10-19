package frc.robot.util.Tuning;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.robot.Constants;

public class TunedDouble {
     private final LoggedNetworkNumber networkNumber;

     private double defaultNumber;
      public TunedDouble(double defaultNumber, String subsystem, String key){
        this.defaultNumber = defaultNumber;
        this.networkNumber = new LoggedNetworkNumber("/Tuning/"+subsystem+"/"+key, defaultNumber);
      }

      public double getValue(){
        if(Constants.enableNTTuning){
            return networkNumber.get();
        }else{
            return defaultNumber; 
        }
      }
}
