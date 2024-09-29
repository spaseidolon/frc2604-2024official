package frc.lib.math.conversions;

public class Conversions {

 public static double degreesToRadians(double degrees)
 {
    double radians = (degrees * 0.017444);
    return radians;
 }

//==========================================================================================
//
//    CTRE CONVERSIONS GO BELOW
//
//==========================================================================================
public static class CtreConversions extends Conversions {
   /**
* @param counts motor Counts
* @param gearRatio Gear Ratio between motor and Mechanism
* @return Degrees of Rotation of Mechanism
*/
public static double motorToDegrees(double counts, double gearRatio) {
   return counts * (360.0 / (gearRatio * 2048.0));
}

/**
* @param degrees Degrees of rotation of Mechanism
* @param gearRatio Gear Ratio between motor and Mechanism
* @return motor Counts
*/
public static double degreesTomotor(double degrees, double gearRatio) {
   double ticks =  degrees / (360.0 / (gearRatio * 2048.0));
   return ticks;
}

/**
* @param velocityCounts motor Velocity Counts
* @param gearRatio Gear Ratio between motor and Mechanism (set to 1 for motor RPM)
* @return RPM of Mechanism
*/
public static double motorToRPM(double velocityCounts, double gearRatio) {
   double motorRPM = velocityCounts * (600.0 / 2048.0);        
   double mechRPM = motorRPM / gearRatio;
   return mechRPM;
}

/**
* @param RPM RPM of mechanism
* @param gearRatio Gear Ratio between motor and Mechanism (set to 1 for motor RPM)
* @return RPM of Mechanism
*/
public static double RPMTomotor(double RPM, double gearRatio) {
   double motorRPM = RPM * gearRatio;
   double sensorCounts = motorRPM * (2048.0 / 600.0);
   return sensorCounts;
}

/**
* @param velocitycounts motor Velocity Counts
* @param circumference Circumference of Wheel
* @param gearRatio Gear Ratio between motor and Mechanism (set to 1 for motor RPM)
* @return motor Velocity Counts
*/
public static double motorToMPS(double velocitycounts, double circumference, double gearRatio){
   double wheelRPM = motorToRPM(velocitycounts, gearRatio);
   double wheelMPS = (wheelRPM * circumference) / 60;
   return wheelMPS;
}

/**
* @param velocity Velocity MPS
* @param circumference Circumference of Wheel
* @param gearRatio Gear Ratio between motor and Mechanism (set to 1 for motor RPM)
* @return motor Velocity Counts
*/
public static double MPSTomotor(double velocity, double circumference, double gearRatio){
   double wheelRPM = ((velocity * 60) / circumference);
   double wheelVelocity = RPMTomotor(wheelRPM, gearRatio);
   return wheelVelocity;
}
}

//==========================================================================================
//
//    PWF CONVERSIONS GO BELOW
//
//==========================================================================================

public static class PwFConversions extends Conversions {
   /**
* @param counts motor Counts
* @param gearRatio Gear Ratio between motor and Mechanism
* @return Degrees of Rotation of Mechanism
*/
public static double motorToDegrees(double counts, double gearRatio) {
   return counts * (360.0 / (gearRatio));
}

/**
* @param degrees Degrees of rotation of Mechanism
* @param gearRatio Gear Ratio between motor and Mechanism
* @return motor Counts
*/
public static double degreesTomotor(double degrees, double gearRatio) {
   double ticks =  degrees / (360.0 / (gearRatio));
   return ticks;
}

/**
* @param velocityCounts motor Velocity Counts
* @param gearRatio Gear Ratio between motor and Mechanism (set to 1 for motor RPM)
* @return RPM of Mechanism
*/
public static double motorToRPM(double velocityCounts, double gearRatio) {
   double motorRPM = velocityCounts;        
   double mechRPM = motorRPM / gearRatio;
   return mechRPM;
}

/**
* @param RPM RPM of mechanism
* @param gearRatio Gear Ratio between motor and Mechanism (set to 1 for motor RPM)
* @return RPM of Mechanism
*/
public static double RPMTomotor(double RPM, double gearRatio) {
   double motorRPM = RPM * gearRatio;
   double sensorCounts = (motorRPM);
   return sensorCounts;
}

/**
* @param velocitycounts motor Velocity Counts
* @param circumference Circumference of Wheel
* @param gearRatio Gear Ratio between motor and Mechanism (set to 1 for motor RPM)
* @return motor Velocity Counts
*/
public static double motorToMPS(double velocitycounts, double circumference, double gearRatio){
   double wheelRPM = motorToRPM(velocitycounts, gearRatio);
   double wheelMPS = (wheelRPM * circumference) / 60;
   return wheelMPS;
}

/**
* @param velocity Velocity MPS
* @param circumference Circumference of Wheel
* @param gearRatio Gear Ratio between motor and Mechanism (set to 1 for motor RPM)
* @return motor Velocity Counts
*/
public static double MPSTomotor(double velocity, double circumference, double gearRatio){
   double wheelRPM = ((velocity * 60) / circumference);
   double wheelVelocity = RPMTomotor(wheelRPM, gearRatio);
   return wheelVelocity;
}
}


}