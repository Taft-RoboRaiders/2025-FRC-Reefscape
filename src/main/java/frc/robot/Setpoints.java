package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotation;

public class Setpoints {
    public static class Elevator
    {
        public static class Coral {
            public static final double L4 = 0.61;
            public static final double L3 = 0.315;
            public static final double L2 = 0.15;
            public static final double L1 = 0.001;

            
        }
        public static class Algae {
            public static final double L23 = 0.25;
            public static final double L34 = 0.45;
  
        }
    }

    public static class Algae
    {
        public static final double StowAngle = Degrees.of(-55).in(Rotation);
        public static final double GrabAlgaeAngle = Degrees.of(-25).in(Rotation);
        public static final double Floor  = Degrees.of(60).in(Rotation);

    }

}
