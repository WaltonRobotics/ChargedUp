package frc.simAlgorithm;
import java.util.Random;


public class PointGrid {
    public static int[] pointValues = new int[27];
    static {
       arrangePointsArray();
    }

    private static void arrangePointsArray() {
        for(int i = 0; i < 27; i++) {
                if(i < 9) {
                    pointValues[i] = 3;
                } else if(i < 18) {
                    pointValues[i] = 2;
                } else {
                    pointValues[i] = 1;
            }
        }
    }

    public static int getPointValues(int index) {
        return pointValues[index];
    }
    
}
