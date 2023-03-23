package frc.simAlgorithm;

import java.util.Arrays;

public class PlacementGrid {
    public static boolean[] placement = new boolean[27];
    public static boolean[] possibleLink = new boolean[27];
    public static int link = 0;

    public static void scoreplacement(int place) {
        placement[place] = true;

    }

    public static void removeplacement(int place) {
        placement[place] = false;
    }

    public static boolean checkplacement(int place) {

        return placement[place];
    }


    public static void arrangePossibleLinks() {
        for (int i = 0; i < 9; i++) {
            if (placement[i] && placement[i + 1]) {
                //if (i < 9 - 2 && placement[i + 2]) {
                    //continue;
                //}
                if (i > 0) {
                    possibleLink[i - 1] = true;
                }
                if (i < 7) {
                    possibleLink[i + 2] = true;
                }
            }
            
        }

        for (int i = 9; i < 18; i++) {
            if (placement[i] && placement[i + 1]) {
                //if (i < 18 - 2 && placement[i + 2]) {
                    //continue;
                //}
                if (i > 9) {
                    possibleLink[i - 1] = true;
                }
                if (i < 16) {
                    possibleLink[i + 2] = true;
                }
            } 
        }

        for (int i = 18; i < 27; i++) {
            if (i < 26) {
                if (placement[i] && placement[i + 1]) {
                    //  if (i < 27 - 2 && placement[i + 2]) {
                      //    continue;
                      //}
                   if (i > 18) {
                      possibleLink[i - 1] = true;
                  }
                  if (i < 25) {
                      possibleLink[i + 2] = true;
                   }
              }
            }
            else {
                if (placement[i] && placement[i - 1]) {
                    possibleLink[i - 2] = true;
                }
            }

        }
    }
    

    public static boolean[] getLinks() {
        return possibleLink;
    }





    /**
     * THIS IS A TESTCASE ONLY
     */
    public static void main(String[] args) {

        
        placement = createTestPlacement();
        arrangePossibleLinks();
        //System.out.println(Arrays.toString(possibleLink));
        PriorityGrid.arrangePrioArray(possibleLink);
        int[] priority = PriorityGrid.getPriorityArray();
        //System.out.println(Arrays.toString(priority));
        Main.calculateNextBest(0, 0);
        //System.out.println(Arrays.toString(priority));
 
    }

    /**
     * Test values
     * @return
     */
    private static boolean[] createTestPlacement() {

        return new boolean[] {
            false, // Position 0
            false, // position 1
            false, // Position 2
            false, // position 3
            false, // Position 4
            false, // position 5
            false, // Position 6
            false, // position 7
            false, // Position 8
            false, // position 9
            false, // Position 10
            false, // position 11
            false, // Position 12
            false, // position 13
            false, // Position 14
            false, // position 15
            false, // Position 16
            false, // position 17
            false, // Position 18
            false, // position 19
            false, // Position 20
            false, // position 21
            false, // Position 22
            false, // position 23
            false, // Position 24
            false, // position 25
            false, // Position 26
            false // position 27
        };
    }

    
}
