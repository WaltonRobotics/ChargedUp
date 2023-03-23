package frc.simAlgorithm;
import java.util.Arrays;

public class Main {
    public static int calculateNextBest(int position, int condition) {
        PlacementGrid placementGrid = new PlacementGrid();
        int prio;
        boolean[] links;
        
        
        int bestPlace = 0;
        int bestPlacePlace = 0;

        if (condition == 1) { //if the condition returned is 1, then the value of the position returned is true
            PlacementGrid.scoreplacement(position); //places piece in that grid spot
            PlacementGrid.arrangePossibleLinks(); //check for links present
            links = PlacementGrid.getLinks(); //updates the link array
            PriorityGrid.arrangePrioArray(links); //updates priority based on links
            bestPlace = 0;


            for(int space = 0; space < 27; space++) { //scans through every spot
                prio = PriorityGrid.getPriority(space); //gets the priority of that spot
                if (prio >= bestPlace // if the position has the highest priority and is not scored
                    && placementGrid.checkplacement(space) == false) {
                    bestPlace = prio;
                    bestPlacePlace = space;
                }
            }
          } else { //if the condition returned is 0, then remove the piece in that position and redo the calculations
            PlacementGrid.removeplacement(position);
            PlacementGrid.arrangePossibleLinks();
            links = PlacementGrid.getLinks();
            PriorityGrid.arrangePrioArray(links);
            bestPlace = 0;


            for(int space = 0; space < 27; space++) {
                prio = PriorityGrid.getPriority(space);
                if (prio >= bestPlace 
                    && placementGrid.checkplacement(space) == false) {
                    bestPlace = prio;
                    bestPlacePlace = space;
                }
            }
          }

        System.out.println(bestPlacePlace);
        System.out.println(Arrays.toString(PriorityGrid.priority));
        return bestPlacePlace;

    }
    
    


}