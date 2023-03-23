package frc.simAlgorithm;


public class PriorityGrid extends PointGrid{ 

    public static PieceType[] pieceType = (new PiecesGrid()).getAllowedTypeGrid();
    public static int linkPrio;
    public static int [] priority = new int[27];


    public static void arrangePrioArray(boolean[] links) {

        for(int i = 0; i < 27; i++) {
            int points = PointGrid.getPointValues(i); // reference the point values of each space

            if(points == Constants.top) { // If the point value is that of a top space
                if (links[i]) {
                    priority[i] = Constants.linkWeight * 3;
                }
                
                else if (pieceType[i] == PieceType.CONE) {
                    priority[i] = (3 * Constants.heightWeight)
                                    + (Constants.topCone * Constants.timeWeight)
                                    + (Constants.top * Constants.pointWeight);
                } else if (pieceType[i] == PieceType.CUBE) {
                    priority[i] = (3 * Constants.heightWeight)
                                    + (Constants.topCube * Constants.timeWeight)
                                    + (Constants.top * Constants.pointWeight);
                } else {System.out.println("AHHHH");}
        
                
            } else if(points == Constants.mid) {
                if (links[i]) {
                    priority[i] = Constants.linkWeight * 2;
                }
                 else if(pieceType[i] == PieceType.CONE) {
                    priority[i] = (2 * Constants.heightWeight)
                                    + (Constants.midCone * Constants.timeWeight)
                                    + (Constants.mid * Constants.pointWeight);
                } else if (pieceType[i] == PieceType.CUBE) {
                    priority[i] = (2 * Constants.heightWeight)
                                    + (Constants.midCube * Constants.timeWeight)
                                    + (Constants.mid * Constants.pointWeight);
                }
            } else {
                if (links[i]) {
                priority[i] = Constants.linkWeight;

                } else priority[i] = (1 * Constants.heightWeight)
                                    + (Constants.botTime * Constants.timeWeight)
                                    + (Constants.bot * Constants.pointWeight); 

            }

        }
    }

    public static int getPriority(int place) {
        return priority[place];
    }

    public static int[] getPriorityArray() {
        return priority;
    }
}






