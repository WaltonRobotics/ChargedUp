package frc.simAlgorithm;
import java.util.Arrays;
import java.util.Random;

public class PiecesGrid {
    private PieceType[] allowedTypeGrid;

    public PiecesGrid() {
        allowedTypeGrid = new PieceType[27];
        arrangePiecesArray();
    }

    public void arrangePiecesArray() {
        //Bottom Row
        allowedTypeGrid[0] = PieceType.CONE;
        allowedTypeGrid[1] = PieceType.CUBE;
        allowedTypeGrid[2] = PieceType.CONE;
        allowedTypeGrid[3] = PieceType.CONE;
        allowedTypeGrid[4] = PieceType.CUBE;
        allowedTypeGrid[5] = PieceType.CONE;
        allowedTypeGrid[6] = PieceType.CONE;
        allowedTypeGrid[7] = PieceType.CUBE;
        allowedTypeGrid[8] = PieceType.CONE;

        //Middle Row
        allowedTypeGrid[9]   = PieceType.CONE;
        allowedTypeGrid[10]  = PieceType.CUBE;
        allowedTypeGrid[11]  = PieceType.CONE;
        allowedTypeGrid[12]  = PieceType.CONE;
        allowedTypeGrid[13]  = PieceType.CUBE;
        allowedTypeGrid[14]  = PieceType.CONE;
        allowedTypeGrid[15]  = PieceType.CONE;
        allowedTypeGrid[16]  = PieceType.CUBE;
        allowedTypeGrid[17]  = PieceType.CONE;

        //Top Row
        allowedTypeGrid[18]  = PieceType.BOTH;
        allowedTypeGrid[19]  = PieceType.BOTH;
        allowedTypeGrid[20]  = PieceType.BOTH;
        allowedTypeGrid[21]  = PieceType.BOTH;
        allowedTypeGrid[22]  = PieceType.BOTH;
        allowedTypeGrid[23]  = PieceType.BOTH;
        allowedTypeGrid[24]  = PieceType.BOTH;
        allowedTypeGrid[25]  = PieceType.BOTH;
        allowedTypeGrid[26]  = PieceType.BOTH;
    }

    public PieceType[] getAllowedTypeGrid() {
        return allowedTypeGrid;
    }
}

