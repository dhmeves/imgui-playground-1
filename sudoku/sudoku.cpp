#include "sudoku.h"
#include "TimersAndCalculations.h"

// returns true if val = realVal in given row 
bool Sudoku::CheckRow(gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS], int row, int column, int val)
{
    for (int columni = 0; columni < NUM_COLUMNS; columni++)
    {
        if (columni == column)
        { // don't check ourselves 
        }
        else if (gameVals_s[row][columni].realVal == val)
        {
            return true;
        }
    }
    return false;
}

// returns true if given cell is the has the only pencilled value in given row 
bool Sudoku::CheckIfOnlyValInRowPencilled(gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS], int row, int column, int val)
{
    if (!gameVals_s[row][column].pencilledVals[val]) // Make sure cell is allowed to have val in it
    {
        return false;
    }
    if (CheckRow(gameVals_s, row, column, val)) // if the value is written in another cell in the same row, return false
    {
        return false;
    }

    for (int columni = 0; columni < NUM_COLUMNS; columni++) // Actually check row for pencilled values equaling val
    {
        if (columni == column)
        { // don't check ourselves 
        }
        else if (gameVals_s[row][columni].pencilledVals[val])
        {
            return false;
        }
    }
    return true;
}

// returns true if val = realVal in given column 
bool Sudoku::CheckColumn(gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS], int row, int column, int val)
{
    for (int rowi = 0; rowi < NUM_ROWS; rowi++)
    {
        if (rowi == row)
        {
        }
        else if (gameVals_s[rowi][column].realVal == val)
        {
            return true;
        }
    }
    return false;
}

// returns true if given cell is the has the only pencilled value in given column
bool Sudoku::CheckIfOnlyValInColumnPencilled(gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS], int row, int column, int val)
{
    if (!gameVals_s[row][column].pencilledVals[val]) // Make sure cell is allowed to have val in it
    {
        return false;
    }
    if (CheckColumn(gameVals_s, row, column, val)) // if the value is written in another cell in the same column, return false
    {
        return false;
    }

    for (int rowi = 0; rowi < NUM_ROWS; rowi++) // Actually check row for pencilled values equaling val
    {
        if (rowi == row)
        { // don't check ourselves 
        }
        else if (gameVals_s[rowi][column].pencilledVals[val])
        {
            return false;
        }
    }
    return true;
}

// returns true if val = realVal in given box 
bool Sudoku::CheckBox(gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS], int row, int column, int val)
{
    //int box = FindBoxNum(row, column);
    for (int rowi = NUM_ROWS_BOX * (row / NUM_ROWS_BOX); rowi < NUM_ROWS_BOX * (1 + row / NUM_ROWS_BOX); rowi++)
    {
        for (int columni = NUM_COLUMNS_BOX * (column / NUM_ROWS_BOX); columni < NUM_COLUMNS_BOX * (1 + column / NUM_COLUMNS_BOX); columni++)
        {
            if (rowi == row && columni == column)
            { // don't check ourselves 
            }
            else if (gameVals_s[rowi][columni].realVal == val)
            {
                return true;
            }
        }
    }
    return false;
}

// returns true if given cell is the has the only pencilled value in given column
bool Sudoku::CheckIfOnlyValInBoxPencilled(gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS], int row, int column, int val)
{
    if (!gameVals_s[row][column].pencilledVals[val]) // Make sure cell is allowed to have val in it
    {
        return false;
    }
    if (CheckBox(gameVals_s, row, column, val)) // if the value is written in another cell in the same column, return false
    {
        return false;
    }

    for (int rowi = NUM_ROWS_BOX * (row / NUM_ROWS_BOX); rowi < NUM_ROWS_BOX * (1 + row / NUM_ROWS_BOX); rowi++)
    {
        for (int columni = NUM_COLUMNS_BOX * (column / NUM_ROWS_BOX); columni < NUM_COLUMNS_BOX * (1 + column / NUM_COLUMNS_BOX); columni++)
        {
            if (rowi == row && columni == column)
            { // don't check ourselves 
            }
            else if (gameVals_s[rowi][columni].pencilledVals[val])
            {
                return false;
            }
        }
    }
    return true;
}

// returns true if only one pencilled value exists in a given cell, automatically assigns the realVal to the pencilled value
bool Sudoku::CheckCellSingletPencilledVal(gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS], int row, int column)
{
    int counter = 0;
    int singletVal = 0;
    for (int val = 0; val < NUM_VALUES; val++)
    {
        if (gameVals_s[row][column].pencilledVals[val])
        {
            counter++;
            singletVal = val;
        }
    }
    if (counter == 1)
    {
        gameVals_s[row][column].realVal = singletVal;
        return true;
    }
    return false;
}

// returns true if any singlets are found + converted
bool Sudoku::CheckSingletPencilledVal(gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS])
{
    bool foundSinglet = false;
    for (int row = 0; row < NUM_ROWS; row++)
    {
        for (int column = 0; column < NUM_COLUMNS; column++)
        {
            if (CheckCellSingletPencilledVal(gameVals_s, row, column))
            {
                foundSinglet = true;
                PencilAllCells(gameVals_s); // if we found a singlet, redo our pencil!
            }
        }
    }
    return foundSinglet;
}

// returns box number given row and column... very crude but should work...
int Sudoku::FindBoxNum(int row, int column)
{
    return column / NUM_COLUMNS_BOX + 3 * (row / NUM_ROWS_BOX);
}

// returns box row number given row... very crude but should work...
int Sudoku::FindBoxRowNum(int row)
{
    return (row / NUM_ROWS_BOX);
}

// returns box row number given row... very crude but should work...
int Sudoku::FindBoxColumnNum(int column)
{
    return (column / NUM_COLUMNS_BOX);
}

bool Sudoku::PencilCell(gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS], int row, int column)
{
    for (int val = 0; val < NUM_VALUES; val++)
    {
        if (!CheckRow(gameVals_s, row, column, val) &&
            !CheckColumn(gameVals_s, row, column, val) &&
            !CheckBox(gameVals_s, row, column, val) &&
            !gameVals_s[row][column].realVal && // if a real value exists, we cannot pencil a value
            val) // DO NOT PENCIL 0!
        {
            gameVals_s[row][column].pencilledVals[val] = true;
        }
        else
        {
            gameVals_s[row][column].pencilledVals[val] = false;
        }
    }
    return true;
}

bool Sudoku::PencilAllCells(gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS])
{
    for (int row = 0; row < NUM_ROWS; row++)
    {
        for (int column = 0; column < NUM_COLUMNS; column++)
        {
            PencilCell(gameVals_s, row, column);
        }
    }
    //CheckBoxRowPencilledVals(gameVals_s);
    //CheckBoxColumnPencilledVals(gameVals_s);
    return true;
}

int Sudoku::CheckBoxRowPencilledVals(gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS])
{
    int numRemovedPencils = 0;
    for (int val = 1; val < NUM_VALUES; val++)
    {
        for (int rowi = 0; rowi < NUM_ROWS; rowi++)
        {
            bool boxPencilled[3] = { 0 };
            for (int columni = 0; columni < NUM_COLUMNS; columni++)
            {
                int boxNum = -1;
                if (gameVals_s[rowi][columni].pencilledVals[val])
                {
                    boxNum = FindBoxColumnNum(columni);
                }
                if (boxNum != -1)
                {
                    boxPencilled[boxNum] = true;
                }
            }
            int boxSingletRowPencilled = ThreeWayXOR(boxPencilled[0], boxPencilled[1], boxPencilled[2]); // if only one box has a pencilled value, remove the value from the other rows in the same box
            int rowBox = FindBoxRowNum(rowi);
            if (boxSingletRowPencilled)
            {
                for (int boxRowi = (rowBox)*NUM_ROWS_BOX; boxRowi < (rowBox + 1) * NUM_ROWS_BOX; boxRowi++)
                {
                    for (int boxColumni = (boxSingletRowPencilled - 1) * NUM_COLUMNS_BOX; boxColumni < (boxSingletRowPencilled)*NUM_COLUMNS_BOX; boxColumni++)
                    {
                        if (boxRowi == rowi)
                        { // do nothing if we're in the row with the singlet row
                        }
                        else if (gameVals_s[boxRowi][boxColumni].pencilledVals[val])
                        {
                            gameVals_s[boxRowi][boxColumni].pencilledVals[val] = false;
                            numRemovedPencils++;
                        }
                    }
                }
            }
        }
    }
    return numRemovedPencils;
}

// CHECKS EACH BOX TO SEE IF A ROW IN A GIVEN BOX IS THE ONLY ROW WITH PENCILLED VALUES IN, IF IT IS, THE CELLS IN THE SAME ROW IN THE OTHER BOXES SHOULD NOT CONTAIN THAT VALUE
int Sudoku::CheckOutsideBoxRowPencilledVals(gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS])
{
    int numRemovedPencils = 0;
    for (int val = 1; val < NUM_VALUES; val++)
    {
        for (int boxNum = 0; boxNum < NUM_BOXES; boxNum++)
        {
            int boxStartRow = NUM_ROWS_BOX * (boxNum / NUM_BOXES_ROW);
            int boxStartColumn = NUM_COLUMNS_BOX * (boxNum % NUM_BOXES_COLUMN);
            int boxNumFoundPencilled = -1;
            bool boxRowPencilled[3] = { 0 };
            for (int rowi = boxStartRow; rowi < boxStartRow + NUM_ROWS_BOX; rowi++)
            {
                for (int columni = boxStartColumn; columni < boxStartColumn + NUM_COLUMNS_BOX; columni++)
                {
                    int rowNum = -1;
                    if (gameVals_s[rowi][columni].pencilledVals[val])
                    {
                        rowNum = rowi % NUM_BOXES_ROW; // Find out what row (inside the box) we are in.
                    }
                    if (rowNum != -1)
                    {
                        boxRowPencilled[rowNum] = true;
                        boxNumFoundPencilled = FindBoxNum(rowi, columni);
                    }
                }
            }
            int boxSingletRowPencilled = ThreeWayXOR(boxRowPencilled[0], boxRowPencilled[1], boxRowPencilled[2]);
            if (boxSingletRowPencilled)
            {
                for (int columni = 0; columni < NUM_COLUMNS; columni++)
                {
                    if (FindBoxNum(boxStartRow, columni) == boxNumFoundPencilled)
                    { // do nothing if we're in the same box as the detected singlet row
                    }
                    else if (gameVals_s[boxStartRow + (boxSingletRowPencilled - 1)][columni].pencilledVals[val])
                    {
                        gameVals_s[boxStartRow + (boxSingletRowPencilled - 1)][columni].pencilledVals[val] = false;
                        numRemovedPencils++;
                    }
                }
            }
        }
    }
    return numRemovedPencils;
}

int Sudoku::CheckBoxColumnPencilledVals(gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS])
{
    int numRemovedPencils = 0;
    for (int val = 1; val < NUM_VALUES; val++)
    {
        for (int columni = 0; columni < NUM_COLUMNS; columni++)
        {
            bool boxPencilled[3] = { 0 };
            for (int rowi = 0; rowi < NUM_ROWS; rowi++)
            {
                int boxNum = -1;
                if (gameVals_s[rowi][columni].pencilledVals[val])
                {
                    boxNum = FindBoxRowNum(rowi);
                }
                if (boxNum != -1)
                {
                    boxPencilled[boxNum] = true;
                }
            }
            int boxSingletColumnPencilled = ThreeWayXOR(boxPencilled[0], boxPencilled[1], boxPencilled[2]); // if only one box has a pencilled value, remove the value from the other rows in the same box
            int columnBox = FindBoxColumnNum(columni);
            if (boxSingletColumnPencilled)
            {
                for (int boxColumni = (columnBox)*NUM_COLUMNS_BOX; boxColumni < (columnBox + 1) * NUM_COLUMNS_BOX; boxColumni++)
                {
                    for (int boxRowi = (boxSingletColumnPencilled - 1) * NUM_ROWS_BOX; boxRowi < (boxSingletColumnPencilled)*NUM_ROWS_BOX; boxRowi++)
                    {
                        if (boxColumni == columni)
                        { // do nothing if we're in the row with the singlet row
                        }
                        else if (gameVals_s[boxRowi][boxColumni].pencilledVals[val])
                        {
                            gameVals_s[boxRowi][boxColumni].pencilledVals[val] = false;
                            numRemovedPencils++;
                        }
                    }
                }
            }
        }
    }
    return numRemovedPencils;
}

// CHECKS EACH BOX TO SEE IF A COLUMN IN A GIVEN BOX IS THE ONLY COLUMN WITH PENCILLED VALUES IN, IF IT IS, THE CELLS IN THE SAME COLUMN IN THE OTHER BOXES SHOULD NOT CONTAIN THAT VALUE
int Sudoku::CheckOutsideBoxColumnPencilledVals(gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS])
{
    int numRemovedPencils = 0;
    for (int val = 1; val < NUM_VALUES; val++)
    {
        for (int boxNum = 0; boxNum < NUM_BOXES; boxNum++)
        {
            int boxStartRow = NUM_ROWS_BOX * (boxNum / NUM_BOXES_ROW);
            int boxStartColumn = NUM_COLUMNS_BOX * (boxNum % NUM_BOXES_COLUMN);
            int boxNumFoundPencilled = -1;
            bool boxRowPencilled[3] = { 0 };
            for (int columni = boxStartColumn; columni < boxStartColumn + NUM_COLUMNS_BOX; columni++)
            {
                for (int rowi = boxStartRow; rowi < boxStartRow + NUM_ROWS_BOX; rowi++)
                {
                    int columnNum = -1;
                    if (gameVals_s[rowi][columni].pencilledVals[val])
                    {
                        columnNum = columni % NUM_BOXES_ROW; // Find out what row (inside the box) we are in.
                    }
                    if (columnNum != -1)
                    {
                        boxRowPencilled[columnNum] = true;
                        boxNumFoundPencilled = FindBoxNum(rowi, columni);
                    }
                }
            }
            int boxSingletColumnPencilled = ThreeWayXOR(boxRowPencilled[0], boxRowPencilled[1], boxRowPencilled[2]);
            if (boxSingletColumnPencilled)
            {
                for (int rowi = 0; rowi < NUM_ROWS; rowi++)
                {
                    if (FindBoxNum(rowi, boxStartColumn) == boxNumFoundPencilled)
                    { // do nothing if we're in the same box as the detected singlet row
                    }
                    else if (gameVals_s[rowi][boxStartColumn + (boxSingletColumnPencilled - 1)].pencilledVals[val])
                    {
                        gameVals_s[rowi][boxStartColumn + (boxSingletColumnPencilled - 1)].pencilledVals[val] = false;
                        numRemovedPencils++;
                    }
                }
            }
        }
    }
    return numRemovedPencils;
}

// TODO - RM: NOT FINSHED YET! Checks for hidden pairs in a given row - a slightly more complex technique for whittling away pencil values
int Sudoku::CheckRowHiddenPair(gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS])
{
    // first check if hidden pair only occur in two of the same cells
    for (int rowi = 0; rowi < NUM_ROWS; rowi++)
    {
        bool hiddenPairFound = false;
        int pairVals[NUM_VALUES] = { 0 };
        for (int val = 1; val < NUM_VALUES; val++)
        {
            int count = 0;
            for (int columni = 0; columni < NUM_COLUMNS; columni++)
            {
                if (gameVals_s[rowi][columni].pencilledVals[val])
                {
                    pairVals[val] = 1 < columni; // bitshift to the column number pencilled values are in for easy comparison later
                }
            }
            if (NumberOfSetBits(pairVals[val] != NUM_PAIR))
            {
                pairVals[val] = 0; // if it's not a pair we don't want to keep the column data
            }
        }
        for (int val = 1; val < NUM_VALUES; val++)
        {
            if (pairVals[val] == NUM_PAIR)
            {
                for (int val2 = val + 1; val2 < NUM_VALUES; val2++) // start val2 one after val since we don't want to check when val2 = val
                {
                    if (pairVals[val] == pairVals[val2])
                    {
                        hiddenPairFound = true; // if the two pairVals match, that means we found a hidden pair in this row
                    }
                }
            }
        }
    }
    // second check if hidden pair are the only pencilled values available in two cells
    return 0;
}

// Checks for duplicate values in the same row, column, and box as the given cell (for error checking)
bool Sudoku::CheckForDuplicateVals(gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS], int row, int column)
{
    if (gameVals_s[row][column].realVal) // only check if our value isn't 0 (no value)
    {
        if (CheckRow(gameVals_s, row, column, gameVals_s[row][column].realVal) ||
            CheckColumn(gameVals_s, row, column, gameVals_s[row][column].realVal) ||
            CheckBox(gameVals_s, row, column, gameVals_s[row][column].realVal))
        {
            return true;
        }
    }
    return false;
}


bool Sudoku::SolveCellSimple(gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS], int row, int column, bool& pencilled)
{
    if (!pencilled)
    {
        pencilled = PencilAllCells(gameVals_s); // make sure all of our pencilled vals are correct before trying to solve
    }
    CheckSingletPencilledVal(gameVals_s);
    for (int val = 0; val < NUM_VALUES; val++)
    {
        if (CheckIfOnlyValInRowPencilled(gameVals_s, row, column, val))
        {
            gameVals_s[row][column].realVal = val;
            pencilled = PencilAllCells(gameVals_s); // if we find that we can solve this cell, make sure to update pencilled values
            return true;
        }
        if (CheckIfOnlyValInColumnPencilled(gameVals_s, row, column, val))
        {
            gameVals_s[row][column].realVal = val;
            pencilled = PencilAllCells(gameVals_s);
            return true;
        }
        if (CheckIfOnlyValInBoxPencilled(gameVals_s, row, column, val))
        {
            gameVals_s[row][column].realVal = val;
            pencilled = PencilAllCells(gameVals_s);
            return true;
        }
    }
    return false;
}

bool Sudoku::SolveSimple(gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS])
{
    bool solved = false; // if ANY cells are solved, return true
    bool pencilled = PencilAllCells(gameVals_s);
    for (int row = 0; row < NUM_ROWS; row++)
    {
        for (int column = 0; column < NUM_COLUMNS; column++)
        {
            if (SolveCellSimple(gameVals_s, row, column, pencilled))
            {
                solved = true;
            }
        }
    }
    return solved;
}

void Sudoku::SerializeSudokuGameData(gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS], Json::Value& sudokuJsonFile)
{
    for (int row = 0; row < NUM_ROWS; row++)
    {
        for (int column = 0; column < NUM_COLUMNS; column++)
        {
            std::string rowName = "row" + std::to_string(row);
            std::string columnName = "col" + std::to_string(column);
            sudokuJsonFile[rowName][columnName]["realVal"] = gameVals_s[row][column].realVal;
            sudokuJsonFile[rowName][columnName]["givenVal"] = gameVals_s[row][column].givenVal;
        }
    }
}

void Sudoku::DeserializeSudokuGameData(Json::Value sudokuJsonFile, gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS])
{
    for (int row = 0; row < NUM_ROWS; row++)
    {
        for (int column = 0; column < NUM_COLUMNS; column++)
        {
            std::string rowStr = "row" + std::to_string(row);
            std::string columnStr = "col" + std::to_string(column);
            if (sudokuJsonFile[(rowStr.c_str())][(columnStr.c_str())].isMember("realVal"))
            {
                gameVals_s[row][column].realVal = sudokuJsonFile[(rowStr.c_str())][(columnStr.c_str())]["realVal"].asInt();
            }
            if (sudokuJsonFile[(rowStr.c_str())][(columnStr.c_str())].isMember("givenVal"))
            {
                gameVals_s[row][column].givenVal = sudokuJsonFile[(rowStr.c_str())][(columnStr.c_str())]["givenVal"].asBool();
            }
        }
    }
}
