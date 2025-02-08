#include "sudoku.h"
#include "TimersAndCalculations.h"

// returns true if val = realVal in given row 
bool Sudoku::CheckRow(gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS], int row, int val)
{
    for (int column = 0; column < NUM_COLUMNS; column++)
    {
        if (gameVals_s[row][column].realVal == val)
        {
            return true;
        }
    }
    return false;
}

// returns true if val = realVal in given column 
bool Sudoku::CheckColumn(gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS], int column, int val)
{
    for (int row = 0; row < NUM_ROWS; row++)
    {
        if (gameVals_s[row][column].realVal == val)
        {
            return true;
        }
    }
    return false;
}

// returns true if val = realVal in given box 
bool Sudoku::CheckBox(gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS], int row, int column, int val)
{
    int box = FindBoxNum(row, column);
    for (int rowi = NUM_ROWS_BOX * (row / NUM_ROWS_BOX); rowi < NUM_ROWS_BOX * (1 + row / NUM_ROWS_BOX); rowi++)
    {
        for (int columni = NUM_COLUMNS_BOX * (column / NUM_ROWS_BOX); columni < NUM_COLUMNS_BOX * (1 + column / NUM_COLUMNS_BOX); columni++)
        {
            if (gameVals_s[rowi][columni].realVal == val)
            {
                return true;
            }
        }
    }
    return false;
}

// returns box number given row and column... very crude but should work...
int Sudoku::FindBoxNum(int row, int column)
{
    return column / NUM_COLUMNS_BOX + 3 * (row / NUM_ROWS_BOX);
}

void Sudoku::PencilCell(gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS], int row, int column)
{
    for (int val = 0; val < NUM_VALUES; val++)
    {
        if (!CheckRow(gameVals_s, row, val) &&
            !CheckColumn(gameVals_s, column, val) &&
            !CheckBox(gameVals_s, row, column, val))
        {
            gameVals_s[row][column].pencilledVals[val] = true;
        }
        else
        {
            gameVals_s[row][column].pencilledVals[val] = false;
        }
    }
}

void Sudoku::PencilAllCells(gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS])
{
    for (int row = 0; row < NUM_ROWS; row++)
    {
        for (int column = 0; column < NUM_COLUMNS; column++)
        {
            PencilCell(gameVals_s, row, column);
        }
    }
}
