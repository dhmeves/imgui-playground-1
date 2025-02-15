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
bool Sudoku::CheckRowPencilledVals (gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS], int row, int column, int val)
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
bool Sudoku::CheckColumnPencilledVals(gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS], int row, int column, int val)
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
    int box = FindBoxNum(row, column);
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
bool Sudoku::CheckBoxPencilledVals(gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS], int row, int column, int val)
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

// returns box number given row and column... very crude but should work...
int Sudoku::FindBoxNum(int row, int column)
{
    return column / NUM_COLUMNS_BOX + 3 * (row / NUM_ROWS_BOX);
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
    return true;
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


bool Sudoku::SolveCellSimple(gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS], int row, int column, bool & pencilled)
{
    if (!pencilled)
    {
        pencilled = PencilAllCells(gameVals_s); // make sure all of our pencilled vals are correct before trying to solve
    }
    for (int val = 0; val < NUM_VALUES; val++)
    {
        if (CheckRowPencilledVals(gameVals_s, row, column, val))
        {
            gameVals_s[row][column].realVal = val;
            pencilled = PencilAllCells(gameVals_s); // if we find that we can solve this cell, make sure to update pencilled values
            return true;
        }
        if (CheckColumnPencilledVals(gameVals_s, row, column, val))
        {
            gameVals_s[row][column].realVal = val;
            pencilled = PencilAllCells(gameVals_s);
            return true;
        }
        if (CheckBoxPencilledVals(gameVals_s, row, column, val))
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
