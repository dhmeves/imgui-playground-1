#pragma once
#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <random>
#include <chrono>

#include "json/json.h"



enum cellVals_e
{
    cellVals_1,
    cellVals_2,
    cellVals_3,
    cellVals_4,
    cellVals_5,
    cellVals_6,
    cellVals_7,
    cellVals_8,
    cellVals_9,
};


class Sudoku
{
public:
    const static int NUM_VALUES = 10; // we're counting 0 (blank) as a value here, since it's a state
    const static int NUM_ROWS = 9;
    const static int NUM_COLUMNS = 9;

    const static int NUM_ROWS_BOX = 3;
    const static int NUM_COLUMNS_BOX = 3;

    const char* fileExtDot = ".sudoku";	//	FILE EXTENSION FOR SUDOKU GAMES
    const char* fileExt = "sudoku";	//	FILE EXTENSION FOR SUDOKU GAMES

    struct gameVals_ts
    {
        int realVal; // value in the cell
        bool givenVal; // if we added the number as a given value at the start of the puzzle (Used for color effects)
        bool pencilledVals[NUM_VALUES];
    };

    gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS];

    bool CheckRow(gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS], int row, int column, int val);
    bool CheckRowPencilledVals(gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS], int row, int column, int val);
    bool CheckColumn(gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS], int row, int column, int val);
    bool CheckColumnPencilledVals(gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS], int row, int column, int val);
    bool CheckBox(gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS], int row, int column, int val);
    int CheckBoxRowPencilledVals(gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS], int row, int column);
    bool CheckBoxPencilledVals(gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS], int row, int column, int val);
    bool CheckCellSingletPencilledVal(gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS], int row, int column);
    bool CheckSingletPencilledVal(gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS]);
    bool PencilCell(gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS], int row, int column);
    bool PencilAllCells(gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS]);
    int FindBoxNum(int row, int column);
    bool CheckForDuplicateVals(gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS], int row, int column);
    bool SolveCellSimple(gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS], int row, int column, bool & pencilled);
    bool SolveSimple(gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS]);
    void SerializeSudokuGameData(gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS], Json::Value& sudokuJsonFile);
    void DeserializeSudokuGameData(Json::Value sudokuJsonFile, gameVals_ts gameVals_s[NUM_ROWS][NUM_COLUMNS]);
};
