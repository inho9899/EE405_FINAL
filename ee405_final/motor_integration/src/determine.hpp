#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <stdlib.h>
#include <cmath>
#include <unistd.h>

void printBoard(const std::vector<std::vector<std::string>> board);

std::vector<std::pair<int, int>> getAvailableMoves(std::vector<std::vector<std::string>> board);

std::string checkWinner(std::vector<std::vector<std::string>> board);

bool isBoardFull(const std::vector<std::vector<std::string>> board);

int minimax(std::vector<std::vector<std::string>> board, bool isMaximizing);

std::pair<int, int> findBestMove(std::vector<std::vector<std::string>> board, std::string who);

std::pair<int, int> determine();