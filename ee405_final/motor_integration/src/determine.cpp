#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <stdlib.h>
#include <cmath>
#include <unistd.h>

using namespace std;

const string PLAYER = "red";
const string OPPONENT = "blue";

// 보드 출력 함수
void printBoard(const vector<vector<string>> board){
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            cout << board[i][j] << " ";
        }
        cout << "\n";
    }
}

// 보드에서 남은 빈 칸을 반환
vector<pair<int, int>> getAvailableMoves(vector<vector<string>> board){
    vector<pair<int, int>> moves;
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            if(board[i][j] == "none"){
                moves.push_back(make_pair(i, j));
            }
        }
    }
    return moves;
}

// 승리 조건을 확인하는 함수
string checkWinner(vector<vector<string>> board){
    for(int i = 0; i < 3; i++){
        // three in a row
        if(board[i][0] == board[i][1] && board[i][1] == board[i][2]){
            if(board[i][0] == PLAYER) return "red";
            else if(board[i][0] == OPPONENT) return "blue";
        }
        // three in a column
        if(board[0][i] == board[1][i] && board[1][i] == board[2][i]){
            if(board[0][i] == PLAYER) return "red";
            else if(board[0][i] == OPPONENT) return "blue";
        }
        // three in a diagonal
        if(board[0][0] == board[1][1] && board[1][1] == board[2][2]){
            if(board[0][0] == PLAYER) return "red";
            else if(board[0][0] == OPPONENT) return "blue";
        }
        if(board[0][2] == board[1][1] && board[1][1] == board[2][0]){
            if(board[0][2] == PLAYER) return "red";
            else if(board[0][2] == OPPONENT) return "blue";
        }
    }
    return "none";
}

// 보드가 꽉 찼는지 확인
bool isBoardFull(const vector<vector<string>> board) {
    return getAvailableMoves(board).empty();
}

// Minimax 알고리즘
int minimax(vector<vector<string>> board, bool isMaximizing) {
    string winner = checkWinner(board);
    if (winner == PLAYER) return 10;
    if (winner == OPPONENT) return -10;
    if (isBoardFull(board)) return 0;

    if (isMaximizing) {
        int bestScore = -100000;
        for (const auto& move : getAvailableMoves(board)) {
            board[move.first][move.second] = PLAYER;
            bestScore = max(bestScore, minimax(board, false));
            board[move.first][move.second] = "none";
        }
        return bestScore;
    } else {
        int bestScore = 100000;
        for (const auto& move : getAvailableMoves(board)) {
            board[move.first][move.second] = OPPONENT;
            bestScore = min(bestScore, minimax(board, true));
            board[move.first][move.second] = "none";
        }
        return bestScore;
    }
}

pair<int, int> findBestMove(vector<vector<string>> board, string who) {
    int bestScore = (who == PLAYER) ? -100000 : 100000; // 최대화/최소화 기준
    pair<int, int> bestMove = {-1, -1};

    for (const auto& move : getAvailableMoves(board)) {
        board[move.first][move.second] = who; // 현재 플레이어의 움직임 시뮬레이션

        // Minimax 호출
        int currentScore = minimax(board, (who == PLAYER) ? false : true);

        // 보드 원상 복구
        board[move.first][move.second] = "none";

        // 최적의 점수 및 움직임 업데이트
        if (who == PLAYER) {
            if (currentScore > bestScore) {
                bestScore = currentScore;
                bestMove = move;
            }
        } else {
            if (currentScore < bestScore) {
                bestScore = currentScore;
                bestMove = move;
            }
        }
    }

    board[bestMove.first][bestMove.second] = who;

    // save in file of board
    FILE* file = fopen("board.txt", "w");
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            fprintf(file, "%s ", board[i][j].c_str());
        }
        fprintf(file, "\n");
    }
    fclose(file);

    return bestMove;
}


pair<int, int> determine() {
    vector<vector<string>> board;
    FILE* pipe = popen("python3 ../src/yolo2py.py", "r");

    char buffer[128];
    string result = "";

    // Python 출력 읽기
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        result += buffer;
    }

    pclose(pipe);

    // 결과를 파싱하여 vector<vector<string>>로 변환
    vector<vector<string>> matrix;
    stringstream ss(result);
    string line;

    while (getline(ss, line)) {  // 각 행을 읽음
        stringstream row_stream(line);
        vector<string> row;
        string value;

        while (row_stream >> value) {  // 각 값을 읽어 vector<string>에 추가
            row.push_back(value);
        }

        matrix.push_back(row);
    }

    board = matrix;
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            cout << board[i][j] << " ";
        }
        cout << "\n";
    }

    string who = "red";

    pair<int, int> bestMove = findBestMove(board, who);

    cout << bestMove.first << " " << bestMove.second << endl;
    return bestMove;
}

