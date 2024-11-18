#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <cstdio>

using namespace std;

string USER_CHAR = "red";
string COMPUTER_CHAR = "blue";

vector<vector<string>> board = {
    {"none", "none", "none"},
    {"none", "none", "none"},
    {"none", "none", "none"}
};  // 'none' : empty, USER_CHAR : user, COMPUTER_CHAR : computer

pair<int, int> determine_gameset(vector<vector<string>> board){
    int i, j;
    int count = 0;
    for(i = 0; i < 3; i++){
        for(j = 0; j < 3; j++){
            if(board[i][j] == "none") count++;
        }
    }

    for(i = 0; i < 3; i++){
        // three in a row
        if(board[i][0] == board[i][1] && board[i][1] == board[i][2]){
            if(board[i][0] == USER_CHAR) return make_pair(-1, count);
            else if(board[i][0] == COMPUTER_CHAR) return make_pair(1, count);
        }
        // three in a column
        if(board[0][i] == board[1][i] && board[1][i] == board[2][i]){
            if(board[0][i] == USER_CHAR) return make_pair(-1, count);
            else if(board[0][i] == COMPUTER_CHAR) return make_pair(1, count);
        }
        // three in a diagonal
        if(board[0][0] == board[1][1] && board[1][1] == board[2][2]){
            if(board[0][0] == USER_CHAR) return make_pair(-1, count);
            else if(board[0][0] == COMPUTER_CHAR) return make_pair(1, count);
        }
        if(board[0][2] == board[1][1] && board[1][1] == board[2][0]){
            if(board[0][2] == USER_CHAR) return make_pair(-1, count);
            else if(board[0][2] == COMPUTER_CHAR) return make_pair(1, count);
        }
    }
    if(count == 0) return make_pair(0, count);
    return make_pair(100, count);
}

int factorial(int n) {
    if (n == 0 || n == 1) return 1;
    return n * factorial(n - 1);
}

int calculate_points(vector<vector<string>> board, string who){
    pair<int, int> result = determine_gameset(board);
    // fist * factorial(second)

    if(result.first != 100) return result.first * factorial(result.second);

    int i, j;
    int count = 0;
    for(i = 0; i < 3; i++){
        for(j = 0; j < 3; j++){
            vector<vector<string>> new_board = board;
            if(new_board[i][j] == "none"){
                new_board[i][j] = who;
                if(who == USER_CHAR) count += calculate_points(new_board, COMPUTER_CHAR);
                else count += calculate_points(new_board, USER_CHAR);
            }
        }
    }
    return count;
}

pair<int, int> best_pick(){
    int i, j;
    int row = 0, col = 0;
    int score = -9999999;
    for(i = 0; i < 3; i++){
        for(j = 0; j < 3; j++){
            vector<vector<string>> new_board = board;
            if(new_board[i][j] == "none"){
                new_board[i][j] = COMPUTER_CHAR;
                int new_score = calculate_points(new_board, USER_CHAR);
                if(new_score > score){
                    score = new_score;
                    row = i;
                    col = j;
                }
            }
        }
    }
    return make_pair(row, col);
}

int main() {
    // Python 스크립트를 호출하고 결과를 읽어옴
    FILE* pipe = popen("python3 yolo2py.py", "r");
    if (!pipe) {
        cerr << "Failed to run Python script" << endl;
        return 1;
    }

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
    cout << best_pick().first << " " << best_pick().second << endl;

    return 0;
}
