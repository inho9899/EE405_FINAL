#include <iostream>
#include <vector>
#include <string>
#include <stdlib.h>
#include <cmath>
#include <unistd.h>

using namespace std;

string USER_CHAR = "USER_CHAR";
string COMPUTER_CHAR = "COMPUTER_CHAR";

vector<vector<string>> board = {
    {"-", "-", "-"},
    {"-", "-", "-"},
    {"-", "-", "-"}
};  // '-' : empty, USER_CHAR : user, COMPUTER_CHAR : computer

pair<int, int> determine_gameset(vector<vector<string>> board){
    int i, j;
    int count = 0;
    for(i = 0; i < 3; i++){
        for(j = 0; j < 3; j++){
            if(board[i][j] == "-") count++;
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
            if(new_board[i][j] == "-"){
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
            if(new_board[i][j] == "-"){
                new_board[i][j] = COMPUTER_CHAR;
                int new_score = calculate_points(new_board, USER_CHAR);
                cout << i << " " << j << " " << new_score << "\n";
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

int set_who_is_first(){
    srand(time(NULL));
    switch(rand() % 2){
        case 0:
            USER_CHAR = "red";
            COMPUTER_CHAR = "blue";
            return 0;
        case 1:
            USER_CHAR = "blue";
            COMPUTER_CHAR = "red";
            return 1;
    }
}

int main(){
    int turn = set_who_is_first(); // first : red, second : blue
    
    while(determine_gameset(board).first == 100){
        system("clear");
        cout << "Welcome to Tic-Tac-Toe game!\n";

        cout << "You are " << USER_CHAR << " and the computer is " << COMPUTER_CHAR << ".\n";

        for(int i = 0; i < 3; i++){
            for(int j = 0; j < 3; j++){
                cout << board[i][j] << " ";
            }
            cout << "\n";
        }

        if(turn == 0){
            cout << "Your turn!\n";
            int row, col;
            cout << "Enter the row and column of your pick : ";
            cin >> row >> col;
            if(board[row][col] != "-" || row < 0 || row >= 3 || col < 0 || col >= 3){
                cout << "Invalid pick. Try again.\n";
                continue;
            }
            board[row][col] = USER_CHAR;

        } 
        else {
            cout << "Computer's turn! Computer is Thinking...\n";
            sleep(5);
            pair<int, int> computer_pick = best_pick();
            board[computer_pick.first][computer_pick.second] = COMPUTER_CHAR;
        }        
        turn = 1 - turn;
    }
    int result = determine_gameset(board).first;

    system("clear");
    cout << "Welcome to Tic-Tac-Toe game!\n";

    cout << "You are " << USER_CHAR << " and the computer is " << COMPUTER_CHAR << ".\n";

    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            cout << board[i][j] << " ";
        }
        cout << "\n";
    }

    if(result == 0) cout << "Draw!\n";
    else if(result == -1) cout << "You win!\n";
    else if(result == 1) cout << "Computer wins!\n";
}
