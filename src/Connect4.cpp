/*
 * Connect4.cpp
 *
 *  Created on: Aug 30, 2016
 *      Author: hansondg
 */

#include <limits>
#include <algorithm>
#include <array>
#include <sstream>
#include <iterator>
#include <bitset>
#include "Connect4.h"
#include "Minimax.hpp"

using namespace std;
using namespace dhlib::minimax;
using namespace std::chrono;

constexpr int infinity = 20000;
constexpr int threshhold = infinity / 2;

#ifdef __GNUC__
unsigned int bit_count(uint64_t x){
	return __builtin_popcountll(x);
}
#else
unsigned int bit_count(uint64_t i){ // magic code copied from stack overflow
    i = i - ((i >> 1) & 0x55555555);
    i = (i & 0x33333333) + ((i >> 2) & 0x33333333);
    return (((i + (i >> 4)) & 0x0F0F0F0F) * 0x01010101) >> 24;
}
#endif

size_t std::hash<const state>::operator()(const state& s) const noexcept {
		return hash<uint64_t>()(s.players[0]) ^ hash<uint64_t>()(s.players[1]+0xd586d8560da3d3b7);
}

bool operator==(const state& s1, const state& s2){
	return s1.turn == s2.turn && std::equal(s1.players.begin(), s1.players.end(), s2.players.begin());
}

ostream& operator<<(std::ostream& os, const state& s){
	os << "0123456" << endl;
	for(int row = 5; row >= 0; --row){
		for(int col = 0; col < 7; ++col){ // we reflect the columns so the indexes go left to right
			if(s.players[0] >> (row*7 + col) & 1){
				os << 'o';
			} else if(s.players[1] >> (row*7 + col) & 1){
				os << 'x';
			} else {
				os << '.';
			}
		}
		os << endl;
	}
	os << "0123456" << endl;
	return os;
}

const uint64_t makeBoard(const initializer_list<int> mask){
	uint64_t uint = 0;
	for(auto bit = mask.begin(); bit != mask.end(); ++bit){
		uint = (uint << 1) | *bit;
	}
	return uint;
}

const uint64_t ROW_MASK = makeBoard({
	1,1,1,1
});

const uint64_t COLUMN_MASK = makeBoard({
	0,0,0,0,0,0,1,
	0,0,0,0,0,0,1,
	0,0,0,0,0,0,1,
	0,0,0,0,0,0,1
});

const uint64_t BACKWARD_DIAG_MASK = makeBoard({
	0,0,0,0,0,0,1,
	0,0,0,0,0,1,0,
	0,0,0,0,1,0,0,
	0,0,0,1,0,0,0
});

const uint64_t FORWARD_DIAG_MASK = makeBoard({
	0,0,0,1,0,0,0,
	0,0,0,0,1,0,0,
	0,0,0,0,0,1,0,
	0,0,0,0,0,0,1
});

static int square(int x){
	return x*x;
}

int score_board(const state& board){
	int score = 0;
	const uint64_t mine = board.players[0];
	const uint64_t theirs = board.players[1];
	bool won = false;
	bool lost = false;
	for(uint64_t mRow = mine, tRow = theirs; mRow; mRow >>= 7, tRow >>= 7){ // ---'s
		for(int i = 0; i < 4; i++){
			unsigned mwindow = (mRow >> i) & ROW_MASK;
			unsigned twindow = (tRow >> i) & ROW_MASK;
			if(mwindow == ROW_MASK){
				won = true;
			} else if(twindow == ROW_MASK){
				lost = true;
			} else {
				if(mwindow == 0){
					score -= square(bit_count(twindow));
				}
				if(twindow == 0){
					score += square(bit_count(mwindow));
				}
			}
		}
	}
	for(int i = 0; i < 21; i++){ // |'s
		uint64_t mShifted= (mine >> i) & COLUMN_MASK;
		uint64_t tShifted = (theirs >> i) & COLUMN_MASK;
		if(mShifted == COLUMN_MASK){
			won = true;
		} else if(tShifted == COLUMN_MASK){
			lost = true;
		} else {
			if(mShifted == 0){
				score -= square(bit_count(tShifted));
			}
			if(tShifted == 0){
				score += square(bit_count(mShifted));
			}
		}
	}
	for(int i = 0; i < 4; i++){ // \'s
		uint64_t mShifted = mine >> i;
		uint64_t tShifted = theirs >> i;
		for(int j = 0; j < 3; j++){
			uint64_t mWindow = (mShifted >> 7*j) & BACKWARD_DIAG_MASK;
			uint64_t tWindow = (tShifted >> 7*j) & BACKWARD_DIAG_MASK;
			if(mWindow == BACKWARD_DIAG_MASK){
				won = true;
			} else if(tWindow == BACKWARD_DIAG_MASK){
				lost = true;
			} else {
				if(mWindow == 0){
					score -= square(bit_count(tWindow));
				}
				if(tWindow == 0){
					score += square(bit_count(mWindow));
				}
			}
		}
	}
	for(int i = 0; i < 4; i++){ // /'s
		uint64_t mShifted = mine >> i;
		uint64_t tShifted = theirs >> i;
		for(int j = 0; j < 3; j++){
			uint64_t mWindow = (mShifted >> 7*j) & FORWARD_DIAG_MASK;
			uint64_t tWindow = (tShifted >> 7*j) & FORWARD_DIAG_MASK;
			if(mWindow == FORWARD_DIAG_MASK){
				won = true;
			} else if(tWindow == FORWARD_DIAG_MASK){
				lost = true;
			} else {
				if(mWindow == 0){
					score -= square(bit_count(tWindow));
				}
				if(tWindow == 0){
					score += square(bit_count(mWindow));
				}
			}
		}
	}
	if(won && lost){
		throw invalid_argument("invalid game state both players with winning arrangement");
	}
	if(won){ // we have a dominating strategy! Finish them!
		return infinity - bit_count(theirs);
	}
	if(lost){ // opponent has a dominating strategy, try to maximize turns left, use score to break ties
		return 100*bit_count(mine) - infinity + score;
	}
	return score;
}

int heuristic::operator()(const state& state) const noexcept {
	return score_board(state);
}

bool check_winner(uint64_t board, int row, int col){
	uint64_t shifted = board >> 7*row;
	for(int i = 0; i < 4; i++){
		unsigned window = (shifted >> i) & ROW_MASK;
		if(window == ROW_MASK){
			return true;
		}
	}
	shifted = board >> col;
	for(int j = 0; j < 3; j++){
		uint64_t window = shifted & COLUMN_MASK;
		if(window == COLUMN_MASK){
			return true;
		}
		shifted >>= 7;
	}
	shifted = board >> col;
	for(int j = 0; j < 3; j++){
		uint64_t window = shifted & BACKWARD_DIAG_MASK;
		if(window == BACKWARD_DIAG_MASK){
			return true;
		}
		shifted >>= 7;
	}
	for(int i = 0; i <= min(3, col); i++){ // /'s
		shifted = board >> i;
		for(int j = 0; j <= min(2, row); j++){
			uint64_t window = (shifted >> 7*j) & FORWARD_DIAG_MASK;
			if(window == FORWARD_DIAG_MASK){
				return true;
			}
		}
	}
	for(int i = 0; i <= min(3, col); i++){ // \'s
		shifted = board >> i;
		for(int j = 0; j <= min(2, row); j++){
			uint64_t window = (shifted >> 7*j) & BACKWARD_DIAG_MASK;
			if(window == BACKWARD_DIAG_MASK){
				return true;
			}
		}
	}
	return false;
}

const size_t get_choices::SEED(std::chrono::system_clock::now().time_since_epoch().count());
std::default_random_engine get_choices::random(get_choices::SEED);

//const size_t get_choices::SEED(1473376696515541738L);
//std::default_random_engine get_choices::random(get_choices::SEED);

const vector<pair<choice,state>> get_choices::operator()(const state& s) noexcept {
	if(s.end){
		return vector<pair<choice,state>>();
	}
	vector<pair<choice,state>> children;
	children.reserve(7);
	state next(!s.turn, s.end, s.players[0], s.players[1]);
	const uint64_t oldPieces = s.players[s.turn];
	uint64_t& newPieces = next.players[s.turn];
	const uint64_t board = s.players[0] | s.players[1];
	for(int nextChoice = 0; nextChoice < 7; nextChoice++){
		uint64_t shift = board >> nextChoice; // select the column choice corresponds to
		int row = 0;
		while(shift & 1){ // look for first open space in column
			shift >>= 7;
			row++;
		}
		if(row > 5){
			continue;
		}
		newPieces = oldPieces | (uint64_t(1) << (nextChoice + row * 7)); // add new piece
		if(check_winner(newPieces, row, nextChoice)){
			next.end = true; // this is a winning child, ignore the other children
			vector<pair<choice,state>> ret = {make_pair(nextChoice, next)};
			return ret;
		}
		children.emplace_back(nextChoice, next);
		next.end = false;
	}
	shuffle(children.begin(), children.end(), random); // add some randomness
	return children;
}

void ais(size_t level){
	minimax<score,state,choice,heuristic,get_choices> mm (state(0,false,0,0),MAX);
	bool turn = 0;
	while(true){
		state state;
		string x;
		mm.compute(level);
		int choice = mm.choose(0);
		cout << (turn ? "o: " : "x: ") << choice << endl;
		bool type = mm.type();
		state = mm.progress(choice);
		cout << mm.score() << ' ' << (type ? "MAX" : "MIN") << endl;
		cout << state << endl;
		if(abs(score_board(state)) > threshhold){
			cout << "game over" << endl;
			return;
		}
		turn = !turn;
	}
}

void hva(size_t level, bool humanFirst){
	state s = state(0,false,0,0);
	minimax<score,state,int,heuristic,get_choices> mm (s, true);
	cout << s << endl;
	string x;
	int choice;
	bool turn = true;
	while(true){
		if(!humanFirst){
			mm.compute(level);
			choice = mm.choose(0);
			cout << (turn ? "o: " : "x: ") << choice << " score: " << mm.score() << endl;
		//	minimax<score,state,int,heuristic,get_choices> mm2 (s, turn);
		//	mm2.compute(level);
		//	mm.root->print(cout);
		//	cout << endl;
		//	mm2.root->print(cout);
		//	cout << endl;
			//mm.verify();
			//mm2.verify();
			s = mm.progress(choice);
			cout << s << endl;
			if(abs(score_board(s)) > threshhold){
				cout << "game over" << endl;
				return;
			}
			turn = !turn;
		}
		humanFirst = false;
		do {
			cout << (turn ? "o: " : "x: ");
			cin >> choice;
		} while(choice > 6 || choice < 0);
		s = mm.progress(choice);
		cout << s << endl;
		if(abs(score_board(s)) > threshhold){
			cout << "game over" << endl;
			return;
		}
		turn = !turn;
	}
}

int main(int argc, char* argv[]){
	char x;
	size_t level;
	cerr << "using seed: " << get_choices::SEED;
	while(true) {
		cout << "0 to spectate, 1 to go first, 2 to go second, q to quit: ";
		cin >> x;
		do {
			cout << "difficulty (0-9): ";
			cin >> level;
		} while('0' <= level && level <= '9');
		switch(x){
		case '0': ais(level); break;
		case '1': hva(level,true); break;
		case '2': hva(level,false); break;
		}
	}
}

/*
int main(int argc, char* argv[]){
	ios_base::sync_with_stdio(false);
	game g;
	char buf[128];
	bool turn = false;
	buf[127] = '\0';
	char *inserter = buf;
	cerr << "USING SEED: " << get_choices::SEED << endl;
	while(true){
		g.minimax.compute(9);
		if(g.action){
			g.action = false;
			choice c = g.minimax.choose(-1);
			cout << "place_disc " << 6-c << endl << flush;
			try {
				g.minimax.progress(c);
			} catch(const invalid_argument& e){
				cerr << e.what() << endl;
			}
			cerr << "MY MOVE: " << c << ':' << g.minimax.score() << endl;
			cerr << g.minimax.state() << endl << endl;
		}
		while(cin.readsome(inserter, 1) > 0){
			if(*inserter != '\n'){
				++inserter;
				continue;
			}
			inserter = buf;
			stringstream in;
			string word;
			in << buf;
			in >> word;
			if(word == "settings"){
				in >> word;
				if(word == "timebank") {
					in >> g.settings.timebank;
				} else if(word == "time_per_move") {
					in >> g.settings.time_per_move;
				} else if(word == "player_names") {

				} else if(word == "your_bot") {
					in >> g.settings.your_bot;
				} else if(word == "your_botid") {
					in >> g.settings.your_botid;
				} else if("field_columns") {
					in >> g.settings.field_columns;
				} else if("field_rows") {
					in >> g.settings.field_rows;
				} else {
					cerr << "bad setting: " << word << endl;
				}
			} else if(word == "update") {
				in >> word;
				if(word == "game") {
					in >> word;
					if(word == "round") {
						in >> g.round;
					} else if(word == "field"){
						if(!turn){
							uint64_t omove = 0;
							string board;
							in >> board;
							for(int i = 0; i < 42; i++) {
								int disc = board[2*i] - '0';
								omove = (omove << 1) | (disc != 0 && disc != g.settings.your_botid);
							}
							uint64_t disc = omove ^ g.minimax.state().players[!(g.settings.your_botid-1)];
							cerr << "DISC: " << bitset<42>(disc) << endl;
							if(disc){ // disc is 0 for the start of the first round
								uint64_t row = disc;
								while(row >= (1 << 7)){
									row >>= 7;
								}
								int choice = -1;
								while(row){
									row >>= 1;
									choice++;
								}
								try {
									g.minimax.progress(choice);
								} catch(const invalid_argument& e){
									cerr << e.what() << endl;
								}
								cerr << "THEIR MOVE: " << choice << ':' << g.minimax.score() << endl;
								cerr << g.minimax.state() << endl << endl;
							}
						}
						turn = !turn;
					} else {
						cerr << "bad update game: " << word << endl;
					}
				} else {
					cerr << "bad update: " << word << endl;
				}
			} else if(word == "action"){
				in >> word;
				if(word == "move") {
					unsigned long addedTime;
					g.action = true;
					in >> addedTime;
					g.timebank = addedTime;
				} else {
					cerr << "bad action: " << word << endl;
				}
			} else {
				cerr << "bad command: " << word << endl;
			}
		}
	}
}
*/
