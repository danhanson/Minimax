/*
 * Connect4.h
 *
 *  Created on: Aug 30, 2016
 *      Author: hansondg
 */

#ifndef CONNECT4_H_
#define CONNECT4_H_

#include <vector>
#include <cstdint>
#include <chrono>
#include <iostream>
#include "Minimax.hpp"

class Board;
struct Game;
struct Settings;

using choice = int;
using score = int;

struct state {
	bool turn;
	bool end;
	std::array<uint64_t,2> players;
	state() { }
	state(bool turnA, bool endA, uint64_t player1, uint64_t player2) : turn(turnA), end(endA) {
		players[0] = player1;
		players[1] = player2;
	}
	state(const state& board) : turn(board.turn), end(board.end), players(board.players) { }
};

namespace std {

	template<>
	struct hash<const state> {
		size_t operator()(const state& state) const noexcept;
	};
}

bool operator==(const state&, const state&);


std::ostream& operator<<(std::ostream& os, const state& state);

struct heuristic {
	score operator()(const state&) const noexcept;
};

struct get_choices {
	const std::vector<std::pair<choice,state>> operator()(const state& state) noexcept;
	static std::default_random_engine random;
	static const size_t SEED;
};

struct Settings {
	unsigned long timebank;
	unsigned long time_per_move;
	std::vector<std::string> player_names;
	std::string your_bot;
	short your_botid;
	short field_columns;
	short field_rows;
};

template class dhlib::minimax::minimax<score,state,choice,heuristic,get_choices>;

struct game {
	short round;
	unsigned long timebank;
	bool action;
	Settings settings;
	dhlib::minimax::minimax<score,state,choice,heuristic,get_choices> minimax;
	game() : round(0), timebank(0), action(false), minimax(state(0,false,0,0),dhlib::minimax::MAX) {}
};


#endif /* CONNECT4_H_ */
