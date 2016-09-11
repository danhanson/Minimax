/*
 * Minimax.h
 *
 *  Created on: Aug 28, 2016
 *      Author: hansondg
 */

#ifndef MINIMAX_H_
#define MINIMAX_H_

#include <memory>
#include <algorithm>
#include <unordered_map>
#include <iostream>
#include <set>
#include <vector>
#include <deque>
#include <limits>
#include <cassert>

namespace dhlib { namespace minimax {

	constexpr bool MAX = true;
	constexpr bool MIN = false;

	namespace details {
		/**
		 * indicates a position in the tree
		 */
		template<typename Minimax>
		class Marker {
		public:
			using Node = typename Minimax::node;
			Marker(Minimax& mm) noexcept {
				path_.emplace_back(mm.root);
			}

			void progress(const Node& newRoot) noexcept {
				while(path_.size()){
					std::weak_ptr<Node> &front = path_.front();
					if(front.expired()){
						path_.pop_front();
						continue;
					}
					std::shared_ptr<Node> next = path_.front().lock();
					if(&newRoot == next.get()){
						return;
					}
					path_.pop_front();
				}
			}
			bool expired() const noexcept {
				while(path_.size()){
					if(path_.front().expired()){
						path_.pop_front();
					} else {
						return false;
					}
				}
				return true;
			}
			std::weak_ptr<Node> node() const noexcept {
				if(path_.size()){
					return path_.back();
				}
				return std::weak_ptr<Node>();
			}
			std::deque<std::weak_ptr<Node>>& path() const noexcept {
				return path_;
			}
		private:
			mutable std::deque<std::weak_ptr<Node>> path_;
		};
	}

	/**
	 * A minimax graph for computing decisions.
	 * 	Score: A type used to rank different states, it is usually a numeric type.
	 *	State: A representation of the game state that is stored on each minimax node, requires
	 *		std::hash<State>
	 *		operator==(State,State).
	 */
	template<typename Score, typename State, typename Choice, typename Heuristic, typename GetChoices>
	class minimax {
	public:
		class node {
		public:
			using NodeScore = Score;
			bool mark;
			State state;
			Score score; // this node's score calculated from its children or from the heuristic
			size_t height; // distance to closest child leaf
			std::vector<std::shared_ptr<node>> children;
			std::vector<Choice> choices;
			node();
			node(const minimax& minimaxA, State stateA, bool type) :
				mark(false), state(stateA),
				score(type ? std::numeric_limits<Score>::min() : std::numeric_limits<Score>::max()),
				height(0) { }

			std::ostream& print(std::ostream& out) const {
				out << "(" << score << ':' << height << ' ';
				for(auto iter = children.begin(); iter != children.end(); ++iter){
					out << ' ' << choices[iter-children.begin()] << ':';
					(*iter)->print(out);
				}
				return out << ")";
			}
			void printChildren(size_t depth){
				std::cout << "score:" << score << ":" << height;
				if(depth == 0){
					return;
				}
				for(auto &child : children) {
					child->printChildren(depth-1);
				}
				std::cout << std::endl;
			}
			void verifyNode(bool type) {
				Score best = type ? std::numeric_limits<Score>::min() : std::numeric_limits<Score>::max();
				for(std::shared_ptr<node> &child : children){
					if(type){
						best = std::max(best, child->score);
						assert(child->score <= score);
					} else {
						best = std::min(best, child->score);
						assert(child->score >= score);
					}
				}
				if(children.size()){
					assert(score == best);
				} else {
					Heuristic h;
					assert(score == h(state));
				}
				size_t infinity = std::numeric_limits<size_t>::max();
				bool allInf = true;
				for(std::shared_ptr<node> &child : children){
					if(child->score == best){
						assert(child->height == infinity || child->height+1 == height);
						if(child->height != infinity) {
							allInf = false;
						}
					}
				}
				assert(height != infinity || allInf);
				for(std::shared_ptr<node> &child : children){
					if(child->score == best){ // pruned nodes won't have their values calculated
						child->verifyNode(!type);
					}
				}
			}
		};

		using node_ptr = std::shared_ptr<node>;
		using const_ptr = std::shared_ptr<const node>;
		using weak_ptr = std::weak_ptr<node>;
		using const_weak_ptr = std::weak_ptr<const node>;
		using child_iter = typename std::vector<node_ptr>::iterator;
		using const_child_iter = typename std::vector<node_ptr>::const_iterator;
		using marker = details::Marker<minimax<Score,State,Choice,Heuristic, GetChoices>>;
		using const_marker = details::Marker<const minimax<Score,State,Choice,Heuristic, GetChoices>>;

		std::unordered_map<const State, std::weak_ptr<node>> nodes;
		node_ptr root;

		minimax(const State& start, bool isMax) noexcept : root(new node(*this, start, isMax)), type_(isMax) {}

		/**
		 * Sets the root's child with the specified choice as the root, it also negates the tree type,
		 * throws invalid argument if choice is not allowed for the root's state
		 */
		const State& progress(const Choice& choice);

		/**
		 * Provides the best calculated move
		 */
		const Choice& choose(const Choice& def) const noexcept;

		/**
		 * Traverses nodes below the specified marker, using the heuristic to calculate leaf values,
		 * uses a/b pruning, continues until node at marker has specified height, if no node is specified,
		 * the root is used
		 */
		void compute(std::size_t height) noexcept;
		void compute(size_t height, const marker& start);

		/**
		 * Uses mark and sweep to remove separated node cycles, doesn't need to be used if cycles can't occur.
		 */
		void collect_garbage() noexcept;

		std::ostream& print(std::ostream& os) const;

		const Score& score() const noexcept {
			return root->score;
		}
		const State& state() const noexcept;

		bool type() const noexcept {
			return type_;
		}

		void verify();
	private:
		bool type_;
	};

	template<typename Node>
	static bool is_good(const std::weak_ptr<Node>& ptr){
		return !ptr.expired();
	}

	template<typename Node>
	static bool is_bad(const std::weak_ptr<Node>& ptr){
		return ptr.expired();
	}

	template<typename Score, typename State, typename Choice, typename Heuristic, typename GetChoices>
	void minimax<Score,State,Choice,Heuristic,GetChoices>::collect_garbage() noexcept {
		std::vector<std::pair<child_iter, child_iter>> path;
		root->mark = true;
		path.emplace_back(root->children.begin(), root->children.end());
		while(path.size() > 0){
			std::pair<child_iter,child_iter>& range = path.back();
			child_iter &begin = range.first, &end = range.second;
			if(begin == end){
				path.pop_back();
			} else {
				node_ptr &child = *begin;
				if(!child->mark) {
					child->mark = true;
					path.emplace_back(child->children.begin(), child->children.end());
				}
				++begin;
			}
		}
		for(auto entry = nodes.begin(); entry != nodes.end(); ){
			if(entry->second.expired()){
				entry = nodes.erase(entry);
			} else {
				node_ptr node = entry->second.lock();
				if(!node->mark) {
					node->children.clear(); // delete the children pointers to remove the garbage
					entry = nodes.erase(entry);
				} else {
					node->mark = false; // reset mark
					++entry;
				}
			}
		}
	}

	template<typename Score, typename State, typename Choice, typename Heuristic, typename GetChoices>
	const State& minimax<Score,State,Choice,Heuristic,GetChoices>::progress(const Choice& choice) {
		type_ = !type_;
		if(root->children.empty()){
			compute(1);
			if(root->children.empty()){
				throw std::logic_error("choice inconsistent with game state");
			}
		}
		node_ptr newRoot;
		for(
				auto childChoice = root->choices.begin(), child = root->children.begin();
				childChoice != root->choices.end();
				++childChoice, ++child
		){
			if(*childChoice == choice && !newRoot){
				newRoot = *child;
			}
		}
		if(!newRoot){
			throw std::invalid_argument("invalid choice");
		}
		root = newRoot; // this may trigger deletions of nodes referenced by mightDelete
		for(auto entry = nodes.begin(); entry != nodes.end(); ){
			if(entry->second.expired()){
				entry = nodes.erase(entry);
			} else {
				++entry;
			}
		}
		return newRoot->state;
	}

	template<typename Score, typename State, typename Choice, typename Heuristic, typename GetChoices>
	const State& minimax<Score,State,Choice,Heuristic,GetChoices>::state() const noexcept {
		return root->state;
	}

	template<typename Score, typename State, typename Choice, typename Heuristic, typename GetChoices>
	void minimax<Score,State,Choice,Heuristic,GetChoices>::compute(size_t depth) noexcept {
		compute(depth, marker(*this));
	}

	/* updates parent's score from child score based on parentType */
	template<typename Node>
	void update_score(Node &child, Node &parent, bool parentType){
		if(parentType == MAX){
			parent.score = std::max(parent.score, child.score);
		} else {
			parent.score = std::min(parent.score, child.score);
		}
	}

	/* same as above, but returns true if we can apply pruning */
	template<typename Node>
	bool update_score(Node &child, Node &parent, Node &gParent, bool parentType){
		if(parentType == MAX){
			parent.score = std::max(parent.score, child.score);
			if(parent.score > gParent.score){
				if(&child != parent.children.back().get()) { // a parent that is pruned based on its last child isn't really pruned
					return true;
				}
			}
		} else {
			parent.score = std::min(parent.score, child.score);
			if(parent.score < gParent.score){
				if(&child != parent.children.back().get()) {
					return true;
				}
			}
		}
		return false;
	}

	/* returns the next explorable node, this function skips explored nodes and applies pruning while upading parent score and height */
	template<typename Node, typename Path>
	Node& next_node(const size_t depth, Node &startNode, Path &path, bool &nodeType) noexcept {
		constexpr size_t infinity = std::numeric_limits<size_t>::max();

		// next child
		while(path.size()) {
			Node *parent = (path.size() >= 2) ? path[path.size() - 2]->get() : &startNode;
			bool parentType = !nodeType;

			// iter is a pointer to an iterator over pointers in a nodes children vector, we use a pointer because we want
			// increments to iter to mutate path.back(), we do not use a reference because we may change iter if pruning occurs
			auto *iter = &path.back();
			while(*iter != parent->children.end()) { // check for next sibling
				size_t childHeight = (**iter)->height;
				if(path.size() >= 2){
					Node &gParent = path.size() >= 3 ? **path[path.size()-3] : startNode;
					if(update_score(***iter, *parent, gParent, parentType)){
						while(++*iter != parent->children.end()){
							if((**iter)->score == parentType ? std::numeric_limits<typename Node::NodeScore>::max() : std::numeric_limits<typename Node::NodeScore>::min()){
								parent->height = std::min(parent->height, (**iter)->height);
								if(parent->height == 0){
									break;
								}
							} else if((**iter)->height != infinity){
								parent->height = std::min(parent->height, (**iter)->height+1);
							}
						}
						if(childHeight != infinity){ // update the grandparent's height
							gParent.height = std::min(gParent.height, childHeight + 2);
						}
						nodeType = parentType;
						parentType = !nodeType;
						path.pop_back(); // we can skip further exploration of parent, pruned nodes do not count for their parents' heights or score
						iter = &path.back();
						parent = &gParent;
					} else if(childHeight != infinity) {
						parent->height = std::min(childHeight + 1, parent->height);
					}
				} else {
					update_score(***iter, *parent, parentType);
					if(childHeight != infinity) {
						parent->height = std::min(childHeight + 1, parent->height);
					}
				}
				++*iter;
				if(*iter != parent->children.end() && (**iter)->height <= depth - path.size()){ // finished this node
					return ***iter; // return next child
				}
			}
			path.pop_back(); // backtrack to parent and search parent's siblings
			nodeType = parentType;
		}
		return startNode;
	}

	template<typename Score, typename State, typename Choice, typename Heuristic, typename GetChoices>
	void minimax<Score,State,Choice,Heuristic,GetChoices>::compute(size_t depth, const marker& start) {
		Heuristic heuristic;
		GetChoices getChoices;
		constexpr size_t infinity = std::numeric_limits<size_t>::max();
		if(start.expired()){
			throw std::invalid_argument("compute received expired marker");
		}
		node_ptr startNode = start.node().lock();
		if(startNode->height >= depth || startNode->height == infinity){
			return;
		}
		std::vector<child_iter> path;
		path.reserve(depth);
		bool nodeType = (start.path().size() & 1) == type_;
		node* at = startNode.get();
		do { // an iteration of this loop calculates the value for at, this loop ends when backtracking to the marker

			// to the leaves
			while(at->height < depth - path.size()){
				at->height = infinity; // height is set to work with min function and indicate that node is visited
				at->score = nodeType ? std::numeric_limits<Score>::min() : std::numeric_limits<Score>::max();
				if(at->children.size()){
					path.emplace_back(at->children.begin());
					at = at->children.front().get();
					nodeType = !nodeType;
				} else { //must construct remaining nodes
					while(depth > path.size()){
						bool childType = !nodeType;
						for(auto &child : getChoices(at->state)){
							auto maybeNode = nodes.find(child.second);
							if(maybeNode != nodes.end()){ // node for child state already exists, use it
								at->children.emplace_back(maybeNode->second);
								at->choices.emplace_back(child.first);
							} else {
								at->children.emplace_back(std::make_shared<node>(*this, child.second, childType));
								at->choices.emplace_back(child.first);
								nodes.emplace(child.second, weak_ptr(at->children.back()));
							}
						}
						at->height = infinity; // set height to infinity for min and for if this node is a dead end
						if(at->children.empty()){
							break; // dead end
						}
						at->children.shrink_to_fit();
						path.emplace_back(at->children.begin());
						at = at->children.front().get();
						nodeType = !nodeType;
					}
				}
			}

			// if at is a leaf, calculate its value
			if(at->children.empty()) {
				at->score = heuristic(at->state);
			}

			// backtrack
			at = &next_node(depth, *startNode, path, nodeType);
		} while(startNode.get() != at);
/*
		// update the parents
		std::deque<weak_ptr> &parents = start.path();
		for(auto parentPtr = parents.rbegin(); parentPtr < parents.rend()-1; ++parentPtr){
			node& parent = *parentPtr->lock();
			bool parentType = !nodeType;

			// update score
			if(parentType == MAX){
				parent.score = std::max(at->score, parent.score);
			} else {
				parent.score = std::min(at->score, parent.score);
			}

			// update height
			if(at->height >= parent.height) {
				parent.height = infinity;
				for(node_ptr &child : parent.children){
					if(child->height == infinity){
						parent.height = infinity;
					} else {
						parent.height = std::min(parent.height, child->height+1);
					}
				}
			}
			at = &parent;
			nodeType = parentType;
		}*/
	};

	template<typename Score, typename State, typename Choice, typename Heuristic, typename GetChoices>
	const Choice& minimax<Score,State,Choice,Heuristic,GetChoices>::choose(const Choice& def) const noexcept {
		const Choice* bestChoice = &def;
		Score best = type_ ? std::numeric_limits<Score>::min() : std::numeric_limits<Score>::max();
		for(
			auto child = root->children.cbegin(), choice = root->choices.cbegin();
			child != root->children.end();
			++child, ++choice
		){
			if(type_ ? ((*child)->score > best) : ((*child)->score < best)){
				best = (*child)->score;
				bestChoice = &(*choice);
			}
		}
		return *bestChoice;
	}

	template<typename Score, typename State, typename Choice, typename Heuristic, typename GetChoices>
	std::ostream& minimax<Score,State,Choice,Heuristic,GetChoices>::print(std::ostream& os) const {
		for(auto iter = nodes.cbegin(); iter != nodes.cend(); ++iter){
			node& node = *iter->second.lock();
			std::cout << &node << ": " << " score: " << node.score << " children: ";
			for(auto child = node.children.begin(); child != node.children.end(); ++child){
				std::cout << &(*child) << ' ';
			}
			std::cout << std::endl;
		}
		return std::cout;
	}

	template<typename Score, typename State, typename Choice, typename Heuristic, typename GetChoices>
	void minimax<Score,State,Choice,Heuristic,GetChoices>::verify(){
		root->verifyNode(type_);
	}
} }

#endif /* MINIMAX_H_ */
