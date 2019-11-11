/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Problem Statement:
//
//  You are given coins of different denominations and a total amount of money amount.
//  Write a function to compute the fewest number of coins that you need to make up that amount.
//  If that amount of money cannot be made up by any combination of the coins, return -1.
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////


#include <stdlib.h>
#include <iostream>
#include <queue>
#include <vector>
#include <climits>
#include <algorithm>

/**
 * @brief The CoinChangeCounter class
 *
 *  Discussion:
 *      I see two different ways of solving the problem. One way would be to put data in a graph/tree
 *      and search the tree using bfs (since we want fewer number of coins, we want the top-most
 *      solution). If all tree is searched and we have no solution, return -1.
 *
 *      Last, I see this similar to a Dynamic Programing, like the popular climb a stair taking
 *      one step, two steps, etc. The solution to a particular level will be the solution to the
 *      value-f(coin) for the n coins we have. (e.g., if we have coins with value 1, 2, 5 and the
 *      value is 10, our solution is min( n_coins(10-1), n_coins(10-2), n_coins(10-5) )+1. See that
 *      we could get 9 as 5+2+2 = 3 coins, 8 as 5 + 3 = 2 coins, 5 as 5 = 1 coin. Hence,
 *      our solution is min( 3, 2, 1 ) + 1 = 2 coins, which are 5 and 5.
 */
struct CoinNode {
    CoinNode( int value, std::vector<int> coins_used ) {
        this->value = value;
        this->n_coins = (int)(coins_used.size());
        coins_change = coins_used;
    }
    CoinNode() = default;

    int value {0};
    int n_coins {0};

    std::vector<int> coins_change;
};

class CoinChangeCounter {
public:
    CoinChangeCounter() {}
    ~CoinChangeCounter() {}

    int coinChange( const std::vector<int>& coins, int amount) {

        if( coins.empty() ) return -1;

        coins_available = coins;
        goal_value = amount;

        // First, we add the coins to the queue since they are roots
        // in our tree
        if( addChildren() ){
            return goal_node.n_coins;
        }

        while( !open_nodes.empty() ) {

            const CoinNode node_idx = open_nodes.front();
            open_nodes.pop_front();

            if( addChildren(node_idx) ){
                return goal_node.n_coins;
            }

        }

        return -1;
    }

    int coinChangeDynamic( const std::vector<int>& coins, int amount ) {
        if( coins.empty() ) return -1;

        // initialize array of our desired value type
        coin_change_vector = std::vector<int>(amount+1, INT_MAX);

        // initialize our coin values
        for( const int& c : coins ) {
            coin_change_vector.at(c) = 1;
        }

        // For all possible solutions
        for( int idx = 0, idx_end = coin_change_vector.size(); idx < idx_end; idx++) {
            if( coin_change_vector.at(idx) != INT_MAX ) continue;

            std::vector<int> vpossible = getPossibleValues(idx);

            int min_coins = *std::min_element( vpossible.begin(), vpossible.end());

            if( min_coins != INT_MAX ){
                coin_change_vector.at(idx) = min_coins + 1;
            }

        }

        if( coin_change_vector.back() == INT_MAX ) return -1;

        return coin_change_vector.back();
    }

    void print_goal() {
        for( const int& e : goal_node.coins_change ){
            std::cout << e << ", ";
        }
    }

private:

    std::deque< CoinNode >      open_nodes;

    std::vector<int>            coins_available;

    int                         goal_value {0};

    CoinNode                    goal_node;

    std::vector<int>            coin_change_vector;

    bool addChildren(const CoinNode& parent) {

        const std::vector<int>& parent_coins = parent.coins_change;

        for( const int& e : coins_available ) {
            std::vector<int> child_coins = parent_coins;
            child_coins.push_back(e);
            int child_val = parent.value + e;

            // If value is greater than our goal, do not add
            if( child_val > goal_value ) continue;

            CoinNode n_idx( child_val, child_coins );

            if( isGoal(n_idx) ){
                return true;
            }

            open_nodes.push_back( n_idx );
        }

        return false;

    }

    bool addChildren() {
        for( const int& e : coins_available ){

            // If value is greater than our goal, do not add
            if( e > goal_value ) continue;

            CoinNode n_idx(e, std::vector<int>(1, e) );

            if( isGoal(n_idx) ){
                return true;
            }

            open_nodes.push_back( n_idx );
        }

        return false;
    }

    bool isGoal( const CoinNode& node ) {
        if ( node.value == goal_value) {
            goal_node = node;
            return true;
        }

        return false;

    }

    int getValue( const int idx ) {
        if( idx < 0 ) {
            return INT_MAX;
        }

        return coin_change_vector.at(idx);
    }

    std::vector<int> getPossibleValues( const int idx ) {
        std::vector<int> vp(coins_available.size(), INT_MAX);
        for( size_t jdx = 0, jdx_end = coins_available.size(); jdx < jdx_end; jdx++ ) {
            vp.at(jdx) = getValue( idx - coins_available.at(jdx) );
        }

        return vp;
    }

};

int main(int argc, char *argv[])
{
    CoinChangeCounter cc;
    std::vector<int> coins{5};
    const int value = 7;

    int n_coins = cc.coinChange( coins, value );
    int n_coins_dyn = cc.coinChangeDynamic(coins, value );

    std::cout << "We need " << n_coins << " coins" << std::endl;
    std::cout << "Which are: ";
    cc.print_goal();
    std::cout << " " << std::endl;

    if( n_coins != n_coins_dyn ) {
        std::cerr << "Note, the dynamic programming version does not work as intended :( \n";
    }

    return 0;
}
