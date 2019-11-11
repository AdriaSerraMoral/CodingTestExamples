/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Problem Statement:
//
//  If you were only permitted to complete at most one transaction
//  (i.e., buy one and sell one share of the stock), design an algorithm to find the
//  maximum profit.
//
//  Note that you cannot sell a stock before you buy one.
//
/////////////////////////////////////////////////////////////////////////////////////////////////////


#include <stdlib.h>
#include <iostream>
#include <vector>
#include <climits>
#include <algorithm>

/**
 * @brief The BestTimeBuyStock class
 *
 *      I see two different ways to solve problem.
 *      First, we could use a brute force, O(n^2)-ish, by running 2 loops,
 *      i from 0 to N, and j from i to N, and check all possible profits
 *
 *      Also, since we know all possible max profits happen after a minimum price,
 *      we can do in 1 loop, keeping track of both minimum price, and maximum profit.
 *      as we advance, we check and update both variables when necessary.
 */
class BestTimeBuyStock {
public:

    int maxProfitBF( const std::vector<int>& prices ) {

        int max_profit {0};

        for( size_t idx = 0, idx_end = prices.size(); idx < idx_end; idx++) {
            for( size_t jdx = idx + 1, jdx_end = prices.size(); jdx < jdx_end; jdx++) {

                if( prices.at(jdx) - prices.at(idx) > max_profit ){
                    max_profit = prices.at(jdx) - prices.at(idx);
                }

            }
        }

        return max_profit;

    }

    int maxProfit( const std::vector<int>& prices ) {
        int min_price {INT_MAX};
        int max_profit {0};

        for ( size_t idx = 0, idx_end = prices.size(); idx < idx_end; idx++ ){

            if( prices.at(idx) < min_price ) {
                min_price = prices.at(idx);
            }

            if( prices.at(idx) - min_price > max_profit ) {
                max_profit = prices.at(idx) - min_price;
            }

        }

        return max_profit;
    }


};


int main(int argc, char *argv[])
{

//    std::vector<int> prices {7, 1, 5, 3, 6, 4};
//    std::vector<int> prices {7, 6, 4, 3, 1};
    std::vector<int> prices {7, 7, 15, 2, 11, 4};

    BestTimeBuyStock bts;

    int max_profit_BF = bts.maxProfitBF( prices );
    int max_profit = bts.maxProfit( prices );

    std::cout << "Max. Profit BF is:  " << max_profit_BF << std::endl;
    std::cout << "Max. Profit is:     " << max_profit << std::endl;

    return 0;
}

