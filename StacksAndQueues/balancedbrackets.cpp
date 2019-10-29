#include <bits/stdc++.h>
#include <deque>

using namespace std;


// Complete the isBalanced function below.
bool isOpening(const string& s) {
    // if it is opening, return true
    if ( s == "{" || s == "(" || s == "[" ) return true;

    // else return false
    return false;
}

bool isClosing( const string& s_left, const string& s_right) {
    // If agreement, return true
    if( s_left == "{" && s_right == "}" ) return true;

    if( s_left == "(" && s_right == ")" ) return true;

    if( s_left == "[" && s_right == "]" ) return true;

    // else return false
    return false;


}

string isBalanced(string s) {

    // Even if we have the same number of open elements than closed ones, they
    // need to be in a specific open-close agreement. Hence, keep a queue of the
    // most recent opening elenent and if the next closing one is not in agreement,
    // we have a "NO". If at the end of the run, we have elements still open,
    // we also have a "NO". If we do not have any open element and still have closing
    // ones, also "NO".

    deque<string> open_elements;

    // iterate throguh string
    for( const auto& e : s ) {

        // get current string element, (cast to string since iter we get char)
        const string s_idx(1, e);

        // First, check if the current character is opening or not
        if( !isOpening( s_idx ) ) {

            // This is a closing character, if our open_elements is empty,
            // this is not a valid situation, return NO
            if( open_elements.empty() ) return "NO";

            // Open_elements is not empty, hence check for agreement
            // get latest open element
            const string s_left = open_elements.back();

            // check agreement, if fails, return "NO"
            if( !isClosing( s_left, s_idx ) ) return "NO";

            // we have agreement, pop latest and continue
            open_elements.pop_back();

            continue;
        }

        // This is not a closing character, add to queue and continue
        open_elements.push_back( s_idx );

    }

    // we checked all the characters, open_elements must be empty
    if( open_elements.empty() ) return "YES";

    // not empty, return NO
    return "NO";

}

int main()
{

    string input;
    cout << " Add a bracket combination using '()', '{}', and/or '[]' \n" << endl;
    cin >> input;

    string result = isBalanced(input);

    cout << result << endl;

    return 0;
}
