#include <bits/stdc++.h>
#include <map>
#include <set>
#include <unordered_set>

using namespace std;

/*
 * Complete the 'sExpression' function below.
 *
 * The function is expected to return a STRING.
 * The function accepts STRING nodes as parameter.
 */

// Although our function takes a string and returns a string, we will add some
// object-oriented programming to make life easier.
// My solution follows this reasoning:
//  1st, we parse the entire string and we construct the binary tree
//  last, we traverse the binary tree and we construct the output.

// There may be a way to construct the output as you traverse the tree,
// to make it faster. But stick to our 2 steps for now.

const string E1 = "E1 Error! One node has more than 2 Children. Please enter a valid Binary Tree";
const string E2 = "E2 Error! We have duplicate edges. please make sure edges have just one parent";
const string E3 = "E3 Error! We have a cycle";
const string E4 = "E4 Error! We have multiple roots";
const string E5 = "E5 Error!";

// Object oriented, create the node structure to keep track of parents and children
// NOTE: Since we have small size and still want to use the node value as the key in a tree
// I will keep all data local to code and not use pointers. Everything will be done by reference (basically same)
class Node {
public:

    //  value of node
    string value  {string("")};

    // the parent of this node (used to check for some errors like loops)
    string parent {string("")};

    // children (I use a set so it is sorted, assuming left is first, with smaller value)
    std::set<string> children;

    // helper functions, get left node
    const string getLeft() const {
        if( children.empty() ){
            return string("");
        }

        return *children.begin();
    }

    // helper functions, get Right node
    const string getRight() const {
        if( children.size() < 2 ) {
            return string("");
        }

        return *(--children.end());
    }

};

// Again, the map could contain pointers Node* instead of node, but since it is small size,
// just do this with reference so we do not worry
bool addNode( const string& parent, const string& child, string& status, map<string, Node>& tree_map, unordered_set<string>& root_nodes ) {


    // check if parent node is already in the map
    if( tree_map.count(parent) > 0 ) {
        // We already have the parent in the node

        // get the parent
        Node& node_parent = tree_map.at(parent);

        // The parent now can either have children or not.
        // And child that we want to add may or may not be already there

        // first, check if parent has 2 childs already, which is an error condition
        if( node_parent.children.size() >= 2 ){

            // 2 children already, if one of the children is the same as ours, E2, else E1
            if( node_parent.children.count(child) > 0 ) {
                // E2
                status = E2;
                return false;
            } else {
                // E1
                status = E1;
                return false;
            }
        }

        // Here, we know that parent has < 2 children. Now we need to check if new children is valid
        // child may or may not be in the map already
        // if child exists in map, it must be root, otherwise we have E3 or E2
        if( tree_map.count(child) > 0 ) {

            // Child is already in the map. get it
            Node& node_child = tree_map.at(child);

            // check if root
            if( node_child.parent.empty() ) {

                // the new child is a sub-tree that started at root so, connect those
                node_child.parent = parent;

                // Add the child
                node_parent.children.insert(child);

                // eliminate from root list
                root_nodes.erase(child);

                // done
                return true;

            } else {

                // the child that exists is not root so, error E3 or E2
                if( node_child.parent == parent ) {
                    // same parent and child, E2
                    status = E2;
                    return false;
                } else {
                    // loop
                    status = E3;
                    return false;
                }

            }

        } else {

            // Child is not in the map, and we have <2 children, so just add the new child
            node_parent.children.insert(child);

            // create child node
            Node node_child;
            node_child.parent = parent;
            node_child.value = child;

            tree_map[child] = node_child;

            return true;
        }



    } else {
        // This is a new parent

        // Now, the children could already be in the map ( we are connecting what was previously a root node to this new parent)
        // or just be a new root and child

        // check if child is in the map
        if( tree_map.count(child) > 0 ){

            // The child is in the map already, get the node
            Node& node_child = tree_map.at(child);

            // Now, if the parent can be empty (it was a root), or not (this is an error)
            if( node_child.parent.empty() ){
                // parent is empty, this is a root. Connect to new parent and deal with it.

                // create new parent node
                Node node_parent;
                node_parent.value = parent;
                node_parent.children.insert(child);

                // Add this new root to list
                root_nodes.insert(parent);

                // Add new parent to map
                tree_map[parent] = node_parent;

                // connect child to new parent
                node_child.parent = parent;

                // remove old root from list of root nodes
                root_nodes.erase(node_child.value);

                // done
                return true;

            } else {
                // parent is not empty, this E3, we have a loop since a child has 2 different parents
                status = E3;
                return false;
            }

        } else {

            // The Child is not in the map, this is a new root or the first branch
            // So we need to create and add both parent and child to map

            // Create new parent (value = parent, parent = empty, children = child)
            Node node_parent;
            node_parent.value = parent;
            node_parent.children.insert(child);

            // Create new child (value = child, parent = parent, children = empty)
            Node node_child;
            node_child.value = child;
            node_child.parent = parent;

            // insert parent to root list
            root_nodes.insert(parent);

            // insert nodes to map
            tree_map[parent] = node_parent;
            tree_map[child] = node_child;

            // done
            return true;

        }
    }

}

void traverseTree( const Node& node, string& output, map<string, Node>& tree_map ) {

    // First, we add the value of the node to output
    output += node.value;

    // Check if we have left node
    auto sLeft = node.getLeft();

    if( sLeft.empty() ) {
        // No left node or children, close and return
        output += ")";
        return;
    } else {
        // we have children, open and continue
        output += "(";
        traverseTree( tree_map.at( sLeft ), output, tree_map );

    }

    // check if we have right node
    auto sRight = node.getRight();

    if( sRight.empty() ){
        // No right node, close and return
        output += ")";
        return;
    } else {
        // we have right child, explore and continue
        output += "(";
        traverseTree( tree_map.at( sRight ), output, tree_map);
    }

}

// Main function
string sExpression(string nodes) {

    // logic to stop the while loop
    // TODO: if we break from the loop, then this would not be necessary
    bool have_solution  {false};

    // The idx of the start of the new group of root-child pair
    size_t idx_start = 0;

    // end of input array to stop while loop once all data has been traversed
    size_t idx_end = nodes.size();

    // Edge case, make sure input has at least one pair?
    if( idx_end < 5) return (E5+" Please enter at least 1 node in the format '(A,B)'");

    // status of adding nodes, to output the proper error type
    string status;

    // Global definition of our map, we populate it on first pass and traverse it on second pass
    // map with the root as the key and the children as the data (Our Binary tree representation)
    map<string, Node> tree_map;

    // Since we do not assume things are added in order, and we want only 1 root, keep track
    // of nodes that are root nodes
    unordered_set<string> root_nodes;


    // while we do not have a solution: keep searching words
    while (!have_solution) {

        // take the next root
        string parent(1, nodes[idx_start+1]);
        string child(1, nodes[idx_start+3]);

        // Add the child to the map, which checks for E1, E2, E3 errors
        if( !addNode(parent, child, status, tree_map, root_nodes) ) {
            // we failed adding node, return error
            return status;
        }

        // if not done, move idx_start to next group
        idx_start += 6;

        if ( idx_start >= idx_end ){
            // end of our input string, stop loop
            have_solution = true;
            break;
        }


    }

    // We have successfully created a tree. Now, check that it has only 1 root. Otherwise, E4
    if( root_nodes.size() > 1 ) {
        return E4;
    }

    // Now we need to explore the tree and generate output, use recursive method.
    string node_start = *root_nodes.begin();
    string output_string = "(";

    traverseTree( tree_map.at( node_start ), output_string, tree_map );

    // done traversing tree, close and return
    output_string += ")";

    return output_string;
}


int main()
{

    string nodes = "(B,D) (D,E) (A,B) (C,F) (E,G) (A,C)";

    string result = sExpression(nodes);

    cout << result << "\n";

    return 0;
}
