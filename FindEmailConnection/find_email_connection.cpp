// Given a list of contacts as the follows:
//
// |Name | registered emails
// ————————————————————————
//| Tom A.   | abc@gmail.com, bcd@aol.com, xyz@comcast.net
//| Jerry B. | cde@yahoo.com, fgh@gmail.com

// And define:
//(a) Direct connection: if there exists a direct email message between person A and B //
//(b) Indirect connection: if there exists a set of direct connections linking person A and B

// Question:
//Given a set of email messages, check whether Tom A. and Jerry B. are connected or not.
#include <string>
#include <vector>
#include <queue>
#include <unordered_map>

// Existing email APIs
class Email {
 public:
   std::string getFromAddress() const;
   std::vector<std::string> getToAddress() const;
 };


class Node {

public:
  Node* parent {nullptr};


  std::vector< Node* > children;

  string email {""};

  int level {0};


  bool is_visited {false};

};


struct CompareMaxLevelFirst {

  bool operator()(const Node& lhs, const Node& rhs) {
    return ( lhs.level > rhs.level );
  }

};


std::priority_queue< Node*, std::vector<Node*>, CompareMaxLevelFirst > open_nodes;

bool addValidNodes( Node* parent, const std::vector<std::string>& emailAddressJerry,
                  const std::vector<std::string>& emailAddressTom, bool& foundTom, bool& foundJerry) {

  const std::vector<Node*>& children = parent->children;

  for( Node* child : children ) {

    child->level = parent->level + 1;

    if(  isGoalNode( child, emailAddressJerry, emailAddressTom, foundTom, foundJerry ) ) {
     // set connection to true and finish main function
      return true;
    }

    if( !child->is_visited ) {
       open_nodes.push(child);
    }

  }

  return false;

}


bool isGoalNode( Node* n, const std::vector<std::string>& emailAddressJerry,
                const std::vector<std::string>& emailAddressTom, bool& foundTom, bool& foundJerry ) {

  if( std::find( emailAddressJerry.begin(), emailAddressJerry.end(), n->email ) != emailAddressJerry.end() ) {
    // we found a goal
    foundJerry = true;

  }

  if( std::find( emailAddressTom.begin(), emailAddressTom.end(), n->email ) != emailAddressTom.end() ) {
    // we found a goal
    foundTom = true;

  }

  if( foundTom && foundJerry ) {
    return true;
  }


  return false;

}


// Depth first search function
bool dfs( const Node* parent, const std::vector<string>& emailAddressJerry,
         const std::vector<string>& emailAddressTom, bool& foundTom, bool& foundJerry ) {

  if( isGoalNode( parent, emailAddressJerry ) ) {

    return true;
  }

  // set root node to visited
  parent->is_visited = true;

  // add root to the queue
  open_nodes.push( parent );

  // create a flag to check if we have a solution
  bool have_solution {false};

  // create main search loop
  while ( !open_nodes.empty() ) {

    // first, we get the highest priority node
    Node* node_idx = open_nodes.top();
    open_nodes.pop();

    if( addValidNodes( node_idx, emailAddressJerry ) ) {

      return true;

    }


  }

  return false;

}

// TODO: Unique_ptr since this may be a memory mess with new and delete

// Function to be implemented
bool isConnected(const std::vector<std::string>& emailAddressTom, const std::vector<std::string>& emailAddressJerry, const std::vector<Email>& emails) {

//   // first, we can find all email from vector<emails> and create our root
//   std::vector< Node* > vRoots;

//   for( const auto& email : emails ) {

//     if( std::find( emailAddressTom.begin(), emailAddressTom.end(), email.getFromAddress() ) != emailAddressTom.end() )     {
//       // we found the root, create a root
//       Node* root = new Node();

//       root->email = email.getFromAddress();

//

  bool foundTom { false };
  bool foundJerry{ false };

  std::unordered_map< string, std::vector<Node*> > mapTom;

  // for all email in emails:
  //    create a node which has value = from and  children = all of the To

  for( const auto& email : emails ) {

    Node* from = new Node();

    from->email = email.getFromAddress();

    std::vector<string>& sentEmails = email.getToAddress();

    for( size_t idx = 0, idx_end = sentEmails.size(); idx < idx_end; idx++ ) {

      Node* child = new Node();
      child->parent = from;

      from->children.push_back(child);
    }

    if( std::find( emailAddressTom.begin(), emailAddressTom.end(), from->email ) != emailAddressTom.end() ){

      if( mapTom.count( from->email ) > 0 ){

        // already in map
        std::vector<Node*>& possibleRoots = mapTom.at( from->email );
        possibleRoots.push_back( from );


      } else {
        // new
        mapTom[ from->email ] = std::vector< Node* >( from );
      }

    }


  }


  for( auto map_it : mapTom ) {

    const vector<Node*>& possibleRoots = mapTom.second;

    for( Node* root_idx : possibleRoots ) {

      // start the search from this root
      if( dfs( root_idx, emailAddressJerry, emailAddressTom, foundTom, foundJerry ) ) {
        return true;
      }

    }

  }


  return false;


}

