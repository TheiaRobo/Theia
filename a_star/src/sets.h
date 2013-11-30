#ifndef _SETS_
#define _SETS_

const int NO_VAL=-123456789;
typedef struct node_struct{
	int coords[2];
	int came_from[2];
	int cost;
	struct node_struct *prev;
}node;


class search_set{
	private:
	node * node_list;
	void add_node(int coords[2],int from[2],int cost);
	void remove_node(int coords[2]);
	public:
	//This constructor initializes an empty set, always called at the beginning
	search_set();
	//This constructor initializes a set with a node in it, always called at the beginning
	search_set(int coords[2],int cost);
	//~search_set();
	void push_node(int coords[2],int from[2],int cost);
	node pop_best();
	bool check_if_in_set(int coords[2]);
	bool isempty();
};
	
#endif
