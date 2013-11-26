#ifndef _SETS_
#define _SETS_


typedef struct node_struct{
	int coords[2];
	int came_from[2];
	int cost;
}node;

typedef struct _node_list{
	node current;
	struct _node_list * prev;
}node_list;

class search_set{
	private:
	node_list nodes;
	void new_node(int coords[2],int from[2],int cost);
	void add_node(int coords[2],int from[2],int cost);
	void remove_node(int coords[2]);
	public:
	search_set();
	search_set(int coords[2],int from[2]);
	~search_set();
	void push_node(int coords[2],int from[2],int cost);
	node pop_best();
	bool check_if_node(int coords[2]);
};
	
#endif
