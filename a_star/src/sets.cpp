#include"sets.h"

/*
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
	node_list * nodes;
	void new_node(int coords[2],int cost);
	void add_node(int coords[2],int cost);
	void remove_node(int coords[2]);
	public:
	search_set();
	search_set(int coords[2]);
	~search_set();
	void push_node(int coords[2],int cost);
	node pop_best();
	bool check_if_node(int coords[2]);
};*/

search_set::search_set(){
	
	node_list=0;

}

search_set::search_set(int coords[2],int cost){

	node * new_node;
	int from[2]={-1,-1};
	
	node_list=0;
	add_node(coords,from,cost);

}

void search_set::add_node(int coords[2],int from[2], int cost){

	node * n;
	node * ptr=0;
	

	n = new node;
	n->coords[0]=coords[0];
	n->coords[1]=coords[1];
	n->came_from[0]=from[0];
	n->came_from[1]=from[1];
	n->cost=cost;
	n->prev=0;
	
	if(node_list==0){
		node_list=n;
	}else{
		ptr=node_list;
		node_list=n;
		n->prev=ptr;	
	}

}

void search_set::remove_node(int coords[2]){
	
	node * ptr = node_list;
	node * ptr2 = 0;
	
	while(ptr!=0){
		if(ptr->coords[0]==coords[0] && ptr->coords[1]==coords[1]){
			
			if(ptr2==0){
				node_list=ptr->prev;
			}else{
				ptr2->prev=ptr->prev;
			}
			
			delete ptr;
			return;
		}
		ptr2=ptr;
		ptr=ptr->prev;
	}
	
	return;

}

void search_set::push_node(int coords[2], int from[2],int cost){

	add_node(coords,from,cost);

}

node search_set::pop_best(){

	node * ptr = node_list->prev;
	node * best = node_list;
	node ret;
	
	while(ptr!=0){
		
		if(ptr->cost < best->cost)
			best=ptr;
		
		ptr=ptr->prev;
	}
	
	ret=(*best);
	remove_node(best->coords);
	
	return ret;

}

bool search_set::check_if_in_set(int coords[2]){
	
	node * ptr = node_list;
	
	while(ptr!=0){
	
		if(ptr->coords[0]==coords[0] && ptr->coords[1]==coords[1])
			return true;
		
		ptr=ptr->prev;
	}
	
	return false;

}

bool search_set::isempty(){
	
	if(node_list==0)
		return true;
	
	return false;
}

