#include"sets.h"



/*
typedef struct node_struct{
	int coords[2];
	int came_from[2];
	int t_f;
	int t_g;
	struct node_struct *prev;
}node;


class search_set{
	private:
	node * node_list;
	void add_node(node new_node);
	void remove_node(int coords[2]);
	public:
	//This constructor initializes an empty set, always called at the beginning
	search_set();
	//This constructor initializes a set with a node in it, always called at the beginning
	search_set(node start_node);
	//~search_set();
	void push_node(node new_node);
	node pop_best();
	node pop_requested(int coords[2]);
	bool check_if_in_set(int coords[2]);
	bool isempty();

};*/

search_set::search_set(){
	
	node_list=0;

}

search_set::search_set(node start_node){

	add_node(start_node);

}

void search_set::add_node(node new_node){

	node * n;
	node * ptr=0;
	

	n = new node;
	*n=new_node;
	
	
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

void search_set::push_node(node new_node){

	add_node(new_node);

}

node search_set::pop_best(){

	node * ptr = node_list->prev;
	node * best = node_list;
	node ret;
	
	while(ptr!=0){
		
		if(ptr->t_f < best->t_f)
			best=ptr;
		
		ptr=ptr->prev;
	}
	
	ret=(*best);
	remove_node(best->coords);
	
	return ret;

}

node search_set::pop_requested(int coords[2]){
	
	node * ptr = node_list->prev;
	node ret;

	while(ptr!=0){

		if(ptr->coords[0] == coords[0] && ptr->coords[1] == coords[1])
			return (*ptr);
	}

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

