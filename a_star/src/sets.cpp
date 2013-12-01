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
	std::vector<node> node_list;
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
	
	return ;

}

search_set::search_set(node start_node){

	add_node(start_node);

}

void search_set::add_node(node new_node){

	node_list.push_back(new_node);

}

void search_set::remove_node(int coords[2]){
	
	for(std::vector<node>::iterator i=node_list.begin(); i < node_list.end(); i++){
		if(i->coords[0] == coords[0] && i->coords[1] == coords[1])
			node_list.erase(i);
	}
	
	return;

}

void search_set::push_node(node new_node){

	add_node(new_node);

}

node search_set::pop_best(){

	node best = node_list[0];

	for(int i=0; i<node_list.size(); i++){
		if(node_list[i].t_f < best.t_f)
			best=node_list[i];
	}
	
	remove_node(best.coords);
	
	return best;

}

node search_set::pop_requested(int coords[2]){
	
	node ret;
	
	for(int i=0; i < node_list.size(); i++){
		if(node_list[i].coords[0] == coords[0] && node_list[i].coords[1] == coords[1])
			ret=node_list[i];
	}
	
	remove_node(ret.coords);
	
	return ret;	
}

node search_set::read_node(int coords[2]){
	
	node ret;
	
	for(int i=0; i < node_list.size(); i++){
		if(node_list[i].coords[0] == coords[0] && node_list[i].coords[1] == coords[1])
			ret=node_list[i];
	}

	return ret;
	
}


bool search_set::check_if_in_set(int coords[2]){
	
	for(int i=0; i<node_list.size(); i++){
		if(node_list[i].coords[0] == coords[0] && node_list[i].coords[1] == coords[1])
			return true;
	}
	
	return false;

}

bool search_set::isempty(){
	
	if(node_list.size()==0)
		return true;
	
	return false;
}

