#include <iostream>
#include <fstream>

using namespace std;

int main(){
	fstream world;
	world.open("../worlds/table.world");
	
	string line;
	if (world.is_open()){
		getline(world,line);
		cout << line << endl;
	} else{
		cout << "open file ERROR" << endl;
	}

	return 0;
}

