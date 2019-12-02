#include "my.h"
#include<string>

using namespace std;

string trial[2] = {"mpg123 ../Desktop/WilhelmScream.mp3", "mpg123 R2screaming.mp3"};


int main(int argc, char const *argv[]) {

	printf("%s\n", argv[1]);
	const char * command = trial[atoi(argv[1])].c_str();
	system(command);
	return 0;

}

