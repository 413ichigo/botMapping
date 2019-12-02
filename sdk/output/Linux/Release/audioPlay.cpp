#include "my.h"
#include<string>

using namespace std;

string trial[3] = {"play audio/A-211.mp3", "play audio/A-209.mp3", "play audio/A-209.mp3"};


int main(int argc, char const *argv[]) {

	usleep(1000000);
	printf("%s\n", argv[1]);
	const char * command = trial[atoi(argv[1])].c_str();
	system(command);
	usleep(1000000);
	return 0;

}

