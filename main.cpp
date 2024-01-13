#include "lib/KR580.h"
#include <iostream>


int main() {
	KR580VM80A KR580;
	KR580.execute(Opcode::INR_A);
	std::cout << KR580.A + 0;
	return 0;
}