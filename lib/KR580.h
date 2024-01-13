#pragma once
#include "opcodes.h"

#include <inttypes.h>
#include <unordered_map>
#include <queue>

#define MAX_SIZE   0xFFFF
#define PORTS_SIZE 0x00FF

enum Opcode : uint8_t;

class KR580VM80A {
private:
	uint16_t getPair(uint8_t value_1, uint8_t value_2);
	uint8_t getByte();
	uint16_t getAddress();

	void setFlags(uint8_t value);
	void setFlagAC(uint8_t value_1, uint8_t value_2);

	void add(uint8_t& value_1, uint8_t value_2);
	void adc(uint8_t value);
	void ana(uint8_t value);
	void call();
	void cmp(uint8_t value);
	void daa();
	void dad(uint16_t registerPair);
	void push(uint16_t value);
	uint16_t pop();
	void ora(uint8_t value);
	void sub(uint8_t& value_1, uint8_t value_2);
	void sbb(uint8_t value);
	void lxi(uint16_t& registerPairs);
	void ral();
	void rar();
	void rlc();
	void rrc();
	void rst(uint8_t number);
	void xra(uint8_t value);

public:
	uint8_t A;

	union {
		struct {
			uint8_t C;
			uint8_t B;
		};
		uint16_t BC;
	};

	union {
		struct {
			uint8_t E;
			uint8_t D;
		};
		uint16_t DE;
	};

	union {
		struct {
			uint8_t L;
			uint8_t H;
		};
		uint16_t HL;
	};


	struct {
		bool CY;
		bool P;
		bool AC;
		bool Z;
		bool S;
	} flag;

	uint8_t memory[MAX_SIZE];
	std::unordered_map<uint8_t, std::queue<uint8_t>> in;
	std::unordered_map<uint8_t, std::vector<uint8_t>> out;

	uint16_t PC;
	uint16_t SP;
	uint16_t PSW;

	void execute(uint8_t opcode);
	void init();
	KR580VM80A();
	~KR580VM80A();
};
