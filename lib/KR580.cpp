#include "KR580.h"

#include <bitset>


void KR580VM80A::init() {

	flag.AC = 0;
	flag.CY = 0;
	flag.P = 0;
	flag.S = 0;
	flag.Z = 0;

	PSW = 0;
	BC = 0;
	DE = 0;
	HL = 0;
	SP = MAX_SIZE;
	PC = 0;
	A = 0;

	memset(memory, 0, MAX_SIZE);

}

KR580VM80A::KR580VM80A() {
	init();
}


KR580VM80A::~KR580VM80A() {
}

uint16_t KR580VM80A::getPair(uint8_t value_1, uint8_t value_2) {
	return (value_1 << 8) | value_2;
}


uint8_t KR580VM80A::getByte() {
	if (PC <= MAX_SIZE) {
		return memory[PC++];
	}
	else {
		exit(0);
	}
}

uint16_t KR580VM80A::getAddress() {
	uint8_t low = getByte();
	uint8_t high = getByte();

	return getPair(high, low);
}

void KR580VM80A::setFlags(uint8_t value) {
	std::bitset<8> bits(value);
	int bits_ones = bits.count();

	flag.S = (value >= 0b10000000);
	flag.Z = (value == 0);
	flag.P = (bits_ones % 2 == 0);
}

void KR580VM80A::setFlagAC(uint8_t value_1, uint8_t value_2) {
	flag.AC = (value_1 & 0xF0) != (value_1 & 0xF0);
}


void KR580VM80A::add(uint8_t& value_1, uint8_t value_2) {
	flag.CY = value_1 > ((uint8_t)0xFF - value_2);

	uint8_t registerOld = value_1;
	value_1 += value_2;

	setFlags(value_1);
	setFlagAC(registerOld, value_1);
}


void KR580VM80A::adc(uint8_t value) {
	uint8_t registerOld = A;
	add(A, value);

	bool carryOld = flag.CY;

	add(A, flag.CY);
	flag.CY |= carryOld;
	setFlagAC(registerOld, A);
}

void KR580VM80A::ana(uint8_t value) {
	A &= value;
	setFlags(A);
	flag.AC = 0;
	flag.CY = 0;
}

void KR580VM80A::call() {
	push(PC + 3);
	PC = getAddress();
}

void KR580VM80A::cmp(uint8_t value) {
	sub(A, value);
	setFlags(A);
	A += value;
}

void KR580VM80A::daa() {
	uint8_t high = A >> 4;
	uint8_t low = A & 0x0F;

	if (high > 9 || flag.CY) {
		high += 6;
	}

	if (low > 9 || flag.AC) {
		low += 6;
	}


	A = (high << 4) + low;
}

void KR580VM80A::dad(uint16_t registerPair) {
	if (HL > ((uint16_t)0xFFFF - registerPair))
		flag.CY = 1;
	HL += registerPair;
}


void KR580VM80A::push(uint16_t value) {
	uint8_t high = value >> 8;
	uint8_t low = value & 0x00FF;

	memory[SP == MAX_SIZE ? SP : --SP] = high;
	memory[--SP] = low;
}

uint16_t KR580VM80A::pop() {
	uint8_t low = memory[SP];
	uint8_t high = memory[++SP];
	SP = SP == MAX_SIZE ? MAX_SIZE : ++SP;

	return getPair(high, low);
}

void KR580VM80A::ora(uint8_t value) {
	A |= value;
	setFlags(A);
	flag.AC = 0;
	flag.CY = 0;
}

void KR580VM80A::sub(uint8_t& value_1, uint8_t value_2) {
	flag.CY = value_1 < value_2;
	uint8_t registerOld = value_1;
	value_1 -= value_2;
	setFlags(value_1);
	setFlagAC(registerOld, value_1);
}

void KR580VM80A::sbb(uint8_t value) {
	uint8_t registerOld = A;
	sub(A, value);

	bool carryOld = flag.CY;

	sub(A, flag.CY);
	flag.CY |= carryOld;
	setFlagAC(registerOld, A);
}

void KR580VM80A::lxi(uint16_t& registerPairs) {
	registerPairs = getAddress();
}

void KR580VM80A::ral() {
	bool highBit = A >> 7;
	A <<= 1;
	A |= (uint8_t)flag.CY;
	flag.CY = highBit;
}

void KR580VM80A::rar() {
	bool highBit = A & 0x01;
	A >>= 1;
	A &= 0b01111111;
	A |= (uint8_t)flag.CY << 7;
	flag.CY = highBit;
}

void KR580VM80A::rlc() {
	bool highBit = (A >> 7) & 0x01;
	A <<= 1;
	A |= (uint8_t)highBit;
	flag.CY = highBit;
}

void KR580VM80A::rrc() {
	bool highBit = A & 0x01;
	A >>= 1;
	A &= 0b01111111;
	A |= (uint8_t)highBit << 7;
	flag.CY = highBit;
}

void KR580VM80A::rst(uint8_t number) {
	push(PC);
	PC = number * 0x08;
}

void KR580VM80A::xra(uint8_t value) {
	A ^= value;

	setFlags(A);
	flag.AC = 0;
	flag.CY = 0;
}


void KR580VM80A::execute(uint8_t opcode) {
	switch (opcode) {
	case NOP:
		break;
	case NOP_8:
	case NOP_10:
	case NOP_18:
	case NOP_20:
	case NOP_28:
	case NOP_30:
	case NOP_38:
	case RET_D9:
	case JMP_CB:
	case CALL_DD:
	case CALL_ED:
	case CALL_FD:
		exit(0);
		break;
	case ADD_A:
		add(A, A);
		break;
	case ADD_B:
		add(A, B);
		break;
	case ADD_C:
		add(A, C);
		break;
	case ADD_D:
		add(A, D);
		break;
	case ADD_E:
		add(A, E);
		break;
	case ADD_H:
		add(A, H);
		break;
	case ADD_L:
		add(A, L);
		break;
	case ADD_M:
		add(A, memory[HL]);
		break;
	case ADI:
		add(A, getByte());
		break;
	case ADC_A:
		adc(A);
		break;
	case ADC_B:
		adc(B);
		break;
	case ADC_C:
		adc(C);
		break;
	case ADC_D:
		adc(D);
		break;
	case ADC_E:
		adc(E);
		break;
	case ADC_H:
		adc(H);
		break;
	case ADC_L:
		adc(L);
		break;
	case ADC_M:
		adc(memory[HL]);
		break;
	case ACI:
		adc(getByte());
		break;
	case ANA_A:
		ana(A);
		break;
	case ANA_B:
		ana(B);
		break;
	case ANA_C:
		ana(C);
		break;
	case ANA_D:
		ana(D);
		break;
	case ANA_E:
		ana(E);
		break;
	case ANA_H:
		ana(H);
		break;
	case ANA_L:
		ana(L);
		break;
	case ANA_M:
		ana(memory[HL]);
		break;
	case ANI:
		ana(getByte());
		break;
	case CALL:
		call();
		break;
	case CZ:
		if (flag.Z) { call(); }
		else { PC += 3; }
		break;
	case CNZ:
		if (!flag.Z) { call(); }
		else { PC += 3; }
		break;
	case CP:
		if (!flag.S) { call(); }
		else { PC += 3; }
		break;
	case CM:
		if (flag.S) { call(); }
		else { PC += 3; }
		break;
	case CC:
		if (flag.CY) { call(); }
		else { PC += 3; }
		break;
	case CNC:
		if (!flag.CY) { call(); }
		else { PC += 3; }
		break;
	case CPE:
		if (flag.P) { call(); }
		else { PC += 3; }
		break;
	case CPO:
		if (!flag.P) { call(); }
		else { PC += 3; }
		break;
	case CMA:
		A ^= 0xFF;
		break;
	case CMC:
		flag.CY = !flag.CY;
		break;
	case CMP_A:
		cmp(A);
		break;
	case CMP_B:
		cmp(B);
		break;
	case CMP_C:
		cmp(C);
		break;
	case CMP_D:
		cmp(D);
		break;
	case CMP_E:
		cmp(E);
		break;
	case CMP_H:
		cmp(H);
		break;
	case CMP_L:
		cmp(L);
		break;
	case CMP_M:
		cmp(memory[HL]);
		break;
	case CPI:
		cmp(getByte());
		break;
	case DAA:
		daa();
		break;
	case DAD_B:
		dad(BC);
		break;
	case DAD_D:
		dad(DE);
		break;
	case DAD_H:
		dad(HL);
		break;
	case DAD_SP:
		dad(SP);
		break;
	case DCR_A:
		sub(A, 1);
		break;
	case DCR_B:
		sub(B, 1);
		break;
	case DCR_C:
		sub(C, 1);
		break;
	case DCR_D:
		sub(D, 1);
		break;
	case DCR_E:
		sub(E, 1);
		break;
	case DCR_H:
		sub(H, 1);
		break;
	case DCR_L:
		sub(L, 1);
		break;
	case DCR_M:
		sub(memory[HL], 1);
		break;
	case DCX_B:
		BC--;
		break;
	case DCX_D:
		DE--;
		break;
	case DCX_H:
		HL--;
		break;
	case DCX_SP:
		SP--;
		break;
	case DI:
	case EI:
	case HLT:
		// TODO: ÑÄÅËÀÉ ÝÒÎ
		break;
	case IN: {
		std::queue<uint8_t> port = in[getByte()];
		A = port.front();
		port.pop();
	}
			break;
	case INR_A:
		add(A, 1);
		break;
	case INR_B:
		add(B, 1);
		break;
	case INR_C:
		add(C, 1);
		break;
	case INR_D:
		add(D, 1);
		break;
	case INR_E:
		add(E, 1);
		break;
	case INR_H:
		add(H, 1);
		break;
	case INR_L:
		add(L, 1);
		break;
	case INR_M:
		add(memory[HL], 1);
		break;
	case INX_B:
		BC++;
		break;
	case INX_D:
		DE++;
		break;
	case INX_H:
		HL++;
		break;
	case INX_SP:
		SP++;
		break;
	case JMP:
		PC = getAddress();
		break;
	case JZ:
		if (flag.Z) { PC = getAddress(); }
		else { PC += 3; }
		break;
	case JNZ:
		if (!flag.Z) { PC = getAddress(); }
		else { PC += 3; }
		break;
	case JP:
		if (!flag.S) { PC = getAddress(); }
		else { PC += 3; }
		break;
	case JM:
		if (flag.S) { PC = getAddress(); }
		else { PC += 3; }
		break;
	case JC:
		if (flag.CY) { PC = getAddress(); }
		else { PC += 3; }
		break;
	case JNC:
		if (!flag.CY) { PC = getAddress(); }
		else { PC += 3; }
		break;
	case JPE:
		if (flag.P) { PC = getAddress(); }
		else { PC += 3; }
		break;
	case JPO:
		if (!flag.P) { PC = getAddress(); }
		else { PC += 3; }
		break;
	case LDA:
		A = memory[getAddress()];
		break;
	case LDAX_B:
		A = memory[BC];
		break;
	case LDAX_D:
		A = memory[DE];
		break;
	case LHLD: {
		uint16_t address = getAddress();
		H = memory[address + 1];
		L = memory[address];
	}
				break;
	case LXI_B:
		lxi(BC);
		break;
	case LXI_D:
		lxi(DE);
		break;
	case LXI_H:
		lxi(HL);
		break;
	case LXI_SP:
		lxi(SP);
		break;
	case MOV_A_A:
		A = A;
		break;
	case MOV_A_B:
		A = B;
		break;
	case MOV_A_C:
		A = C;
		break;
	case MOV_A_D:
		A = D;
		break;
	case MOV_A_E:
		A = E;
		break;
	case MOV_A_H:
		A = H;
		break;
	case MOV_A_L:
		A = L;
		break;
	case MOV_A_M:
		A = memory[HL];
		break;
	case MOV_B_A:
		B = A;
		break;
	case MOV_B_B:
		B = B;
		break;
	case MOV_B_C:
		B = C;
		break;
	case MOV_B_D:
		B = D;
		break;
	case MOV_B_E:
		B = E;
		break;
	case MOV_B_H:
		B = H;
		break;
	case MOV_B_L:
		B = L;
		break;
	case MOV_B_M:
		B = memory[HL];
		break;
	case MOV_C_A:
		C = A;
		break;
	case MOV_C_B:
		C = B;
		break;
	case MOV_C_C:
		C = C;
		break;
	case MOV_C_D:
		C = D;
		break;
	case MOV_C_E:
		C = E;
		break;
	case MOV_C_H:
		C = H;
		break;
	case MOV_C_L:
		C = L;
		break;
	case MOV_C_M:
		C = memory[HL];
		break;
	case MOV_D_A:
		D = A;
		break;
	case MOV_D_B:
		D = B;
		break;
	case MOV_D_C:
		D = C;
		break;
	case MOV_D_D:
		D = D;
		break;
	case MOV_D_E:
		D = E;
		break;
	case MOV_D_H:
		D = H;
		break;
	case MOV_D_L:
		D = L;
		break;
	case MOV_D_M:
		D = memory[HL];
		break;
	case MOV_E_A:
		E = A;
		break;
	case MOV_E_B:
		E = B;
		break;
	case MOV_E_C:
		E = C;
		break;
	case MOV_E_D:
		E = D;
		break;
	case MOV_E_E:
		E = E;
		break;
	case MOV_E_H:
		E = H;
		break;
	case MOV_E_L:
		E = L;
		break;
	case MOV_E_M:
		E = memory[HL];
		break;
	case MOV_H_A:
		H = A;
		break;
	case MOV_H_B:
		H = B;
		break;
	case MOV_H_C:
		H = C;
		break;
	case MOV_H_D:
		H = D;
		break;
	case MOV_H_E:
		H = E;
		break;
	case MOV_H_H:
		H = H;
		break;
	case MOV_H_L:
		H = L;
		break;
	case MOV_H_M:
		H = memory[HL];
		break;
	case MOV_L_A:
		L = A;
		break;
	case MOV_L_B:
		L = B;
		break;
	case MOV_L_C:
		L = C;
		break;
	case MOV_L_D:
		L = D;
		break;
	case MOV_L_E:
		L = E;
		break;
	case MOV_L_H:
		L = H;
		break;
	case MOV_L_L:
		L = L;
		break;
	case MOV_L_M:
		L = memory[HL];
		break;
	case MOV_M_A:
		memory[HL] = A;
		break;
	case MOV_M_B:
		memory[HL] = B;
		break;
	case MOV_M_C:
		memory[HL] = C;
		break;
	case MOV_M_D:
		memory[HL] = D;
		break;
	case MOV_M_E:
		memory[HL] = E;
		break;
	case MOV_M_H:
		memory[HL] = H;
		break;
	case MOV_M_L:
		memory[HL] = L;
		break;
	case MVI_A:
		A = getByte();
		break;
	case MVI_B:
		B = getByte();
		break;
	case MVI_C:
		C = getByte();
		break;
	case MVI_D:
		D = getByte();
		break;
	case MVI_E:
		E = getByte();
		break;
	case MVI_H:
		H = getByte();
		break;
	case MVI_L:
		L = getByte();
		break;
	case MVI_M:
		memory[HL] = getByte();
		break;
	case ORA_A:
		ora(A);
		break;
	case ORA_B:
		ora(B);
		break;
	case ORA_C:
		ora(C);
		break;
	case ORA_D:
		ora(D);
		break;
	case ORA_E:
		ora(E);
		break;
	case ORA_H:
		ora(H);
		break;
	case ORA_L:
		ora(L);
		break;
	case ORA_M:
		ora(memory[HL]);
		break;
	case ORI:
		ora(getByte());
		break;
	case OUT:
		out[getByte()].push_back(A);
		break;
	case PCHL:
		PC = HL;
		break;
	case POP_B:
		BC = pop();
		break;
	case POP_D:
		DE = pop();
		break;
	case POP_H:
		HL = pop();
		break;
	case POP_PSW:
		PSW = pop();
		break;
	case PUSH_B:
		push(BC);
		break;
	case PUSH_D:
		push(DE);
		break;
	case PUSH_H:
		push(HL);
		break;
	case PUSH_PSW:
		push(PSW);
		break;
	case RAL:
		ral();
		break;
	case RAR:
		rar();
		break;
	case RLC:
		rlc();
		break;
	case RRC:
		rrc();
		break;
	case RET:
		PC = pop();
		break;
	case RZ:
		if (flag.Z) { PC = pop(); }
		else { PC += 3; }
		break;
	case RNZ:
		if (!flag.Z) { PC = pop(); }
		else { PC += 3; }
		break;
	case RP:
		if (!flag.S) { PC = pop(); }
		else { PC += 3; }
		break;
	case RM:
		if (flag.S) { PC = pop(); }
		else { PC += 3; }
		break;
	case RC:
		if (flag.CY) { PC = pop(); }
		else { PC += 3; }
		break;
	case RNC:
		if (!flag.CY) { PC = pop(); }
		else { PC += 3; }
		break;
	case RPE:
		if (flag.P) { PC = pop(); }
		else { PC += 3; }
		break;
	case RPO:
		if (!flag.P) { PC = pop(); }
		else { PC += 3; }
		break;
	case RST_0:
		rst(0);
		break;
	case RST_1:
		rst(1);
		break;
	case RST_2:
		rst(2);
		break;
	case RST_3:
		rst(3);
		break;
	case RST_4:
		rst(4);
		break;
	case RST_5:
		rst(5);
		break;
	case RST_6:
		rst(6);
		break;
	case RST_7:
		rst(7);
		break;
	case SPHL:
		SP = HL;
		break;
	case SHLD: {
		uint16_t address = getAddress();
		memory[address + 1] = H;
		memory[address] = L;
	} break;
	case STA:
		memory[getAddress()] = A;
		break;
	case STAX_B:
		memory[BC] = A;
		break;
	case STAX_D:
		memory[DE] = A;
		break;
	case STC:
		flag.CY = true;
		break;
	case SUB_A:
		sub(A, A);
		break;
	case SUB_B:
		sub(A, B);
		break;
	case SUB_C:
		sub(A, C);
		break;
	case SUB_D:
		sub(A, D);
		break;
	case SUB_E:
		sub(A, E);
		break;
	case SUB_H:
		sub(A, H);
		break;
	case SUB_L:
		sub(A, L);
		break;
	case SUB_M:
		sub(A, memory[HL]);
		break;
	case SUI:
		sub(A, getByte());
		break;
	case SBB_A:
		sbb(A);
		break;
	case SBB_B:
		sbb(B);
		break;
	case SBB_C:
		sbb(C);
		break;
	case SBB_D:
		sbb(D);
		break;
	case SBB_E:
		sbb(E);
		break;
	case SBB_H:
		sbb(H);
		break;
	case SBB_L:
		sbb(L);
		break;
	case SBB_M:
		sbb(memory[HL]);
		break;
	case SBI:
		sbb(getByte());
		break;
	case XCHG: {
		uint16_t temp = HL;
		HL = DE;
		DE = temp;
	} break;
	case XTHL: {
		uint16_t temp = HL;
		HL = pop();
		push(temp);
	} break;
	case XRA_A:
		xra(A);
		break;
	case XRA_B:
		xra(B);
		break;
	case XRA_C:
		xra(C);
		break;
	case XRA_D:
		xra(D);
		break;
	case XRA_E:
		xra(E);
		break;
	case XRA_H:
		xra(H);
		break;
	case XRA_L:
		xra(L);
		break;
	case XRI:
		xra(getByte());
		break;
	}

}
