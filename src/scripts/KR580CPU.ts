import { Register, RegisterPair } from "./registers.ts";
import { getPair } from "./tools.ts";

const MAX_SIZE: number = 0xFFFF;

class KR580CPU { 
    private A: Register = new Register();
    private BC: RegisterPair = new RegisterPair();
    private DE: RegisterPair = new RegisterPair();
    private HL: RegisterPair = new RegisterPair();
    
    private PC: number = 0;
    private SP: number = MAX_SIZE;
    private PSW: number = 0;

    private flags: Record<string, Boolean> = {
        S:  false,
        Z:  false,
        AC: false,
        P:  false,
        CY: false
    }

    private memory: Array<number> = Array(MAX_SIZE).fill(0);


    private setFlags(value: number) {
        const bitsPositiveAmount = value.toString(2).replace('0', '').length;

        this.flags.S = (value >= 0b10000000);
	    this.flags.Z = (value == 0);
	    this.flags.P = (bitsPositiveAmount % 2 == 0);
    }

    private setFlagAC(value_1: number, value_2: number) {
        this.flags.AC = (value_1 & 0xF0) != (value_2 & 0xF0);
    }

    private getByte(): number {
        if (this.PC <= MAX_SIZE) {
            return this.memory[this.PC++];
        } 
        return -1; // TODO: следует вызывать исключение или типа того
    }

    private getAddress() {
        let low  = this.getByte();
        let high = this.getByte();
        
        return getPair(high, low);
    }


}