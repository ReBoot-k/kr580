import { Register, RegisterPair } from "./registers.ts";

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
}