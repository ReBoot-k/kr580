class Registers {
    // TODO: control input registers (filter, only: A, B, C, D, E, H, L, BC, DE, HL)

    /* Example:
        Register.setValue("A", 66);
        Register.getValue("BC");
    */
    private registers: Record<string, number> = {
        "A": 0,
        "B": 0,
        "C": 0,
        "D": 0,
        "E": 0,
        "H": 0,
        "L": 0,
    }

    private limit(value: number, bits: number = 8): number {
        const maximum = (1 << bits);
        return value % maximum;
    }

    public getValue(register: string): number {
        if (register.length == 1) {
            return this.registers[register];
        } else {
            let registerFirst = register[0];
            let registerSecond = register[1];
            return (this.registers[registerFirst] << 8) | this.registers[registerSecond];
        }
    }

    public setValue(register: string, value: number) {
        if (register.length == 1) {
            this.registers[register] = this.limit(value);
        } else {
            let registerFirst = register[0];
            let registerSecond = register[1];
            this.registers[registerFirst] = this.limit(value) >> 8;
            this.registers[registerSecond] = this.limit(value);
        }
    }
}