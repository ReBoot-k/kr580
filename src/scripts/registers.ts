export {Register, RegisterPair};

/*
const A = new Register();
A.value = 9;
A.value; // значение равно 9

const BC = new RegisterPair(0, 0); // аргументы не обязательны
BC.high // первые 8 бит (регистр B)
BC.low // последние 8 бит (регистр C)
*/

class Register {
    private register: number = 0;

    get value(): number {
        return this.register;
    }

    set value(val: number) {
        this.register = val & 0xFF;
    }
}


class RegisterPair {
    private _low: number;
    private _high: number;

    constructor(high: number = 0, low: number = 0) {
        this._high = high;
        this._low = low;
    }

    get low(): number {
        return this._low;
    }

    set low(value: number) {
        this._low = value & 0xFF;
    }

    get high(): number {
        return this._high;
    }

    set high(value: number) {
        this._high = (value >> 8) & 0xFF;
    }

    get value(): number {
        return (this._high << 8) | this._low;
    }

    set value(val: number) {
        this._low = val & 0xFF;
        this._high = (val >> 8) & 0xFF;
    }
}

