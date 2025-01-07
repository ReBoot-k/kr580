export {getPair, getInverseValue}

function getPair(high: number, low: number): number {
    return (high << 8) | low;
}

function getInverseValue(value: number): number {
    return value ^ 0xFF;
}