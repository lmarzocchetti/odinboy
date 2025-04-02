package src

RegisterFlags :: enum u8 {
    FLAG_ZERO = (1 << 7),
    FLAG_SUBTRACT = (1 << 6),
    FLAG_HALF_CARRY = (1 << 5),
    FLAG_CARRY = (1 << 4)
}

Register :: struct {
    value: u16
}

register_get_hi :: proc(reg: ^Register) -> u8 {
    return cast(u8) (reg.value & 0xFF00) >> 8
}

register_get_low :: proc(reg: ^Register) -> u8 {
    return cast(u8) (reg.value & 0x00FF)
}

register_set_hi :: proc(reg: ^Register, value: u8) {
    tmp_high: u16 = auto_cast value
    tmp_low: u16 = auto_cast register_get_low(reg)
    reg.value = tmp_low | (tmp_high << 8)
}

register_set_low :: proc(reg: ^Register, value: u8) {
    tmp_high: u16 = auto_cast register_get_hi(reg)
    tmp_low: u16 = auto_cast value
    reg.value = tmp_low | (tmp_high << 8)
}

register_inc :: proc(reg: ^Register) {
    // TODO: Controllare se serve settare flags
    reg.value += 1
}

register_dec :: proc(reg: ^Register) {
    // TODO: Come sopra
    reg.value -= 1
}

register_inc_hi :: proc(reg: ^Register, cpu: ^Cpu) {
    cpu_set_flags(cpu, u8(RegisterFlags.FLAG_HALF_CARRY), (register_get_hi(reg) & 0x0f) == 0x0f)

    register_set_hi(reg, register_get_hi(reg) + 1)

    cpu_set_flags(cpu, u8(RegisterFlags.FLAG_ZERO), true if (register_get_hi(reg) == 0) else false)
    cpu_set_flags(cpu, u8(RegisterFlags.FLAG_SUBTRACT), false)
}

register_dec_hi :: proc(reg: ^Register, cpu: ^Cpu) {
    cpu_set_flags(cpu, u8(RegisterFlags.FLAG_HALF_CARRY), true if (register_get_hi(reg) & 0x0f) == 0 else false)

    register_set_hi(reg, register_get_hi(reg) - 1)

    cpu_set_flags(cpu, u8(RegisterFlags.FLAG_ZERO), true if (register_get_hi(reg) == 0) else false)
    cpu_set_flags(cpu, u8(RegisterFlags.FLAG_SUBTRACT), true)
}

register_inc_low :: proc(reg: ^Register, cpu: ^Cpu) {
    cpu_set_flags(cpu, u8(RegisterFlags.FLAG_HALF_CARRY), (register_get_low(reg) & 0x0f) == 0x0f)

    register_set_low(reg, register_get_low(reg) + 1)

    cpu_set_flags(cpu, u8(RegisterFlags.FLAG_ZERO), (register_get_low(reg) == 0))
    cpu_set_flags(cpu, u8(RegisterFlags.FLAG_SUBTRACT), false)
}

register_dec_low :: proc(reg: ^Register, cpu: ^Cpu) {
    cpu_set_flags(cpu, u8(RegisterFlags.FLAG_HALF_CARRY), true if (register_get_low(reg) & 0x0f) == 0 else false)

    register_set_low(reg, register_get_low(reg) - 1)

    cpu_set_flags(cpu, u8(RegisterFlags.FLAG_ZERO), (register_get_low(reg) == 0))
    cpu_set_flags(cpu, u8(RegisterFlags.FLAG_SUBTRACT), true)
}