package src

import "core:fmt"
import "core:os/os2"
import "core:math/bits"

Clock :: struct {
    t: u32,
    t_instr: u32
}

instruction_ticks: [256]u8 = {
    4, 12, 8, 8, 4, 4, 8, 4, 20, 8, 8, 8, 4, 4, 8, 4, // 0x0_
    4, 12, 8, 8, 4, 4, 8, 4, 12, 8, 8, 8, 4, 4, 8, 4, // 0x1_
    0, 12, 8, 8, 4, 4, 8, 4, 0, 8, 8, 8, 4, 4, 8, 4, // 0x2_
    0, 12, 8, 8, 12, 12, 12, 4, 0, 8, 8, 8, 4, 4, 8, 4, // 0x3_
    4, 4, 4, 4, 4, 4, 8, 4, 4, 4, 4, 4, 4, 4, 8, 4, // 0x4_
    4, 4, 4, 4, 4, 4, 8, 4, 4, 4, 4, 4, 4, 4, 8, 4, // 0x5_
    4, 4, 4, 4, 4, 4, 8, 4, 4, 4, 4, 4, 4, 4, 8, 4, // 0x6_
    8, 8, 8, 8, 8, 8, 4, 8, 4, 4, 4, 4, 4, 4, 8, 4, // 0x7_
    4, 4, 4, 4, 4, 4, 8, 4, 4, 4, 4, 4, 4, 4, 8, 4, // 0x8_
    4, 4, 4, 4, 4, 4, 8, 4, 4, 4, 4, 4, 4, 4, 8, 4, // 0x9_
    4, 4, 4, 4, 4, 4, 8, 4, 4, 4, 4, 4, 4, 4, 8, 4, // 0xa_
    4, 4, 4, 4, 4, 4, 8, 4, 4, 4, 4, 4, 4, 4, 8, 4, // 0xb_
    0, 12, 0, 16, 0, 16, 8, 16, 0, 16, 0, 0, 0, 24, 8, 16, // 0xc_
    0, 12, 0, 0, 0, 16, 8, 16, 0, 16, 0, 0, 0, 0, 8, 16, // 0xd_
    12, 12, 8, 0, 0, 16, 8, 16, 16, 4, 16, 0, 0, 0, 8, 16, // 0xe_
    12, 12, 8, 4, 0, 16, 8, 16, 12, 8, 16, 4, 0, 0, 8, 16, // 0xf_
}

extended_instruction_ticks: [256]u8 = {
    8, 8, 8, 8, 8, 8, 16, 8, 8, 8, 8, 8, 8, 8, 16, 8, // 0x0_
    8, 8, 8, 8, 8, 8, 16, 8, 8, 8, 8, 8, 8, 8, 16, 8, // 0x1_
    8, 8, 8, 8, 8, 8, 16, 8, 8, 8, 8, 8, 8, 8, 16, 8, // 0x2_
    8, 8, 8, 8, 8, 8, 16, 8, 8, 8, 8, 8, 8, 8, 16, 8, // 0x3_
    8, 8, 8, 8, 8, 8, 12, 8, 8, 8, 8, 8, 8, 8, 12, 8, // 0x4_
    8, 8, 8, 8, 8, 8, 12, 8, 8, 8, 8, 8, 8, 8, 12, 8, // 0x5_
    8, 8, 8, 8, 8, 8, 12, 8, 8, 8, 8, 8, 8, 8, 12, 8, // 0x6_
    8, 8, 8, 8, 8, 8, 12, 8, 8, 8, 8, 8, 8, 8, 12, 8, // 0x7_
    8, 8, 8, 8, 8, 8, 16, 8, 8, 8, 8, 8, 8, 8, 16, 8, // 0x8_
    8, 8, 8, 8, 8, 8, 16, 8, 8, 8, 8, 8, 8, 8, 16, 8, // 0x9_
    8, 8, 8, 8, 8, 8, 16, 8, 8, 8, 8, 8, 8, 8, 16, 8, // 0xa_
    8, 8, 8, 8, 8, 8, 16, 8, 8, 8, 8, 8, 8, 8, 16, 8, // 0xb_
    8, 8, 8, 8, 8, 8, 16, 8, 8, 8, 8, 8, 8, 8, 16, 8, // 0xc_
    8, 8, 8, 8, 8, 8, 16, 8, 8, 8, 8, 8, 8, 8, 16, 8, // 0xd_
    8, 8, 8, 8, 8, 8, 16, 8, 8, 8, 8, 8, 8, 8, 16, 8, // 0xe_
    8, 8, 8, 8, 8, 8, 16, 8, 8, 8, 8, 8, 8, 8, 16, 8, // 0xf_
}

Cpu :: struct {
    // AF splitted in bit: 4=Carry flag, 5=Half Carry Flag, 6=Subtraction Flag,
    // 7=Zero Flag, the high 8 bit is the Accumulator
    ir_ie, af, bc, de, hl: Register,
    pc, sp: u16,
    memory: Memory,
    clock: Clock,
    interrupts: Interrupt
}

cpu_init :: proc() -> Cpu {
    return Cpu{
        memory = memory_create()
    }
}

cpu_set_flags :: proc(cpu: ^Cpu, flags: u8, condition: bool) {
    curr_flags := register_get_low(&cpu.af)
    new_flags := (curr_flags | flags) if condition else (curr_flags & (~flags))
    register_set_low(&cpu.af, new_flags)
}

cpu_is_flag_set :: proc(cpu: ^Cpu, flag: u8) -> bool {
    return true if (register_get_low(&cpu.af) & flag) == 1 else false
}

cpu_xor_a :: proc(cpu: ^Cpu, value: u8) {
    using RegisterFlags
    register_set_hi(&cpu.af, register_get_hi(&cpu.af) ~ value)

    state := true if register_get_hi(&cpu.af) == 0 else false
    cpu_set_flags(cpu, u8(FLAG_ZERO), state)
    cpu_set_flags(cpu, u8(FLAG_CARRY) | u8(FLAG_SUBTRACT) | u8(FLAG_HALF_CARRY), false)
}

cpu_jump_add :: proc(cpu: ^Cpu, condition: bool) {
    if condition {
        cpu.pc += 1 + u16(memory_read_byte(&cpu.memory, cpu.pc))
        cpu.clock.t_instr += 12
    } else {
        cpu.pc += 1
        cpu.clock.t_instr += 8
    }
}

cpu_bit :: proc(cpu: ^Cpu, bit: u8, value: u8) {
    using RegisterFlags
    state := true if (value & bit) == 0 else false
    cpu_set_flags(cpu, u8(FLAG_ZERO), state)
    cpu_set_flags(cpu, u8(FLAG_HALF_CARRY), true)
    cpu_set_flags(cpu, u8(FLAG_SUBTRACT), false)
}

cpu_rl :: proc(cpu: ^Cpu, value: u8) -> u8 {
    using RegisterFlags
    retval := value
    carry := cpu_is_flag_set(cpu, u8(FLAG_CARRY))

    cpu_set_flags(cpu, u8(FLAG_CARRY), false if (retval & (1 << 7)) == 0 else true)

    retval <<= 1
    retval += u8(carry) // TODO: Verifica

    cpu_set_flags(cpu, u8(FLAG_ZERO), retval == 0)
    cpu_set_flags(cpu, u8(FLAG_SUBTRACT) | u8(FLAG_HALF_CARRY), false)

    return retval
}

cpu_rlc :: proc(cpu: ^Cpu, value: u8) -> u8 {
    using RegisterFlags
    
    retval := value
    carry := (retval >> 7) & 0x01

    cpu_set_flags(cpu, u8(FLAG_CARRY), false if (retval * (1 << 7)) == 0 else true)

    retval <<= 1
    retval += carry

    cpu_set_flags(cpu, u8(FLAG_ZERO), true if retval == 0 else false)
    cpu_set_flags(cpu, u8(FLAG_SUBTRACT) | u8(FLAG_HALF_CARRY), false)

    return retval
}

cpu_rr :: proc(cpu: ^Cpu, value: u8) -> u8 {
    using RegisterFlags
    retval := value
    carry := cpu_is_flag_set(cpu, u8(FLAG_CARRY))

    cpu_set_flags(cpu, u8(FLAG_CARRY), false if value & 0x01 == 0 else true)
    retval >>= 1
    retval |= u8(carry) << 7

    cpu_set_flags(cpu, u8(FLAG_ZERO), retval == 0)
    cpu_set_flags(cpu, u8(FLAG_SUBTRACT) | u8(FLAG_HALF_CARRY), false)

    return retval
}

cpu_rrc :: proc(cpu: ^Cpu, value: u8) -> u8 {
    using RegisterFlags
    retval := value
    carry := retval & 0x01

    cpu_set_flags(cpu, u8(FLAG_CARRY), false if carry == 0 else true)
    retval >>= 1
    retval |= (carry << 7)

    cpu_set_flags(cpu, u8(FLAG_ZERO), retval == 0)
    cpu_set_flags(cpu, u8(FLAG_SUBTRACT) | u8(FLAG_HALF_CARRY), false)

    return retval
}

cpu_add_u16_u16 :: proc(cpu: ^Cpu, destination: u16, value: u16) -> u16 {
    using RegisterFlags
    result: u32 = auto_cast (destination + value)

    cpu_set_flags(cpu, u8(FLAG_CARRY), (result > 0xFFFF))
    cpu_set_flags(cpu, u8(FLAG_HALF_CARRY), ((destination & 0x0FFF) + (value & 0x0FFF)) > 0x0FFF)
    cpu_set_flags(cpu, u8(FLAG_SUBTRACT), false)

    return u16(result)
}

cpu_inc :: proc(cpu: ^Cpu, value: u8) -> u8 {
    using RegisterFlags
    value := value
    
    cpu_set_flags(cpu, u8(FLAG_HALF_CARRY), (value & 0x0F) == 0x0F)

    value = value + 1

    cpu_set_flags(cpu, u8(FLAG_ZERO), true if value == 0 else false)
    cpu_set_flags(cpu, u8(FLAG_SUBTRACT), false)

    return value
}

cpu_dec :: proc(cpu: ^Cpu, value: u8) -> u8 {
    using RegisterFlags
    value := value
    
    cpu_set_flags(cpu, u8(FLAG_HALF_CARRY), true if (value & 0x0F) == 0 else false)

    value = value - 1

    cpu_set_flags(cpu, u8(FLAG_ZERO), true if value == 0 else false)
    cpu_set_flags(cpu, u8(FLAG_SUBTRACT), true)

    return value
}

cpu_extended_execute :: proc(cpu: ^Cpu, opcode: u8) {
    cpu.clock.t_instr += u32(extended_instruction_ticks[opcode])

    switch opcode {
        case 0x7c:
            cpu_bit(cpu, (1 << 7), register_get_hi(&cpu.hl))
        case:
            fmt.printfln("OpCode %#x not implemented", opcode)
            os2.exit(1)
    }
}

cpu_execute_instruction :: proc(cpu: ^Cpu, opcode: u8) {
    using RegisterFlags
    cpu.clock.t_instr += auto_cast instruction_ticks[opcode]

    switch opcode {
        case 0x00: // NOP
            // fmt.printfln("DEBUG: %d", cpu.pc)
            // os2.exit(1)
        case 0x01: // LD BC, nn
            cpu.bc.value = memory_read_short(&cpu.memory, cpu.pc)
            cpu.pc += 2
        case 0x02: // LD (BC), A
            memory_write_byte(&cpu.memory, cpu.bc.value, register_get_hi(&cpu.af))
        case 0x03: // INC BC
            register_inc(&cpu.bc)
        case 0x04: // INC B
            register_inc_hi(&cpu.bc, cpu)
        case 0x05: // DEC B
            register_dec_hi(&cpu.bc, cpu)
        case 0x06: // LD B, n
            register_set_hi(&cpu.bc, memory_read_byte(&cpu.memory, cpu.pc))
            cpu.pc += 1
        case 0x07: // RLCA
            register_set_hi(&cpu.af, cpu_rlc(cpu, register_get_hi(&cpu.af)))
            cpu_set_flags(cpu, u8(FLAG_ZERO), false)
        case 0x08: // LD (nn), SP
            memory_write_short(&cpu.memory, memory_read_short(&cpu.memory, cpu.pc), cpu.sp)
            cpu.pc += 2
        case 0x09: // ADD HL, BC
            cpu.hl.value = cpu_add_u16_u16(cpu, cpu.hl.value, cpu.bc.value)
        case 0x0A: // LD A, (BC)
            register_set_hi(&cpu.af, memory_read_byte(&cpu.memory, cpu.bc.value))
        case 0x0B: // DEC BC
            register_dec(&cpu.bc)
        case 0x0C: // INC C
            register_inc_low(&cpu.bc, cpu)
        case 0x0D: // DEC C
            register_dec_low(&cpu.bc, cpu)
        case 0x0E: // LD C, n
            register_set_low(&cpu.bc, memory_read_byte(&cpu.memory, cpu.pc))
            cpu.pc += 1
        case 0x0F: // RRCA
            register_set_hi(&cpu.af, cpu_rrc(cpu, register_get_hi(&cpu.af)))
            cpu_set_flags(cpu, u8(FLAG_ZERO), false)
        case 0x10: // STOP
        case 0x11: // LD DE, nn
            cpu.de.value = memory_read_short(&cpu.memory, cpu.pc)
            cpu.pc += 2
        case 0x12: // LD (DE), A
            memory_write_byte(&cpu.memory, cpu.de.value, register_get_hi(&cpu.af))
        case 0x13: // INC DE
            register_inc(&cpu.de)
        case 0x14: // INC D
            register_inc_hi(&cpu.de, cpu)
        case 0x15: // DEC D
            register_dec_hi(&cpu.de, cpu)
        case 0x16: // LD D, n
            register_set_hi(&cpu.de, memory_read_byte(&cpu.memory, cpu.pc))
            cpu.pc += 1
        case 0x17: // RLA
            register_set_hi(&cpu.af, cpu_rl(cpu, register_get_hi(&cpu.af)))
            cpu_set_flags(cpu, u8(FLAG_ZERO), false)
        case 0x18: // JR nn
            // TODO: check better
            operand: u8 = memory_read_byte(&cpu.memory, cpu.pc)
            new_pc: i32 = i32(cpu.pc) + 1 + i32(i8(operand))
            cpu.pc = u16(new_pc)
        case 0x19: // ADD HL, DE
            cpu.hl.value = cpu_add_u16_u16(cpu, cpu.hl.value, cpu.de.value)
        case 0x1A: // LD A, (DE)
            register_set_hi(&cpu.af, memory_read_byte(&cpu.memory, cpu.de.value))
        case 0x1B: // DEC DE
            register_dec(&cpu.de)
        case 0x1C: // INC E
            register_inc_low(&cpu.de, cpu)
        case 0x1D: // DEC E
            register_dec_low(&cpu.de, cpu)
        case 0x1E: // LD E, n
            register_set_low(&cpu.de, memory_read_byte(&cpu.memory, cpu.pc))
            cpu.pc += 1
        case 0x1F: // RRA
            register_set_hi(&cpu.af, cpu_rr(cpu, register_get_hi(&cpu.af)))
            cpu_set_flags(cpu, u8(FLAG_ZERO), false)
        case 0x20: // JR NZ, *
            cpu_jump_add(cpu, !cpu_is_flag_set(cpu, u8(FLAG_ZERO)))
        case 0x21: // LD HL, nn
            cpu.hl.value = memory_read_short(&cpu.memory, cpu.pc)
            cpu.pc += 2
        case 0x22: // LD (HLI), A | LD (HL+), A | LDI (HL), A
            memory_write_byte(&cpu.memory, cpu.hl.value, register_get_hi(&cpu.af))
            register_inc(&cpu.hl)
        case 0x23: // INC HL
            register_inc(&cpu.hl)
        case 0x24: // INC H
            register_inc_hi(&cpu.hl, cpu)
        case 0x25: // DEC H
            register_dec_hi(&cpu.hl, cpu)
        case 0x26: // LD H, n
            register_set_hi(&cpu.hl, memory_read_byte(&cpu.memory, cpu.pc))
            cpu.pc += 1
        case 0x27: // DAA
            value: u16 = u16(register_get_hi(&cpu.af))

            if cpu_is_flag_set(cpu, u8(FLAG_SUBTRACT)) {
                if cpu_is_flag_set(cpu, u8(FLAG_CARRY)) {
                    value -= 0x60
                }

                if cpu_is_flag_set(cpu, u8(FLAG_HALF_CARRY)) {
                    value -= 0x6
                }
            } else {
                if cpu_is_flag_set(cpu, u8(FLAG_CARRY)) || value > 0x99 {
                    value += 0x60
                    cpu_set_flags(cpu, u8(FLAG_CARRY), true)
                }

                if cpu_is_flag_set(cpu, u8(FLAG_HALF_CARRY)) || (value & 0xF) > 0x9 {
                    value += 0x6
                }
            }

            register_set_hi(&cpu.af, u8(value))

            cpu_set_flags(cpu, u8(FLAG_ZERO), register_get_hi(&cpu.af) == 0)
            cpu_set_flags(cpu, u8(FLAG_HALF_CARRY), false)
        case 0x28: // JR Z, *
            cpu_jump_add(cpu, cpu_is_flag_set(cpu, u8(FLAG_ZERO)))
        case 0x29: // ADD HL, HL
            cpu.hl.value = cpu_add_u16_u16(cpu, cpu.hl.value, cpu.hl.value)
        case 0x2A: // LD A, (HL+)
            register_set_hi(&cpu.af, memory_read_byte(&cpu.memory, cpu.hl.value))
            register_inc(&cpu.hl)
        case 0x2B: // DEC HL
            register_dec(&cpu.hl)
        case 0x2C: // INC L
            register_inc_low(&cpu.hl, cpu)
        case 0x2D: // DEC L
            register_dec_low(&cpu.hl, cpu)
        case 0x2E: // LD L, n
            register_set_low(&cpu.hl, memory_read_byte(&cpu.memory, cpu.pc))
            cpu.pc += 1
        case 0x2F: // CPL
            register_set_hi(&cpu.af, ~register_get_hi(&cpu.af))
        case 0x30: // JR NC, *
            cpu_jump_add(cpu, !cpu_is_flag_set(cpu, u8(FLAG_CARRY)))
        case 0x31:
            cpu.sp = memory_read_short(&cpu.memory, cpu.pc)
            cpu.pc += 2
        case 0x32:
            memory_write_byte(&cpu.memory, cpu.hl.value, register_get_hi(&cpu.af))
            register_dec(&cpu.hl)
        case 0x33: // INC SP
            cpu.sp += 1
        case 0x34: // INC (HL)
            tmp_val := memory_read_byte(&cpu.memory, cpu.hl.value)
            tmp_val = cpu_inc(cpu, tmp_val)
            memory_write_byte(&cpu.memory, cpu.hl.value, tmp_val)
        case 0x35: // DEC (HL)
            tmp_val := memory_read_byte(&cpu.memory, cpu.hl.value)
            tmp_val = cpu_dec(cpu, tmp_val)
            memory_write_byte(&cpu.memory, cpu.hl.value, tmp_val)
        case 0x36: // LD (HL), n
            memory_write_byte(&cpu.memory, cpu.hl.value, memory_read_byte(&cpu.memory, cpu.pc))
            cpu.pc += 1
        case 0x37: // SCF
            cpu_set_flags(cpu, u8(FLAG_CARRY), true)
            cpu_set_flags(cpu, u8(FLAG_SUBTRACT) | u8(FLAG_HALF_CARRY), false)
        case 0x38: // JR C, *
            cpu_jump_add(cpu, cpu_is_flag_set(cpu, u8(FLAG_CARRY)))
        case 0x39: // ADD HL, SP
            val := cpu_add_u16_u16(cpu, cpu.hl.value, cpu.sp)
            cpu.hl.value = val
        case 0x3A: // LD A, (HL-)
            register_set_hi(&cpu.af, memory_read_byte(&cpu.memory, cpu.hl.value))
            cpu.hl.value -= 1
        case 0x3B: // DEC SP
            cpu.sp -= 1
        case 0x3C: // INC A
            val := cpu_inc(cpu, register_get_hi(&cpu.af))
            register_set_hi(&cpu.af, val)
        case 0x3D: // DEC A
            val := cpu_dec(cpu, register_get_hi(&cpu.af))
            register_set_hi(&cpu.af, val)
        case 0x3E: // LD A, n
            register_set_hi(&cpu.af, memory_read_byte(&cpu.memory, cpu.pc))
            cpu.pc += 1
        case 0x3F: // CCF
            cpu_set_flags(cpu, u8(FLAG_CARRY), !cpu_is_flag_set(cpu, u8(FLAG_CARRY)))
            cpu_set_flags(cpu, u8(FLAG_SUBTRACT) | u8(FLAG_HALF_CARRY), false)
        case 0x40: // LD B, B
        case 0x41: // LD B, C
            register_set_hi(&cpu.bc, register_get_low(&cpu.bc))
        case 0x42: // LD B, D
            register_set_hi(&cpu.bc, register_get_hi(&cpu.de))
        case 0x43: // LD B, E
            register_set_hi(&cpu.bc, register_get_low(&cpu.de))
        case 0x44: // LD B, H
            register_set_hi(&cpu.bc, register_get_hi(&cpu.hl))
        case 0x45: // LD B, L
            register_set_hi(&cpu.bc, register_get_low(&cpu.hl))
        case 0x46: // LD B, (HL)
            register_set_hi(&cpu.bc, memory_read_byte(&cpu.memory, cpu.hl.value))
        case 0x47: // LD B, A
            register_set_hi(&cpu.bc, register_get_hi(&cpu.af))
        case 0x48: // LD C, B
            register_set_low(&cpu.bc, register_get_hi(&cpu.bc))
        case 0x49: // LD C, C
        case 0x4A: // LD C, D
            register_set_low(&cpu.bc, register_get_hi(&cpu.de))
        case 0x4B: // LD C, E
            register_set_low(&cpu.bc, register_get_low(&cpu.de))
        case 0x4C: // LD C, H
            register_set_low(&cpu.bc, register_get_hi(&cpu.hl))
        case 0x4D: // LD C, L
            register_set_low(&cpu.bc, register_get_low(&cpu.hl))
        case 0x4E: // LD C, (HL)
            register_set_low(&cpu.bc, memory_read_byte(&cpu.memory, cpu.hl.value))
        case 0x4F: // LD C, A
            register_set_low(&cpu.bc, register_get_hi(&cpu.af))
        case 0x50: // LD D, B
            register_set_hi(&cpu.de, register_get_hi(&cpu.bc))
        case 0x51: // LD D, C
            register_set_hi(&cpu.de, register_get_low(&cpu.bc))
        case 0x52: // LD D, D
        case 0x53: // LD D, E
            register_set_hi(&cpu.de, register_get_low(&cpu.de))
        case 0x54: // LD D, H
            register_set_hi(&cpu.de, register_get_hi(&cpu.hl))
        case 0x55: // LD D, L
            register_set_hi(&cpu.de, register_get_low(&cpu.hl))
        case 0x56: // LD D, (HL)
            register_set_hi(&cpu.de, memory_read_byte(&cpu.memory, cpu.hl.value))
        case 0x57: // LD D, A
            register_set_hi(&cpu.de, register_get_hi(&cpu.af))
        case 0x58: // LD E, B
            register_set_low(&cpu.de, register_get_hi(&cpu.bc))
        case 0x59: // LD E, C
            register_set_low(&cpu.de, register_get_low(&cpu.bc))
        case 0x5A: // LD E, D
            register_set_low(&cpu.de, register_get_hi(&cpu.de))
        case 0x5B: // LD E, E
        case 0x5C: // LD E, H
            register_set_low(&cpu.de, register_get_hi(&cpu.hl))
        case 0x5D: // LD E, L
            register_set_low(&cpu.de, register_get_low(&cpu.hl))
        case 0x5E: // LD E, (HL)
            register_set_low(&cpu.de, memory_read_byte(&cpu.memory, cpu.hl.value))
        case 0x5F: // LD E, A
            register_set_low(&cpu.de, register_get_hi(&cpu.af))
        case 0x60: // LD H, B
            register_set_hi(&cpu.hl, register_get_hi(&cpu.bc))
        case 0x61: // LD H, C
            register_set_hi(&cpu.hl, register_get_low(&cpu.bc))
        case 0x62: // LD H, D
            register_set_hi(&cpu.hl, register_get_hi(&cpu.de))
        case 0x63: // LD H, E
            register_set_hi(&cpu.hl, register_get_low(&cpu.de))
        case 0x64: // LD H, H
        case 0x65: // LD H, L
            register_set_hi(&cpu.hl, register_get_low(&cpu.hl))
        case 0x66: // LD H, (HL)
            register_set_hi(&cpu.hl, memory_read_byte(&cpu.memory, cpu.hl.value))
        case 0x67: // LD H, A
            register_set_hi(&cpu.hl, register_get_hi(&cpu.af))
        case 0x68: // LD L, B
            register_set_low(&cpu.hl, register_get_hi(&cpu.bc))
        case 0x69: // LD L, C
            register_set_low(&cpu.hl, register_get_low(&cpu.bc))
        case 0x6A: // LD L, D
            register_set_low(&cpu.hl, register_get_hi(&cpu.de))
        case 0x6B: // LD L, E
            register_set_low(&cpu.hl, register_get_low(&cpu.de))
        case 0x6C: // LD L, H
            register_set_low(&cpu.hl, register_get_hi(&cpu.hl))
        case 0x6D: // LD L, L
        case 0x6E: // LD L, (HL)
            register_set_low(&cpu.hl, memory_read_byte(&cpu.memory, cpu.hl.value))
        case 0x6F: // LD L, A
            register_set_low(&cpu.hl, register_get_hi(&cpu.af))
        case 0x70: // LD (HL), B
            memory_write_byte(&cpu.memory, cpu.hl.value, register_get_hi(&cpu.bc))
        case 0x71: // LD (HL), C
            memory_write_byte(&cpu.memory, cpu.hl.value, register_get_low(&cpu.bc))
        case 0x72: // LD (HL), D
            memory_write_byte(&cpu.memory, cpu.hl.value, register_get_hi(&cpu.de))
        case 0x73: // LD (HL), E
            memory_write_byte(&cpu.memory, cpu.hl.value, register_get_low(&cpu.de))
        case 0x74: // LD (HL), H
            memory_write_byte(&cpu.memory, cpu.hl.value, register_get_hi(&cpu.hl))
        case 0x75: // LD (HL), L
            memory_write_byte(&cpu.memory, cpu.hl.value, register_get_low(&cpu.hl))
        case 0x76: // TODO: HALT
            if !interrupt_is_master_enabled(&cpu.interrupts) && bool(memory_read_byte(&cpu.memory, 0xFF0F) & 0x1F) {
                cpu.memory.is_halted = false
                cpu.memory.trigger_halt_bug = true
            } else {
                cpu.memory.is_halted = true
            }
        case 0x77: // LD (HL), A
            memory_write_byte(&cpu.memory, cpu.hl.value, register_get_hi(&cpu.af))
        case 0xAF:
            cpu_xor_a(cpu, register_get_hi(&cpu.af))
        case 0xCB:
            cpu.pc += 1
            cpu_extended_execute(cpu, memory_read_byte(&cpu.memory, cpu.pc - 1))
        case:
            fmt.printfln("OpCode %#x not implemented", opcode)
            os2.exit(1)
    }
}

cpu_step :: proc(cpu: ^Cpu) {
    // TODO: Memory is halted then t_instr = 4 then return
    opcode := memory_read_byte(&cpu.memory, cpu.pc)
    new_pc, err := bits.overflowing_add(cpu.pc, 1)
    // cpu.pc += 1
    if err {
        fmt.println("ERROR: OVERFLOW in PROGRAM COUNTER")
        os2.exit(1)
    }
    cpu.pc = new_pc

    // TODO: trigger halt bug
    cpu_execute_instruction(cpu, opcode)
}