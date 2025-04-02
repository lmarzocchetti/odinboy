package src

InterruptFlags :: enum u8 {
    INTERRUPT_VBLANK = (1 << 0),
    INTERRUPT_LCD = (1 << 1),
    INTERRUPT_TIMER = (1 << 2),
    INTERRUPT_SERIAL = (1 << 3),
    INTERRUPT_JOYPAD = (1 << 4)
}

Interrupt :: struct {
    IME: bool
}

// TODO: Verify this function
interrupt_set_master_flag :: proc(interrupt: ^Interrupt, state: bool) {
    interrupt.IME = bool(u32(state) << 0)
}

// TODO: Verify this function
interrupt_is_master_enabled :: proc(interrupt: ^Interrupt) -> bool {
    return bool(u32(interrupt.IME) & 1)
}

interrupt_set_interrupt_flag :: proc(interrupt: ^Interrupt, memory: ^Memory, flag: u8) {
    IF_value := memory_read_byte(memory, 0xFF0F)
    IF_value |= flag
    memory_write_byte(memory, 0xFF0F, IF_value)
}

interrupt_unset_interrupt_flag :: proc(interrupt: ^Interrupt, memory: ^Memory, flag: u8) {
    IF_value := memory_read_byte(memory, 0xFF0F)
    IF_value &= ~flag
    memory_write_byte(memory, 0xFF0F, IF_value)
}

interrupt_is_interrupt_enabled :: proc(interrupt: ^Interrupt, memory: ^Memory, flag: u8) -> bool {
    return bool(memory_read_byte(memory, 0xFFFF) & flag)
}

interrupt_is_interrupt_flag_set :: proc(interrupt: ^Interrupt, memory: ^Memory, flag: u8) -> bool {
    return bool(memory_read_byte(memory, 0xFF0F) & flag)
}

interrupt_check :: proc(interrupt: ^Interrupt, cpu: ^Cpu) -> bool {
    using InterruptFlags
    if bool(memory_read_byte(&cpu.memory, 0xFFFF)) && bool(memory_read_byte(&cpu.memory, 0xFF0F) & 0x0F) {
        cpu.memory.is_halted = false
    }

    if interrupt_is_master_enabled(interrupt) {
        return false
    }

    if interrupt_is_interrupt_enabled(interrupt, &cpu.memory, u8(INTERRUPT_VBLANK)) && interrupt_is_interrupt_flag_set(interrupt, &cpu.memory, u8(INTERRUPT_VBLANK)) {
        interrupt_trigger_interrupt(interrupt, cpu, INTERRUPT_VBLANK, 0x40)
        return true
    }

    if interrupt_is_interrupt_enabled(interrupt, &cpu.memory, u8(INTERRUPT_LCD)) && interrupt_is_interrupt_flag_set(interrupt, &cpu.memory, u8(INTERRUPT_LCD)) {
        interrupt_trigger_interrupt(interrupt, cpu, INTERRUPT_LCD, 0x48)
        return true
    }

    if interrupt_is_interrupt_enabled(interrupt, &cpu.memory, u8(INTERRUPT_TIMER)) && interrupt_is_interrupt_flag_set(interrupt, &cpu.memory, u8(INTERRUPT_TIMER)) {
        interrupt_trigger_interrupt(interrupt, cpu, INTERRUPT_TIMER, 0x50)
        return true
    }

    if interrupt_is_interrupt_enabled(interrupt, &cpu.memory, u8(INTERRUPT_JOYPAD)) && interrupt_is_interrupt_flag_set(interrupt, &cpu.memory, u8(INTERRUPT_JOYPAD)) {
        interrupt_trigger_interrupt(interrupt, cpu, INTERRUPT_JOYPAD, 0x60)
        return true
    }

    return false
}

interrupt_trigger_interrupt :: proc(interrupt: ^Interrupt, cpu: ^Cpu, interrupt_flag: InterruptFlags, jump_pc: u8) {
    memory_write_short_stack(&cpu.memory, &cpu.sp, cpu.pc)
    cpu.pc = u16(jump_pc)
    interrupt_set_master_flag(interrupt, false)
    interrupt_unset_interrupt_flag(interrupt, &cpu.memory, u8(interrupt_flag))
    cpu.memory.is_halted = false

    cpu.clock.t_instr = 20
}