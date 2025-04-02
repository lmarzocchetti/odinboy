package src

Status :: struct {
    debug: bool,
    is_running: bool,
    is_paused: bool,
    do_step: bool
}

Gb :: struct {
    cpu: Cpu,
    status: Status
}

gb_create :: proc() -> Gb {
    return Gb {
        cpu = cpu_init(),
        status = Status {
            is_running = true
        }
    }
}

// TODO: Renderer check add bool when ppu can render
gb_run_step :: proc(gb: ^Gb) {
    if !gb.status.is_paused || gb.status.do_step {
        gb.cpu.clock.t_instr = 0
        // TODO: check for interrupts
        
        cpu_step(&gb.cpu)

        // TODO: ppu step and timer inc
    }

    gb.status.do_step = false

    // TODO: joypad check

    // TODO: Renderer check
}

gb_run :: proc(gb: ^Gb) {
    for gb.status.is_running {
        gb_run_step(gb)
    }
}