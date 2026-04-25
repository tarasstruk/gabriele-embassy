use embassy_rp::peripherals::PIO1;
use embassy_rp::pio::program::pio_asm;
use embassy_rp::pio::{Common, Config as PioConf, ShiftConfig, ShiftDirection, StateMachine};
use fixed::types::extra::U8;

pub fn setup_pio_task_sm0<'d>(pio: &mut Common<'d, PIO1>, sm: &mut StateMachine<'d, PIO1, 0>) {
    let prg = pio_asm!(
        ".wrap_target"
        "  wait 0 pin 4"
        "  wait 1 pin 4"
        "  wait 0 pin 3"
        "  wait 1 pin 3"
        "  wait 0 pin 3"
        "  in null, 1",
        ".wrap"
    );
    let mut cfg = PioConf::default();
    cfg.use_program(&pio.load_program(&prg.program), &[]);

    cfg.shift_in = ShiftConfig {
        auto_fill: true,
        direction: ShiftDirection::Left,
        threshold: 1,
    };

    cfg.clock_divider = fixed::FixedU32::<U8>::from_num(10);
    sm.set_config(&cfg);
    sm.set_enable(true);
}
