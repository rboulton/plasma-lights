//! Blinks the 3 colour LEDs on a Pimoroni Plasma 2040 in sequence
#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::OutputPin;
use panic_probe as _;

use pimoroni_plasma_2040 as bsp;
use cortex_m::singleton;

use bsp::hal::{
    adc::Adc,
    adc::AdcPin,
    dma::single_buffer,
    dma::DMAExt,
    clocks::{init_clocks_and_plls, Clock},
    gpio::PinState,
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let clocks = init_clocks_and_plls(
        bsp::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let dma = pac.DMA.split(&mut pac.RESETS);
    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);
    let mut adc_pin_0 = AdcPin::new(pins.adc0.into_floating_input()).unwrap();
    // adc.free_running(&adc_pin_0);

    const len: usize = 1024;
    let buf_for_samples = singleton!(: [u16; len] = [0; len]).unwrap();
    let mut adc_fifo = adc.build_fifo()
        .clock_divider(999, 0) // (48MHz / 48ksps) - 1 = 999 
        .set_channel(&mut adc_pin_0)
        .enable_dma()
        .start_paused();

    let mut dma_transfer = single_buffer::Config::new(dma.ch0, adc_fifo.dma_read_target(), buf_for_samples).start();
    adc_fifo.resume();

    //let mut led_green = pins.led_green.into_push_pull_output_in_state(PinState::High);
    //let mut led_red = pins.led_red.into_push_pull_output_in_state(PinState::High);
    //let mut led_blue = pins.led_blue.into_push_pull_output_in_state(PinState::High);

    loop {
        let (ch, adc_read_target, buf_for_samples) = dma_transfer.wait();
        let mut min: u16 = 65535;
        let mut max: u16 = 0;
        let mut total: u32 = 0;
        for i in 0..len {
            if buf_for_samples[i] < min { min = buf_for_samples[i] }
            if buf_for_samples[i] > max { max = buf_for_samples[i] }
            total += u32::from(buf_for_samples[i]);
            // defmt::println!("D {}", buf_for_samples[i]);
        }
        dma_transfer = single_buffer::Config::new(
            ch, adc_read_target, buf_for_samples
        ).start();
        defmt::println!("min:{} max:{} avg:{} range:{}", min, max, total / 1024, max - min);

        // let pin_adc_counts: u16 = adc.read(&mut adc_pin_0).unwrap();
        //led_green.set_low().unwrap();
        //delay.delay_ms(500);
        //led_green.set_high().unwrap();
        //delay.delay_ms(500);
        //led_blue.set_low().unwrap();
        //delay.delay_ms(500);
        //led_blue.set_high().unwrap();
        //led_red.set_low().unwrap();
        //delay.delay_ms(500);
        //led_red.set_high().unwrap();
    }
}

// End of file
