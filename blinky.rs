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
    pio::PIOExt,
    pac,
    sio::Sio,
    timer::Timer,
    watchdog::Watchdog,
};

use smart_leds::{brightness, SmartLedsWrite, RGB8};
use ws2812_pio::Ws2812;

use cichlid::{HSV, ColorRGB, GradientDirection, prelude::*};

const STRIP_LEN: usize = 50;
const CHUNKLEN: usize = 1024;

fn fade(rgb: &mut RGB8) {
    if rgb.r > 0 {
        rgb.r = rgb.r - 1;
        rgb.r = rgb.r - rgb.r / 8;
    }
    if rgb.g > 0 {
        rgb.g = rgb.g - 1;
        rgb.g = rgb.g - rgb.g / 8;
    }
    if rgb.b > 0 {
        rgb.b = rgb.b - 1;
        rgb.b = rgb.b - rgb.b / 8;
    }
}

#[entry]
fn main() -> ! {
    info!("Program start");

    let mut palette_tmp = [ColorRGB::Black; 256];

    let start_hue: u8 = 0;
    let hue_delta: u16 = (1 << 8);

    palette_tmp.rainbow_fill(start_hue, hue_delta); // From step size
    palette_tmp.rainbow_fill_single_cycle(start_hue); // Complete rainbow
                                                  //
    let mut palette: [RGB8; 256] = [(0, 0, 0).into(); 256];
    for i in 0..256 {
        palette[i].r = palette_tmp[i].r;
        palette[i].g = palette_tmp[i].g;
        palette[i].b = palette_tmp[i].b;
    }


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


    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Setup ws2812 leds
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let mut ws = Ws2812::new(
        pins.data.into_function(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    let dma = pac.DMA.split(&mut pac.RESETS);
    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);
    let mut adc_pin_0 = AdcPin::new(pins.adc0.into_floating_input()).unwrap();
    // adc.free_running(&adc_pin_0);

    let buf_for_samples = singleton!(: [u16; CHUNKLEN] = [0; CHUNKLEN]).unwrap();
    let buf_for_fft = singleton!(: [f32; CHUNKLEN] = [0.; CHUNKLEN]).unwrap();
    let buf_for_fft_normed = singleton!(: [f32; CHUNKLEN / 2] = [0.0; CHUNKLEN / 2]).unwrap();
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

    let mut leds: [RGB8; STRIP_LEN] = [(0, 0, 0).into(); STRIP_LEN];
    let strip_brightness = 255u8;

    let mut colour: usize = 0;
    loop {
        colour = (colour + 5) % 256;
        // Should really do the transfer triggered by an interrupt
        let (ch, adc_read_target, buf_for_samples) = dma_transfer.wait();
        let mut min: u16 = 65535;
        let mut max: u16 = 0;
        let mut total: u32 = 0;
        for i in 0..CHUNKLEN {
            if buf_for_samples[i] < min { min = buf_for_samples[i] }
            if buf_for_samples[i] > max { max = buf_for_samples[i] }
            total += u32::from(buf_for_samples[i]);
            // defmt::println!("D {}", buf_for_samples[i]);
        }
        let avg: f32 = total as f32 / CHUNKLEN as f32;
        defmt::println!("min:{} max:{} avg:{} range:{}", min, max, avg, max - min);

        for i in 0..CHUNKLEN {
            buf_for_fft[i] = (buf_for_samples[i] as f32 - avg) / 4096.0;
        }
        let spectrum = microfft::real::rfft_1024(buf_for_fft);
        for i in 0..CHUNKLEN / 2 {
            buf_for_fft_normed[i] = spectrum[i].norm_sqr();
        }
        let mut max_i: usize = 0;
        let mut max_val: f32 = 0.;
        for i in 0..CHUNKLEN / 4 {
            if buf_for_fft[i] > max_val {
                max_val = buf_for_fft[i];
                max_i = i;
            }
        }

        //for i in 0..40 {
        //    defmt::println!("F {} {}", i, buf_for_fft[i]);
        //}
        // defmt::println!("");

        // Start next transfer
        dma_transfer = single_buffer::Config::new(
            ch, adc_read_target, buf_for_samples
        ).start();

        let mut increased: i32 = 0;
        colour = (max_i * 2) % 256;
        for i in 0..STRIP_LEN {
            let signal = buf_for_fft[i] / max_val;
            if max_val > 0.5 && signal > 0.5 {
                let r = (palette[colour].r as f32 * signal) as u8;
                let g = (palette[colour].g as f32 * signal) as u8;
                let b = (palette[colour].b as f32 * signal) as u8;
                if leds[i].r < r { leds[i].r = r }
                if leds[i].g < g { leds[i].g = g }
                if leds[i].b < b { leds[i].b = b }
                increased += 1;
            } else {
                fade(&mut leds[i]);
            }
        }
        defmt::println!("max {:03} {:03} - increased {}", max_i, (max_val * 1000.0) as u16, increased);

        ws.write(brightness(leds.iter().copied(), strip_brightness)).unwrap();
    }
}

// End of file
