#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
// use defmt_serial as _;
use defmt_rtt as _;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_0_2::{
    adc::OneShot,
};
use panic_probe as _;

use rp_pico as bsp;
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

const STRIP_MAIN_LEN: usize = 83;
const STRIP_SIDE_LEN: usize = 20;
const STRIP_CUPBOARD_1_LEN: usize = 18;
const STRIP_CUPBOARD_2_LEN: usize = 18;
const STRIP_CUPBOARD_3_LEN: usize = 18;
const STRIP_CUPBOARD_4_LEN: usize = 18;

const STRIP_LEN: usize = STRIP_MAIN_LEN + STRIP_SIDE_LEN + STRIP_CUPBOARD_1_LEN + STRIP_CUPBOARD_2_LEN + STRIP_CUPBOARD_3_LEN + STRIP_CUPBOARD_4_LEN;
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
    let hue_delta: u16 = 1 << 8;

    palette_tmp.rainbow_fill(start_hue, hue_delta); // From step size
    // palette_tmp.rainbow_fill_single_cycle(start_hue); // Complete rainbow
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

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);

    // Control buttons
    let mut button1_pin = pins.gpio10.into_pull_up_input();
    let mut button2_pin = pins.gpio11.into_pull_up_input();
    let mut button3_pin = pins.gpio12.into_pull_up_input();
    let mut button4_pin = pins.gpio13.into_pull_up_input();
    let mut button5_pin = pins.gpio14.into_pull_up_input();
    let mut button6_pin = pins.gpio15.into_pull_up_input();
    let mut dial1_pin = AdcPin::new(pins.gpio27.into_floating_input()).unwrap();
    let mut dial2_pin = AdcPin::new(pins.gpio28.into_floating_input()).unwrap();
    adc.free_running(&dial1_pin);
    adc.free_running(&dial2_pin);

    let mut button1_state: bool = false;
    let mut button2_state: bool = false;
    let mut button3_state: bool = false;
    let mut button4_state: bool = false;
    let mut button5_state: bool = false;
    let mut button6_state: bool = false;
    let mut dial1_value: u16 = 0;
    let mut dial2_value: u16 = 0;

    // Setup ws2812 leds
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let mut ws = Ws2812::new(
        pins.gpio22.into_function(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    let dma = pac.DMA.split(&mut pac.RESETS);
    let mut adc_pin_0 = AdcPin::new(pins.gpio26.into_floating_input()).unwrap();
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
    let mut strip_brightness_average = 255u16;
    let mut strip_brightness = 255u8;

    let mut led_pin = pins.led.into_push_pull_output();
    let mut counter: u32 = 0;

    let mut mode: u32 = 0;

    let mut button1_last_state = false;
    let mut button2_last_state = false;
    let mut button3_last_state = false;
    let mut button4_last_state = false;
    let mut button5_last_state = false;
    let mut button6_last_state = false;

    loop {
        if button1_pin.is_low().unwrap() {
            button1_state = true;
        } else {
            button1_state = false;
        }

        if button2_pin.is_low().unwrap() {
            button2_state = true;
        } else {
            button2_state = false;
        }

        if button3_pin.is_low().unwrap() {
            button3_state = true;
        } else {
            button3_state = false;
        }

        if button4_pin.is_low().unwrap() {
            button4_state = true;
        } else {
            button4_state = false;
        }

        if button5_pin.is_low().unwrap() {
            button5_state = true;
        } else {
            button5_state = false;
        }

        if button6_pin.is_low().unwrap() {
            button6_state = true;
        } else {
            button6_state = false;
        }

        dial1_value = adc.read(&mut dial1_pin).unwrap();
        dial2_value = adc.read(&mut dial2_pin).unwrap();

        if button1_state && !button1_last_state {
            mode += 1;
            if mode >= 5 {
                mode = 0
            }
        }

        if button2_state && !button2_last_state {
            if mode == 0 {
                mode = 4
            } else {
                mode -= 1;
            }
        }

        if button1_state || button2_state || button3_state ||
           button4_state || button5_state || button6_state {
            led_pin.set_high().unwrap();
        } else {
            led_pin.set_low().unwrap();
        }

        button1_last_state = button1_state;
        button2_last_state = button2_state;
        button3_last_state = button3_state;
        button4_last_state = button4_state;
        button5_last_state = button5_state;
        button6_last_state = button6_state;

        let change_speed = u8::try_from(dial1_value / 512).unwrap();
        counter += u32::from(change_speed);
        strip_brightness_average = (strip_brightness_average * 7 + u16::try_from(dial2_value / 16).unwrap()) / 8;
        strip_brightness = u8::try_from(strip_brightness_average).unwrap();
        defmt::println!("D {} {}", change_speed, strip_brightness);

        if mode != 4 {
            delay.delay_ms(20);
            for i in 0..STRIP_LEN {
                fade(&mut leds[i]);
            }
        }

        if mode == 0 {
            ws.write(brightness(leds.iter().copied(), strip_brightness)).unwrap();
        }


        if mode == 1 {
            let mut colour : usize = usize::try_from(counter % 256).unwrap();
            for i in 0..STRIP_MAIN_LEN {
                let r = 255;
                let g = 255;
                let b = 255;
                if leds[i].r < r { leds[i].r = r }
                if leds[i].g < g { leds[i].g = g }
                if leds[i].b < b { leds[i].b = b }
            }
            ws.write(brightness(leds.iter().copied(), strip_brightness)).unwrap();
        }

        if mode == 2 {
            let mut colour : usize = usize::try_from(counter % 256).unwrap();
            colour = 95;
            for i in STRIP_MAIN_LEN..STRIP_MAIN_LEN + STRIP_SIDE_LEN {
                let r = (palette[colour].r as f32) as u8;
                let g = (palette[colour].g as f32) as u8;
                let b = (palette[colour].b as f32) as u8;
                if leds[i].r < r { leds[i].r = r }
                if leds[i].g < g { leds[i].g = g }
                if leds[i].b < b { leds[i].b = b }
            }
            ws.write(brightness(leds.iter().copied(), strip_brightness)).unwrap();
        }

        if mode == 3 {
            let mut colour : usize = usize::try_from(counter % 256).unwrap();
            colour = 150;
            for i in STRIP_MAIN_LEN + STRIP_SIDE_LEN .. STRIP_MAIN_LEN + STRIP_SIDE_LEN + STRIP_CUPBOARD_1_LEN {
                let r = (palette[colour].r as f32) as u8;
                let g = (palette[colour].g as f32) as u8;
                let b = (palette[colour].b as f32) as u8;
                if leds[i].r < r { leds[i].r = r }
                if leds[i].g < g { leds[i].g = g }
                if leds[i].b < b { leds[i].b = b }
            }

            colour = 220;
            for i in STRIP_MAIN_LEN + STRIP_SIDE_LEN +
                     STRIP_CUPBOARD_1_LEN
            ..
                     STRIP_MAIN_LEN + STRIP_SIDE_LEN +
                     STRIP_CUPBOARD_1_LEN +
                     STRIP_CUPBOARD_2_LEN
            {
                let r = (palette[colour].r as f32) as u8;
                let g = (palette[colour].g as f32) as u8;
                let b = (palette[colour].b as f32) as u8;
                if leds[i].r < r { leds[i].r = r }
                if leds[i].g < g { leds[i].g = g }
                if leds[i].b < b { leds[i].b = b }
            }
 
            colour = 80;
            for i in STRIP_MAIN_LEN + STRIP_SIDE_LEN +
                     STRIP_CUPBOARD_1_LEN +
                     STRIP_CUPBOARD_2_LEN
            ..
                     STRIP_MAIN_LEN + STRIP_SIDE_LEN +
                     STRIP_CUPBOARD_1_LEN +
                     STRIP_CUPBOARD_2_LEN +
                     STRIP_CUPBOARD_3_LEN
            {
                let r = (palette[colour].r as f32) as u8;
                let g = (palette[colour].g as f32) as u8;
                let b = (palette[colour].b as f32) as u8;
                if leds[i].r < r { leds[i].r = r }
                if leds[i].g < g { leds[i].g = g }
                if leds[i].b < b { leds[i].b = b }
            }

            colour = 190;
            for i in STRIP_MAIN_LEN + STRIP_SIDE_LEN +
                     STRIP_CUPBOARD_1_LEN +
                     STRIP_CUPBOARD_2_LEN +
                     STRIP_CUPBOARD_3_LEN
            ..
                     STRIP_MAIN_LEN + STRIP_SIDE_LEN +
                     STRIP_CUPBOARD_1_LEN +
                     STRIP_CUPBOARD_2_LEN +
                     STRIP_CUPBOARD_3_LEN +
                     STRIP_CUPBOARD_4_LEN
            {
                let r = (palette[colour].r as f32) as u8;
                let g = (palette[colour].g as f32) as u8;
                let b = (palette[colour].b as f32) as u8;
                if leds[i].r < r { leds[i].r = r }
                if leds[i].g < g { leds[i].g = g }
                if leds[i].b < b { leds[i].b = b }
            }


            ws.write(brightness(leds.iter().copied(), strip_brightness)).unwrap();
        }

        if mode == 4 {
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
            // defmt::println!("min:{} max:{} avg:{} range:{}", min, max, avg, max - min);

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

            // Start next transfer
            dma_transfer = single_buffer::Config::new(
                ch, adc_read_target, buf_for_samples
            ).start();

            let mut increased: i32 = 0;
            let mut colour: usize = (max_i * 2) % 256;
            for i in 0..STRIP_MAIN_LEN {
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
            for i in STRIP_MAIN_LEN..STRIP_LEN {
                fade(&mut leds[i]);
            }
            // defmt::println!("max {:03} {:03} - increased {}", max_i, (max_val * 1000.0) as u16, increased);

            ws.write(brightness(leds.iter().copied(), strip_brightness)).unwrap();
        }

        // defmt_serial::defmt_serial(uart);

    }
}

// End of file
