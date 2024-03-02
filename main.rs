#![no_std]  //not using the standard 
#![no_main] //not using typical function enter

/**** low-level imports *****/
 use panic_halt as _;
use cortex_m_rt::entry;
use embedded_hal::{
    digital::v2::{OutputPin},
};

/***** board-specific imports *****/
use adafruit_feather_rp2040::hal as hal;
use hal::{
    i2c::I2C,
    gpio::{FunctionI2C,Pin,PullUp},
    //pac::interrupt,
    //fugit::RateExtU32,
    clocks::{init_clocks_and_plls, Clock},
    pac,
    watchdog::Watchdog,
    pio::PIOExt,
    timer::Timer,
    Sio,
};
use adafruit_feather_rp2040::{
    Pins, XOSC_CRYSTAL_FREQ,
};


use embedded_hal::blocking::i2c::{Write,WriteRead};
use fugit::RateExtU32;
use ws2812_pio::Ws2812;
use smart_leds::{RGB8, SmartLedsWrite};
mod nodes;


#[entry]
fn main() -> ! {
    // Grab the singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    // Init the watchdog timer, to pass into the clock init
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let clocks = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    ).ok().unwrap();
    
    let sio = Sio::new(pac.SIO);
    // initialize the pins to default state
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut timer = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let mut led_pin = pins.d13.into_push_pull_output();

    //enable the prop-maker pin
    let mut pwr_pin = pins.d10.into_push_pull_output();
    pwr_pin.set_high().unwrap();

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let mut neopixels = Ws2812::new(
        pins.d5.into_mode(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    //let sda_pin: Pin<_, FunctionI2C> = pins.sda;
    //let scl_pin: Pin<_, FunctionI2C> = pins.scl;
    let sda_pin = pins.sda.into_mode::<FunctionI2C>();
    let scl_pin = pins.scl.into_mode::<FunctionI2C>();




    let mut i2c = I2C::i2c1(
        pac.I2C1,
        sda_pin,
        scl_pin,
        //pins.sda.into_function(),
        //pins.scl.into_function(),
        //sda_pin.into_function(),//pins.sda.into_function(), //SDA pin is 2 SCl is 3
        //scl_pin.into_function(),//pins.scl.into_function(),
        100.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );


    /*let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let mut ws = Ws2812::new(
        pins.d5.into_mode(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );*/

    /*
    Loop Section
    */
    const Y_L : u8 = 0x2A;
    const X_L : u8 = 0x28;
    const WHO_AM_I : u8 = 0b0001111;
    let delay: u32 = 500;   // loop delay in ms
    i2c.write(0x18u8, &[0x20,0b10010011]).unwrap();
    i2c.write(0x18u8, &[0x23,0x80]).unwrap();
    i2c.write(0x18u8, &[0xC0,0xC0]).unwrap();
    let mut data: [u8; 1] = [0; 1]; //recieved data
    i2c.write_read(0x18u8,&[WHO_AM_I],&mut data).unwrap(); //this very likely works
    //let first_class = nodes::nodes::new(false,"up",true,false);
    let mut led_array : [nodes::nodes;64] = [nodes::nodes::new(false,"no",false,false,false,0,00,0,0); 64];
    let mut colors : [RGB8;64] = [(0,0,50).into();64];

    //let color : RGB8 = (255, 0, 255).into();
    //neopixels.write([color,color].iter().copied()).unwrap();

    //let colors : [RGB8;64] = [(0,0,50).into();64];
        //for i in 0..64 {
            //let color : RGB8 = (elem.red,elem.green,elem.blue).into();
    //neopixels.write(colors.iter().copied()).unwrap();

    
    initialize_game(&mut led_array);
    let mut data_xl: [u8; 1] = [0; 1]; //recieved data
    let mut data_xh: [u8; 1] = [0; 1]; //recieved data
    let mut data_yl: [u8; 1] = [0; 1]; //recieved data
    let mut data_yh: [u8; 1] = [0; 1]; //recieved data



    if(data[0] == 0b00110011)
    {
    loop 
    {
        //contantly want to read what the accel's status is
        i2c.write_read(0x18u8,&[X_L],&mut data_xl).unwrap(); //this very likely works
        i2c.write_read(0x18u8,&[X_L+1],&mut data_xh).unwrap(); //this very likely works
        i2c.write_read(0x18u8,&[Y_L],&mut data_yl).unwrap(); //this very likely works
        i2c.write_read(0x18u8,&[Y_L+1],&mut data_yh).unwrap(); //this very likely works
        let scaling : f32 = (64 as f32)/(0.004 as f32);
        let mut x: f32 = ((((data_xh[0] as i16)<<8) | data_xl[0] as i16)as f32)/(scaling as f32);
        let mut y: f32 = ((((data_yh[0] as i16)<<8) | data_yl[0] as i16)as f32)/(scaling as f32);
        //constantly want to read what the accel's status is

        led_pin.set_low().unwrap();
        for _ in 0..100000 {
            // Do nothing, just iterate
        }
        //timer.delay_ms(delay as u32);
        led_pin.set_high().unwrap();
        //timer.delay_ms(delay as u32);

        //displaying the LED array -just don't touch this code honestly
        for _ in 0..100000 {
            // Do nothing, neomatrix NEEDS this - also compiler leaves it for some reason.
        }
        neopixels.write(colors.iter().copied()).unwrap();
        update_nodes(&mut led_array);
        update_colors(&mut led_array,&mut colors);
        //end displaying
    }
}
    else
    {
        loop
        {

        }
    }
}

fn update_colors(led_array: &mut [nodes::nodes<'_>; 64], colors : &mut [RGB8;64] ) {
    for i in 0..63{
        let color : RGB8 = (led_array[i].red,led_array[i].green,led_array[i].blue).into();
        colors[i] = color;
    }
}

fn update_nodes(led_array: &mut [nodes::nodes<'_>; 64])
{
    for i in 0..63{
        if(led_array[i].occupied == true) //if a snake is on the square prioritize it.
        {
            led_array[i].green  = 50;
            led_array[i].red    = 0;
            led_array[i].blue   = 0;
        }
        else if(led_array[i].food == true)
        {
            led_array[i].green  = 0;
            led_array[i].red    = 50;
            led_array[i].blue   = 0;
        }
    }
}

fn initialize_game(led_array: &mut [nodes::nodes<'_>; 64]){
    led_array[20].head      = true;
    led_array[20].occupied  = true;

    led_array[19].occupied  = true;
    led_array[19].tail      = true;
    led_array[19].direction = "E";

    led_array[44].food      = true;
}

fn get_head_direction(led_array: &mut [nodes::nodes<'_>; 64], x_values: &f32, y_values: &f32)
{
    let mut head_index = 0;
    for i in 0..63{
        if(led_array[i].head == true)
        {
            head_index = i; //need to determine this to set everything - storing where the head is
        }
    }



}


