use embassy_stm32::usb_otg::{Driver, self};
use embassy_usb::UsbDevice;
use embassy_stm32::{bind_interrupts, peripherals};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::{Builder, Config};
use core::mem::MaybeUninit;

bind_interrupts!(pub struct Irqs {
    OTG_FS => usb_otg::InterruptHandler<peripherals::USB_OTG_FS>;
});

// Type definitions for easier usage
pub type UsbDriver = Driver<'static, peripherals::USB_OTG_FS>;
pub type UsbSerial<'a> = CdcAcmClass<'a, UsbDriver>;

// Static buffers to keep then alive during the program execution
// We use StaticCell to avoid `static mut` and unsafe where possible for the structure,
// but for the raw buffers passed to the Builder, static mut is still the standard way in embedded
// unless we pass `&'static mut [u8]` from main using `make_static!`.
// For simplicity here we will use `static mut` for the raw buffers like in the original code but encapsulated.

pub struct UsbResources<'a> {
    config_desc: [u8; 256],
    bos_desc: [u8; 256],
    control_buf: [u8; 64],
    state: MaybeUninit<State<'a>>,
    // buffer for the driver
    ep_out_buffer: [u8; 256],
}

impl<'a> UsbResources<'a> {
    pub const fn new() -> Self {
        Self {
            config_desc: [0; 256],
            bos_desc: [0; 256],
            control_buf: [0; 64],
            state: MaybeUninit::uninit(),
            ep_out_buffer: [0; 256],
        }
    }
}

// Global static storage
static mut USB_RES: UsbResources<'static> = UsbResources::new();

#[embassy_executor::task]
pub async fn usb_task(mut device: UsbDevice<'static, UsbDriver>) -> ! {
    device.run().await
}

pub fn init(
    usb_periph: peripherals::USB_OTG_FS,
    pa12: peripherals::PA12,
    pa11: peripherals::PA11,
) -> (UsbDevice<'static, UsbDriver>, UsbSerial<'static>) {
    
    // Create the driver
    // We access the static buffer unsafely. Since this init is called once, it is safe.
    // using &raw mut to avoid creating a reference that could alias if not careful (though here it is unique)
    let driver_buf = unsafe { &mut *(&raw mut USB_RES.ep_out_buffer) };
    let mut usb_config = embassy_stm32::usb_otg::Config::default();
    usb_config.vbus_detection = false;
    let driver = Driver::new_fs(usb_periph, Irqs, pa12, pa11, driver_buf, usb_config);

    let mut config = Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("JHEF Rust");
    config.product = Some("JHEF405 Pro Controller");
    config.serial_number = Some("12345678");

    // Builder
    // We access static buffers unsafely
    let builder = unsafe {
        let res = &mut *(&raw mut USB_RES);
        Builder::new(
            driver,
            config,
            &mut res.config_desc,
            &mut res.bos_desc,
            &mut [], // msos_descs
            &mut res.control_buf,
        )
    };
    
    let res = unsafe { &mut *(&raw mut USB_RES) };
    // Init state
    let state = res.state.write(State::new());
    
    let mut builder = builder;
    let class = CdcAcmClass::new(&mut builder, state, 64);
    let usb = builder.build();

    (usb, class)
}
