
// TODO: Haven't been able to get this to work yet.

/*const DMA_CH0_CTRL_TRIG_BUSY_BITS: u32 = 0x01000000;

bitflags! {
    struct DMAChannelSize: u32 {
        const DMA_SIZE_8 = 0;
        const DMA_SIZE_16 = 1;
        const DMA_SIZE_32 = 2;
    }
}

const DMA_CH0_CTRL_TRIG_INCR_READ_BITS: u32 = 0x00000010;
const DMA_CH0_CTRL_TRIG_INCR_WRITE_BITS: u32 = 0x00000020;
const DMA_CH0_CTRL_TRIG_TREQ_SEL_BITS: u32 = 0x001f8000;
const DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB: u32 = 15;
const DMA_CH0_CTRL_TRIG_CHAIN_TO_BITS: u32 = 0x00007800;
const DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB: u32 = 14;
const DMA_CH0_CTRL_TRIG_DATA_SIZE_BITS: u32 = 0x0000000c;
const DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB: u32 = 1;
const DMA_CH0_CTRL_TRIG_RING_SIZE_BITS: u32 = 0x000003c0;
const DMA_CH0_CTRL_TRIG_RING_SEL_BITS: u32 = 0x00000400;
const DMA_CH0_CTRL_TRIG_RING_SIZE_LSB: u32 = 6;
const DMA_CH0_CTRL_TRIG_BSWAP_BITS: u32 = 0x00400000;
const DMA_CH0_CTRL_TRIG_IRQ_QUIET_BITS: u32 = 0x00200000;
const DMA_CH0_CTRL_TRIG_EN_BITS: u32 = 0x00000001;
const DMA_CH0_CTRL_TRIG_SNIFF_EN_BITS: u32 = 0x00800000;

const DREQ_FORCE: u32 = 63;

struct DMAChannelConfig {
    ctrl: u32
}

impl DMAChannelConfig {
    fn set_read_increment(&mut self, incr : bool) {
        self.ctrl = if incr {
            self.ctrl | DMA_CH0_CTRL_TRIG_INCR_READ_BITS
        } else {
            self.ctrl & DMA_CH0_CTRL_TRIG_INCR_READ_BITS.not()
        };
    }

    fn set_write_increment(&mut self, incr : bool) {
        self.ctrl = if incr {
            self.ctrl | DMA_CH0_CTRL_TRIG_INCR_WRITE_BITS
        } else {
            self.ctrl & DMA_CH0_CTRL_TRIG_INCR_WRITE_BITS.not()
        };
    }

    fn set_dreq(&mut self, dreq : u32) {
        self.ctrl = (self.ctrl & DMA_CH0_CTRL_TRIG_TREQ_SEL_BITS.not())
            | (dreq << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB);
    }

    fn set_chain_to(&mut self, chain_to : u32) {
        self.ctrl = (self.ctrl & DMA_CH0_CTRL_TRIG_CHAIN_TO_BITS.not())
            | (chain_to << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB);
    }

    fn set_transfer_data_size(&mut self, size : DMAChannelSize) {
        self.ctrl = (self.ctrl & DMA_CH0_CTRL_TRIG_DATA_SIZE_BITS.not())
            | (size.bits << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB);
    }

    fn set_ring(&mut self, write: bool, size_bits: u32) {
        assert!(size_bits < 32);
        self.ctrl = (self.ctrl & (DMA_CH0_CTRL_TRIG_RING_SIZE_BITS | DMA_CH0_CTRL_TRIG_RING_SEL_BITS).not())
            | (size_bits << DMA_CH0_CTRL_TRIG_RING_SIZE_LSB)
            | if write {DMA_CH0_CTRL_TRIG_RING_SEL_BITS} else {0};
    }

    fn set_bswap(&mut self, bswap: bool) {
        self.ctrl = if bswap {
            self.ctrl | DMA_CH0_CTRL_TRIG_BSWAP_BITS
        } else {
            self.ctrl & DMA_CH0_CTRL_TRIG_BSWAP_BITS.not()
        };
    }

    fn set_irq_quiet(&mut self, irq_quiet: bool) {
        self.ctrl = if irq_quiet {
            self.ctrl | DMA_CH0_CTRL_TRIG_IRQ_QUIET_BITS
        } else {
            self.ctrl & DMA_CH0_CTRL_TRIG_IRQ_QUIET_BITS.not()
        };
    }

    fn set_enable(&mut self, enable: bool) {
        self.ctrl = if enable {
            self.ctrl | DMA_CH0_CTRL_TRIG_EN_BITS
        } else {
            self.ctrl & DMA_CH0_CTRL_TRIG_EN_BITS.not()
        };
    }

    fn set_sniff_enable(&mut self, sniff_enable: bool) {
        self.ctrl = if sniff_enable {
            self.ctrl | DMA_CH0_CTRL_TRIG_SNIFF_EN_BITS
        } else {
            self.ctrl & DMA_CH0_CTRL_TRIG_SNIFF_EN_BITS.not()
        };
    }

    fn default_for_channel(channel: u32) -> Self {
        let mut config = DMAChannelConfig {
            ctrl: 0
        };

        config.set_read_increment(true);
        config.set_write_increment(false);
        config.set_dreq(DREQ_FORCE);
        config.set_chain_to(channel);
        config.set_transfer_data_size(DMAChannelSize::DMA_SIZE_32);
        config.set_ring(false, 0);
        config.set_bswap(false);
        config.set_irq_quiet(false);
        config.set_enable(true);
        config.set_sniff_enable(false);

        config
    }
}

fn dma_channel_set_read_addr(pac: &pac::Peripherals, dma_channel: u32, source_location: *const u8, trigger: bool) {
    let dma_channel : &pac::dma::CH = &pac.DMA.ch[dma_channel as usize];

    if !trigger {
        dma_channel.ch_read_addr.write_with_zero(|x| unsafe {x.bits(source_location as u32)});
    } else {
        //dma_channel.ch_al3_read_addr_trig.write_with_zero(|x| unsafe {x.bits(source_location as u32)});
    }

}

fn dma_channel_set_write_addr(pac: &pac::Peripherals, dma_channel: u32, target_location: *mut u8, trigger: bool) {
    let dma_channel : &pac::dma::CH = &pac.DMA.ch[dma_channel as usize];

    if !trigger {
        dma_channel.ch_write_addr.write_with_zero(|x| unsafe {x.bits(target_location as u32)});
    } else {
        //dma_channel.ch_al2_write_addr_trig.write_with_zero(|x| unsafe {x.bits(target_location as u32)});
    }
}

fn dma_channel_set_trans_count(pac: &pac::Peripherals, dma_channel: u32, trans_count: u32, trigger: bool) {
    let dma_channel : &pac::dma::CH = &pac.DMA.ch[dma_channel as usize];

    if !trigger {
        dma_channel.ch_trans_count.write_with_zero(|x| unsafe {x.bits(trans_count)});
    } else {
        //dma_channel.ch_al1_trans_count_trig.write_with_zero(|x| unsafe {x.bits(trans_count)});
    }
}

fn dma_channel_set_config(pac: &pac::Peripherals, dma_channel: u32, config: &DMAChannelConfig, trigger: bool) {
    let dma_channel : &pac::dma::CH = &pac.DMA.ch[dma_channel as usize];

    if !trigger {
        // TODO: Check if this will actually work
        dma_channel.ch_ctrl_trig.write_with_zero(|x| unsafe { x.bits(config.ctrl) });
    } else {
        dma_channel.ch_ctrl_trig.write_with_zero(|x| unsafe { x.bits(config.ctrl) });
    }
}

fn dma_channel_configure(pac: &pac::Peripherals, dma_channel: u32, config: &DMAChannelConfig,
                         target_location: *mut u8, source_location: *const u8, size: usize, trigger: bool) {
    dma_channel_set_read_addr(pac, dma_channel, source_location, false);
    dma_channel_set_write_addr(pac, dma_channel, target_location, false);
    dma_channel_set_trans_count(pac, dma_channel, size as _, false);
    dma_channel_set_config(pac, dma_channel, config, trigger);
}

fn dma_channel_wait_for_finish_blocking(pac: &pac::Peripherals, channel: u32) {
    let channel : &pac::dma::CH = &pac.DMA.ch[channel as usize];
    while (channel.ch_al1_ctrl.read().bits() & DMA_CH0_CTRL_TRIG_BUSY_BITS) != 0 {}
}*/

/*fn st7789_command_dma(pac: &pac::Peripherals, channel: u32, command: ST7789Registers, data: Option<&[u8]>) {
    // Ported from https://github.com/pimoroni/pimoroni-pico/blob/main/drivers/st7789/st7789.cpp:

    // Wait for DMA to be ready
    dma_channel_wait_for_finish_blocking(pac, 0);

    // Write 0 (low) to the CS and DC pins
    pac.SIO
        .gpio_out_clr
        .modify(|_, x| unsafe { x.bits((1 << CS) | (1 << DC)) });

    spi_write_blocking(pac, &[command.get_value() as _]);

    if let Some(data) = data {
        // Data mode
        pac.SIO.gpio_out_set.modify(|_, x| unsafe { x.bits(1 << DC) });

        let channel : &pac::dma::CH = &pac.DMA.ch[channel as usize];
        //channel.ch_al3_read_addr_trig.write_with_zero(|x| unsafe { x.bits(data.as_ptr() as _) });
        channel.ch_write_addr.write_with_zero(|x| unsafe { x.bits(data.as_ptr() as _) });

        //spi_write_blocking(pac, data);
    } else {
        pac.SIO.gpio_out_set.modify(|_, x| unsafe { x.bits(1 << CS) });
    }
}*/
