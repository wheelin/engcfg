#![cfg_attr(not(test), no_std)]

//! # EngCfg
//!
//! ## Introduction
//!
//! This crate allows to generate 4-stroke engine waveforms for direct writing on GPIO as bit mask.
//!
//! The goal is to generate a pulse train with the following information:
//! * crankshaft wheel signal
//! * camshaft wheel signal
//! * top-dead-center piston position for each cylinder.
//!
//! ## Technical information
//!
//! The pulse train buffer is an array of [`EngBit`] (generally u8, u16 or u32, depending on the GPIO register width) with length 7200. An array has been used
//! in order to be compatible with DMA (Direct Memory Access) mechanisms. The goal
//! of this crate is really to create crank/cam/tdc generators for ECU development purpose.
//!
//! Each array element is a bit field representing the current crank, cam, tdc state, at a precise
//! position (the array index) in the engine cycle:
//!
//! | Bit | Desc.     |
//! |:----|-----------|
//! |0    | Camshaft  |
//! |1    | Crankshaft|
//! |2    | TDC cyl. 1|
//! |3    | TDC cyl. 2|
//! |4    | TDC cyl. 3|
//! |5    | TDC cyl. 4|
//! |6    | TDC cyl. 5|
//! |7    | TDC cyl. 6|
//!
//! When generated, the pulse train array shall be applied on output pins using a strict timing:
//! * either by using interrupts and an hardware timer, by writing each element of the array on the GPIO port one by one and restarting to 0 at the end.
//! * or by using a DMA with circular configuration and timer triggering. This way, the CPU has really nothing to do but reconfigure the timer in case of engine speed changes.
//!
//! ## Limitations
//!
//! The following engines can not be generated at the moment:
//! * Engines with more than 6 cylinders
//! * Asymmetrical engines (TDCs are not spaced evenly)
//! * The concept uses a relatively high amount of RAM. But with the use of appropriate DMA and timers, the pulse train generation should not even require CPU processing.
//!

/// Pulse train bit manipulation
/// 
/// Defines function helpers and positions in the pulse train element of
/// * camshaft signal
/// * crankshaft signal
/// * TDCs
/// using masks.
pub trait EngBit: Sized {
    /// Camshaft signal mask, indicates bit position in the pulse train element
    const CAM_MSK: Self;
    /// Crankshaft signal mask, indicates bit position in the pulse train element
    const CRK_MSK: Self;
    /// TDCs position (max 6, can be left as is when unused), indicates bit position in the pulse train element
    const TDC_MSK: [Self; 6];

    /// Set camshaft signal bit to `lvl`
    fn set_cam_lvl(&mut self, lvl: Level);
    /// Check if camshaft signal is high
    fn cam_is_high(&self) -> bool;
    /// Check if camshaft signal is low
    fn cam_is_low(&self) -> bool;

    /// Set crankshaft signal bit to `lvl`
    fn set_crk_lvl(&mut self, lvl: Level);
    /// Check if crankshaft signal is high
    fn crk_is_high(&self) -> bool;
    /// Check if crankshaft signal is low
    fn crk_is_low(&self) -> bool;

    /// Set TDC for cylinder `tdc` signal bit to `lvl`
    fn set_tdc_lvl(&mut self, cyl: usize, lvl: Level);
    /// Check if TDC for cylinder `tdc` signal is high
    fn tdc_is_high(&self, cyl: usize) -> bool;
    /// Check if TDC for cylinder `tdc` signal is low
    fn tdc_is_low(&self, cyl: usize) -> bool;
}

impl EngBit for u8 {
    fn set_cam_lvl(&mut self, lvl: Level) {
        match lvl {
            Level::Low => *self &= !Self::CAM_MSK,
            Level::High => *self |= Self::CAM_MSK,
        };
    }
    
    fn cam_is_high(&self) -> bool {
        self & Self::CAM_MSK != 0
    }
    
    fn cam_is_low(&self) -> bool {
        !self.cam_is_high()
    }
    
    fn set_crk_lvl(&mut self, lvl: Level) {
        match lvl {
            Level::Low => *self &= !Self::CRK_MSK,
            Level::High => *self |= Self::CRK_MSK,
        };
    }
    
    fn crk_is_high(&self) -> bool {
        self & Self::CRK_MSK != 0
    }
    
    fn crk_is_low(&self) -> bool {
        !self.crk_is_high()
    }
    
    fn set_tdc_lvl(&mut self, tdc: usize, lvl: Level) {
        match lvl {
            Level::Low => *self &= !Self::TDC_MSK[tdc],
            Level::High => *self |= Self::TDC_MSK[tdc],
        };
    }

    fn tdc_is_high(&self, tdc: usize) -> bool {
        self & Self::TDC_MSK[tdc] != 0
    }
    
    fn tdc_is_low(&self, tdc: usize) -> bool {
        !self.tdc_is_high(tdc)
    }

    const CAM_MSK: u8 = 0x01;
    const CRK_MSK: u8 = 0x02;
    const TDC_MSK: [u8; 6] = [0x04, 0x08, 0x10, 0x20, 0x40, 0x80];
}

/// Level implementation, for crank, cam and TDC signaling
#[derive(Eq, PartialEq, Clone, Copy)]
pub enum Level {
    Low = 0,
    High = 1,
}

impl core::ops::Not for Level {
    type Output = Level;

    fn not(self) -> Self::Output {
        if self == Level::High {
            Level::Low
        } else {
            Level::High
        }
    }
}

/// Number of cylinders for the current engine, helps to calculate TDC positions
pub enum CylNr {
    /// Engine with 4 cylinders
    Cyl4,
    /// Engine with 6 cylinders
    Cyl6,
}

impl CylNr {
    /// Enum conversion into real cylinder number
    pub const fn val(&self) -> usize {
        match *self {
            CylNr::Cyl4 => 4,
            CylNr::Cyl6 => 6,
        }
    }
}

/// Type of crankshaft
pub enum CrkType {
    /// 30 teeth, 1 missing, gap level is low
    Crk30m1,
    /// 30 teeth, 2 missing, gap level is low
    Crk30m2,
    /// 60 teeth, 1 missing, gap level is low
    Crk60m1,
    /// 60 teeth, 1 missing, gap level is low
    Crk60m2,
    /// 120 teeth, 1 missing, gap level is low
    Crk120m1,
    /// 120 teeth, 2 missing, gap level is low
    Crk120m2,
    /// 30 teeth, 1 missing, gap level is high
    Crk30m1Inv,
    /// 30 teeth, 2 missing, gap level is high
    Crk30m2Inv,
    /// 60 teeth, 1 missing, gap level is high
    Crk60m1Inv,
    /// 60 teeth, 2 missing, gap level is high
    Crk60m2Inv,
    /// 120 teeth, 1 missing, gap level is high
    Crk120m1Inv,
    /// 120 teeth, 2 missing, gap level is high
    Crk120m2Inv,
}

impl CrkType {
    /// Returns the number of teeth for the current wheel
    pub const fn nr_of_teeth(&self) -> usize {
        match *self {
            CrkType::Crk30m1 | CrkType::Crk30m2 | CrkType::Crk30m1Inv | CrkType::Crk30m2Inv => 30,
            CrkType::Crk60m1 | CrkType::Crk60m2 | CrkType::Crk60m1Inv | CrkType::Crk60m2Inv => 60,
            CrkType::Crk120m1 | CrkType::Crk120m2 | CrkType::Crk120m1Inv | CrkType::Crk120m2Inv => {
                120
            }
        }
    }

    /// Returns the number of missing teeth for the current wheel
    pub const fn nr_of_missing_teeth(&self) -> usize {
        match *self {
            CrkType::Crk30m1
            | CrkType::Crk30m1Inv
            | CrkType::Crk60m1
            | CrkType::Crk60m1Inv
            | CrkType::Crk120m1
            | CrkType::Crk120m1Inv => 1,
            CrkType::Crk30m2
            | CrkType::Crk30m2Inv
            | CrkType::Crk60m2
            | CrkType::Crk60m2Inv
            | CrkType::Crk120m2
            | CrkType::Crk120m2Inv => 2,
        }
    }

    /// Returns the tooth size in degrees DEG_I16_DEC1
    pub const fn angle_per_tooth(&self) -> usize {
        match *self {
            CrkType::Crk30m1 | CrkType::Crk30m2 | CrkType::Crk30m1Inv | CrkType::Crk30m2Inv => {
                3600 / 30
            }
            CrkType::Crk60m1 | CrkType::Crk60m2 | CrkType::Crk60m1Inv | CrkType::Crk60m2Inv => {
                3600 / 60
            }
            CrkType::Crk120m1 | CrkType::Crk120m2 | CrkType::Crk120m1Inv | CrkType::Crk120m2Inv => {
                3600 / 120
            }
        }
    }

    /// Returns first level seen when starting a rotation from angle 0
    pub const fn first_level(&self) -> Level {
        match *self {
            CrkType::Crk30m1
            | CrkType::Crk30m2
            | CrkType::Crk60m1
            | CrkType::Crk60m2
            | CrkType::Crk120m1
            | CrkType::Crk120m2 => Level::High,
            CrkType::Crk30m1Inv
            | CrkType::Crk30m2Inv
            | CrkType::Crk60m1Inv
            | CrkType::Crk60m2Inv
            | CrkType::Crk120m1Inv
            | CrkType::Crk120m2Inv => Level::Low,
        }
    }
}

/// Camshaft wheel configuration
pub struct Cam {
    /// Level when first crankshaft gap is met
    pub first_level: Level,
    /// Angle of each camshaft edge, starting from the first crankshaft gap, DEG_I16_DEC1. Negative angle if not used.
    pub ev_angles: [i16; 20],
}

/// Engine configuration
pub struct EngCfg {
    /// Camshaft wheel configuration
    pub cam: Cam,
    /// Crankshaft type configuration
    pub crk: CrkType,
    /// Angle from reference (crank gap) to first Top-Dead-Center (TDC): DEG_S16_DEC1
    pub ref_to_tdc0: i16,
    /// Number of cylinders, used for TDC generation
    pub nr_of_cyl: CylNr,
}

impl EngCfg {
    /// Arguments:
    /// * pt: output argument, pulse train generated from engine configuration for waveform generation
    ///
    /// Returns:
    /// * Ok: generation has been achieved correctly
    /// * Err: the buffer has not the minimal required length
    pub fn gen_pulse_train<T:EngBit>(&self, pt: &mut [T; 7200]) {
        let mut idx_cam_edges = 0;
        let mut cam_lvl = self.cam.first_level;

        let crk_tooth_angle = self.crk.angle_per_tooth();
        let angle_missing_teeth = self.crk.nr_of_missing_teeth() * self.crk.angle_per_tooth();
        let mut crk_lvl = self.crk.first_level();

        for (angle, val) in pt.iter_mut().enumerate() {
            val.set_cam_lvl(cam_lvl);
            if idx_cam_edges < 20 {
                if self.cam.ev_angles[idx_cam_edges] == angle as i16 {
                    cam_lvl = !cam_lvl;
                    idx_cam_edges += 1;
                }
            }

            val.set_crk_lvl(crk_lvl);
            if angle % ((crk_tooth_angle / 2) as usize) == 0 && angle != 0 {
                if (angle % 3600) >= 3600 - angle_missing_teeth {
                    crk_lvl = !self.crk.first_level();
                } else {
                    crk_lvl = !crk_lvl;
                }
            }
        }

        let tdc_to_tdc = 7200 / self.nr_of_cyl.val();
        pt[self.ref_to_tdc0 as usize].set_tdc_lvl(0, Level::High);

        for cyl in 1..self.nr_of_cyl.val() {
            pt[self.ref_to_tdc0 as usize + (cyl * tdc_to_tdc)].set_tdc_lvl(cyl, Level::High);
        }
    }
}

pub static CFGS: [EngCfg; 1] = [EngCfg {
    cam: Cam {
        first_level: Level::High,
        ev_angles: [
            289, 389, 1189, 1289, 1489, 1589, 2089, 2189, 2689, 2789, 3889, 3989, 5089, 5189, 5689,
            5789, 6289, 6389, 6589, 6689,
        ],
    },
    crk: CrkType::Crk60m2Inv,
    ref_to_tdc0: 658,
    nr_of_cyl: CylNr::Cyl6,
}];

#[cfg(test)]
mod tests {
    use core::panic;

    use crate::EngBit;
    use crate::CFGS;
    use rstest::rstest;

    #[rstest(
        angle,
        tdc,
        expected,
        case(657, 0, false),
        case(658, 0, true),
        case(659, 0, false),
        case(1857, 1, false),
        case(1858, 1, true),
        case(1859, 1, false),
        case(3057, 2, false),
        case(3058, 2, true),
        case(3059, 2, false),
        case(4257, 3, false),
        case(4258, 3, true),
        case(4259, 3, false),
        case(5457, 4, false),
        case(5458, 4, true),
        case(5459, 4, false),
        case(6657, 5, false),
        case(6658, 5, true),
        case(6659, 5, false)
    )]
    fn tdc_test(angle: usize, tdc: usize, expected: bool) {
        let mut pls = [0u8; 7200];

        CFGS[0].gen_pulse_train(&mut pls);
        assert_eq!(expected, pls[angle].tdc_is_high(tdc))
    }

    #[rstest(
        angle,
        expected,
        case(0, true),
        case(1, true),
        case(290, false),
        case(388, false),
        case(390, true),
        case(1190, false),
        case(1288, false),
        case(1290, true),
        case(1490, false),
        case(1590, true),
        case(2090, false),
        case(2190, true),
        case(2690, false),
        case(2790, true),
        case(3890, false),
        case(3990, true),
        case(5090, false),
        case(5190, true),
        case(5690, false),
        case(5790, true),
        case(6290, false),
        case(6390, true),
        case(6590, false),
        case(6690, true),
        case(7199, true)
    )]
    fn cam_test(angle: usize, expected: bool) {
        let mut pls = [0u8; 7200];

        CFGS[0].gen_pulse_train(&mut pls);
        assert_eq!(expected, pls[angle].cam_is_high())
    }

    #[rstest(
        angle,
        expected,
        case(3449, false),
        case(3481, true),
        case(3599, true),
        case(3601, false),
        case(7049, false),
        case(7081, true),
        case(7199, true),
        case(0, false)
    )]
    fn crk_gap_test(angle: usize, expected: bool) {
        let mut pls = [0u8; 7200];

        CFGS[0].gen_pulse_train(&mut pls);
        assert_eq!(expected, pls[angle].crk_is_high());
    }
}
