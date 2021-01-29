#![cfg_attr(not(test), no_std)]
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
    pub const fn val(&self) -> usize {
        match *self {
            CylNr::Cyl4 => 4,
            CylNr::Cyl6 => 6,
        }
    }
}

/// Type of crankshaft
pub enum CrkType {
    Crk30m1,
    Crk30m2,
    Crk60m1,
    Crk60m2,
    Crk120m1,
    Crk120m2,
    Crk30m1Inv,
    Crk30m2Inv,
    Crk60m1Inv,
    Crk60m2Inv,
    Crk120m1Inv,
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
    first_level: Level,
    /// Angle of each camshaft edge, starting from the first crankshaft gap
    ev_angles: [i16; 20],
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
    /// * Err: the buffer has not the minimal required lenght
    pub fn gen_pulse_train(&self, pt: &mut [u8;7200]) {

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
        pt[self.ref_to_tdc0 as usize] |= TDC_MSK[0];

        for cyl in 1..self.nr_of_cyl.val() {
            pt[self.ref_to_tdc0 as usize + (cyl * tdc_to_tdc)] |= TDC_MSK[cyl];
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

trait EngBit {
    fn set_cam_lvl(&mut self, lvl: Level);
    fn cam_is_high(&self) -> bool;
    fn cam_is_low(&self) -> bool;

    fn set_crk_lvl(&mut self, lvl: Level);
    fn crk_is_high(&self) -> bool;
    fn crk_is_low(&self) -> bool;

    fn set_tdc_lvl(&mut self, tdc: usize, lvl: Level);
    fn tdc_is_high(&self, tdc: usize) -> bool;
    fn tdc_is_low(&self, tdc: usize) -> bool;
}

const CAM_MSK: u8 = 0x01;
const CRK_MSK: u8 = 0x02;
const TDC_MSK: [u8; 6] = [0x04, 0x08, 0x10, 0x20, 0x40, 0x80];

impl EngBit for u8 {
    fn cam_is_high(&self) -> bool {
        self & CAM_MSK != 0
    }

    fn crk_is_high(&self) -> bool {
        self & CRK_MSK != 0
    }

    fn tdc_is_high(&self, tdc: usize) -> bool {
        self & TDC_MSK[tdc] != 0
    }

    fn set_cam_lvl(&mut self, lvl: Level) {
        match lvl {
            Level::Low => *self &= !CAM_MSK,
            Level::High => *self |= CAM_MSK,
        };
    }

    fn set_crk_lvl(&mut self, lvl: Level) {
        match lvl {
            Level::Low => *self &= !CRK_MSK,
            Level::High => *self |= CRK_MSK,
        };
    }

    fn set_tdc_lvl(&mut self, tdc: usize, lvl: Level) {
        match lvl {
            Level::Low => *self &= !TDC_MSK[tdc],
            Level::High => *self |= TDC_MSK[tdc],
        };
    }

    fn cam_is_low(&self) -> bool {
        !self.cam_is_high()
    }

    fn crk_is_low(&self) -> bool {
        !self.crk_is_high()
    }

    fn tdc_is_low(&self, tdc: usize) -> bool {
        !self.tdc_is_high(tdc)
    }
}


#[cfg(test)]
mod tests {
    use core::panic;

    use crate::CFGS;
    use crate::EngBit;
    use rstest::rstest;

    #[rstest(angle, tdc, expected,
        case(657 , 0, false),
        case(658 , 0, true),
        case(659 , 0, false),
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

    #[rstest(angle, expected,
        case(0   , true),
        case(1   , true),
        case(290 , false),
        case(388 , false),
        case(390 , true),
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

    #[rstest(angle, expected,
        case(3449, false),
        case(3481, true),
        case(3599, true),
        case(3601, false),

        case(7049, false),
        case(7081, true),
        case(7199, true),
        case(0   , false),
    )]
    fn crk_gap_test(angle: usize, expected: bool) {
        let mut pls = [0u8; 7200];

        CFGS[0].gen_pulse_train(&mut pls);
        println!("0b{:b}", pls[angle]);
        assert_eq!(expected, pls[angle].crk_is_high());
    }

    #[test]
    fn pr_all() {
        let mut pls = [0u8; 7200];

        CFGS[0].gen_pulse_train(&mut pls);
        for idx in 0..300 {
            println!("{:04}: 0b{:08b}", idx, pls[idx]);
        }
        println!("######################");
        for idx in 3400..3800 {
            println!("{:04}: 0b{:08b}", idx, pls[idx]);
        }
        println!("######################");
        for idx in 7000..7199 {
            println!("{:04}: 0b{:08b}", idx, pls[idx]);
        }
        panic!();
    }
}
