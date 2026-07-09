//! Register Injector - Maps ISPOptimizedParams to hardware registers

use crate::types::{ISPOptimizedParams, ISPRegisters, RegisterLimits};
use std::fmt;

/// Inject optimized parameters into ISP register structure
pub fn inject_registers(params: &ISPOptimizedParams, limits: &RegisterLimits) -> ISPRegisters {
    // First clamp parameters
    let mut clamped = params.clone();
    clamped.clamp(limits);
    
    ISPRegisters::from_params(&clamped)
}

/// Register injector with configurable pipeline
pub struct RegisterInjector {
    limits: RegisterLimits,
    hooks: Vec<Box<dyn RegisterHook>>,
}

impl RegisterInjector {
    pub fn new(limits: RegisterLimits) -> Self {
        Self {
            limits,
            hooks: Vec::new(),
        }
    }
    
    pub fn add_hook(&mut self, hook: Box<dyn RegisterHook>) {
        self.hooks.push(hook);
    }
    
    pub fn inject(&self, params: &ISPOptimizedParams) -> ISPRegisters {
        let mut registers = inject_registers(params, &self.limits);
        
        // Run hooks (pre-write validation, logging, etc.)
        for hook in &self.hooks {
            hook.pre_write(&mut registers);
        }
        
        registers
    }
}

/// Trait for register injection hooks
pub trait RegisterHook: Send + Sync {
    fn pre_write(&self, registers: &mut ISPRegisters);
}

/// Example: Logging hook (uses std::fmt::Debug)
#[derive(Debug)]
pub struct LoggingHook;

impl RegisterHook for LoggingHook {
    fn pre_write(&self, registers: &mut ISPRegisters) {
        eprintln!("[ISP] Injecting registers: WB=({}, {}, {}), Zoom={}", 
            registers.wb_r_gain, registers.wb_g_gain, registers.wb_b_gain, 
            registers.zoom_scale);
    }
}

/// Example: Safety clamp hook (additional safety)
#[derive(Debug)]
pub struct SafetyHook;

impl RegisterHook for SafetyHook {
    fn pre_write(&self, registers: &mut ISPRegisters) {
        // Additional safety checks
        if registers.wb_r_gain > 40960 {  // 10.0 in Q4.12
            eprintln!("[ISP] WARNING: WB R gain exceeds safe limit, clamping");
            registers.wb_r_gain = 40960;
        }
        if registers.zoom_scale > 16384 {  // 4.0 in Q4.12
            eprintln!("[ISP] WARNING: Zoom exceeds safe limit, clamping");
            registers.zoom_scale = 16384;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_inject_registers() {
        let params = ISPOptimizedParams {
            wb_r_gain: 1.5,
            wb_g_gain: 1.0,
            wb_b_gain: 0.8,
            ccm: [[1.0, 0.1, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            tone_curve_lut: vec![0.0, 0.25, 0.5, 0.75, 1.0, 1.0, 1.0],
            zoom_factor: 1.5,
        };
        
        let limits = RegisterLimits::default();
        let registers = inject_registers(&params, &limits);
        
        assert_eq!(registers.wb_r_gain, 6144);  // 1.5 * 4096
        assert_eq!(registers.wb_g_gain, 4096);
        assert_eq!(registers.wb_b_gain, 3276);  // 0.8 * 4096
    }
}