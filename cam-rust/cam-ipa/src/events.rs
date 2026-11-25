//! Event emitter for thread-safe callback invocation

use std::sync::{Arc, Mutex, mpsc};
use std::thread;
use crate::{ControlList, IpaCallbacks};

/// Internal event types
enum IpaEvent {
    MetadataReady(u32, ControlList),
    FrameDone(u32, u32),
    SetSensorControls(ControlList),
    SetIspParams,
    Shutdown,
}

/// Thread-safe event emitter
pub struct EventEmitter {
    sender: Option<mpsc::Sender<IpaEvent>>,
    handle: Option<thread::JoinHandle<()>>,
}

impl EventEmitter {
    /// Create a new event emitter
    pub fn new() -> Self {
        Self {
            sender: None,
            handle: None,
        }
    }
    
    /// Start the event loop with callbacks
    pub fn start(&mut self, callbacks: IpaCallbacks) {
        let (tx, rx) = mpsc::channel();
        
        let handle = thread::spawn(move || {
            while let Ok(event) = rx.recv() {
                match event {
                    IpaEvent::MetadataReady(frame, metadata) => {
                        if let Some(cb) = &callbacks.metadata_ready {
                            cb(frame, &metadata);
                        }
                    }
                    IpaEvent::FrameDone(frame, buffer_id) => {
                        if let Some(cb) = &callbacks.frame_done {
                            cb(frame, buffer_id);
                        }
                    }
                    IpaEvent::SetSensorControls(controls) => {
                        if let Some(cb) = &callbacks.set_sensor_controls {
                            cb(&controls);
                        }
                    }
                    IpaEvent::SetIspParams => {
                        if let Some(cb) = &callbacks.set_isp_params {
                            cb();
                        }
                    }
                    IpaEvent::Shutdown => break,
                }
            }
        });
        
        self.sender = Some(tx);
        self.handle = Some(handle);
    }
    
    /// Emit metadata ready event
    pub fn emit_metadata_ready(&self, frame: u32, metadata: ControlList) {
        if let Some(tx) = &self.sender {
            let _ = tx.send(IpaEvent::MetadataReady(frame, metadata));
        }
    }
    
    /// Emit frame done event
    pub fn emit_frame_done(&self, frame: u32, buffer_id: u32) {
        if let Some(tx) = &self.sender {
            let _ = tx.send(IpaEvent::FrameDone(frame, buffer_id));
        }
    }
    
    /// Emit set sensor controls event
    pub fn emit_set_sensor_controls(&self, controls: ControlList) {
        if let Some(tx) = &self.sender {
            let _ = tx.send(IpaEvent::SetSensorControls(controls));
        }
    }
    
    /// Emit set ISP params event
    pub fn emit_set_isp_params(&self) {
        if let Some(tx) = &self.sender {
            let _ = tx.send(IpaEvent::SetIspParams);
        }
    }
    
    /// Shutdown the event loop
    pub fn shutdown(&mut self) {
        if let Some(tx) = self.sender.take() {
            let _ = tx.send(IpaEvent::Shutdown);
        }
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}

impl Default for EventEmitter {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for EventEmitter {
    fn drop(&mut self) {
        self.shutdown();
    }
}