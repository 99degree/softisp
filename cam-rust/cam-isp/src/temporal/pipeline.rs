//! FramePipeline — thread-wise temporal processing pipeline.

use super::coefficients::{FrameCoefficients, FrameHistory};
use super::frame::TemporalFrame;
use super::plugin::TemporalPlugin;

/// Thread-wise temporal processing pipeline.
pub struct FramePipeline {
    history: std::sync::Arc<std::sync::Mutex<FrameHistory>>,
    coefficients: std::sync::Arc<std::sync::Mutex<Option<FrameCoefficients>>>,
    input_tx: std::sync::mpsc::Sender<TemporalFrame>,
    output_rx: std::sync::mpsc::Receiver<TemporalFrame>,
    running: std::sync::Arc<std::sync::Mutex<bool>>,
}

impl FramePipeline {
    /// Create a new pipeline with a plugin.
    pub fn new(plugin: Box<dyn TemporalPlugin>, max_history: usize) -> Self {
        let history = std::sync::Arc::new(std::sync::Mutex::new(FrameHistory::new(max_history)));
        let coefficients = std::sync::Arc::new(std::sync::Mutex::new(None));
        let running = std::sync::Arc::new(std::sync::Mutex::new(false));
        let (input_tx, input_rx): (
            std::sync::mpsc::Sender<TemporalFrame>,
            std::sync::mpsc::Receiver<TemporalFrame>,
        ) = std::sync::mpsc::channel();
        let (output_tx, output_rx) = std::sync::mpsc::channel();

        // We need to own the extractor/processor in the threads
        // Create a wrapper that owns the plugin
        let plugin = std::sync::Arc::new(plugin);

        // Extraction thread — recover from poisoned mutexes so one bad frame
        // doesn't cascade-panic the pipeline.
        let hist = history.clone();
        let coeffs = coefficients.clone();
        let run = running.clone();
        let plugin_clone = plugin.clone();
        std::thread::spawn(move || {
            while *run.lock().unwrap_or_else(|e| e.into_inner()) {
                if let Some(frame) = hist.lock().unwrap_or_else(|e| e.into_inner()).prev_frame().cloned() {
                    let prev = coeffs.lock().unwrap_or_else(|e| e.into_inner()).clone();
                    *coeffs.lock().unwrap_or_else(|e| e.into_inner()) =
                        Some(plugin_clone.extractor().extract(&frame, prev.as_ref()));
                }
                std::thread::sleep(std::time::Duration::from_millis(1));
            }
        });

        // Processing thread
        let hist2 = history.clone();
        let coeffs2 = coefficients.clone();
        let run2 = running.clone();
        let plugin_clone2 = plugin.clone();
        std::thread::spawn(move || {
            while *run2.lock().unwrap_or_else(|e| e.into_inner()) {
                if let Ok(frame) = input_rx.recv() {
                    hist2.lock().unwrap_or_else(|e| e.into_inner()).push(frame.clone());
                    let output = if let Some(ref c) = *coeffs2.lock().unwrap_or_else(|e| e.into_inner()) {
                        plugin_clone2.processor().process(&frame, c)
                    } else {
                        frame
                    };
                    let _ = output_tx.send(output);
                }
            }
        });

        Self {
            history,
            coefficients,
            input_tx,
            output_rx,
            running,
        }
    }

    /// Start the pipeline.
    pub fn start(&self) {
        *self.running.lock().unwrap_or_else(|e| e.into_inner()) = true;
    }

    /// Stop the pipeline.
    pub fn stop(&self) {
        *self.running.lock().unwrap_or_else(|e| e.into_inner()) = false;
    }

    /// Send a frame for processing.
    pub fn send(&self, frame: TemporalFrame) -> Result<(), String> {
        self.input_tx.send(frame).map_err(|e| e.to_string())
    }

    /// Receive a processed frame.
    pub fn receive(&self) -> Option<TemporalFrame> {
        self.output_rx.recv().ok()
    }

    /// Get current coefficients.
    pub fn coefficients(&self) -> Option<FrameCoefficients> {
        self.coefficients.lock().unwrap_or_else(|e| e.into_inner()).clone()
    }

    /// Get history size.
    pub fn history_size(&self) -> usize {
        self.history.lock().unwrap_or_else(|e| e.into_inner()).len()
    }
}
