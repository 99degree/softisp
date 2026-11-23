use cam_disp::DrmDisplay;
fn main() {
    // Reference the type without constructing it: DrmDisplay::new() opens
    // /dev/dri/card0, which is unavailable in CI/Termux.
    let _display_type: Option<DrmDisplay> = None;
    println!("DRM display module loaded");
}
