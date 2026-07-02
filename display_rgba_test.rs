use cam_isp::blocks::DisplayBlock;
use cam_isp::engine::OutputFormat;

fn main() {
    let display = DisplayBlock::new(64).rgba();
    let nodes = display.nodes();
    println!("RGBA DisplayBlock emits {} nodes:", nodes.len());
    for (i, node) in nodes.iter().enumerate() {
        println!("{:2}: {:?}", i, String::from_utf8_lossy(node));
    }
}
