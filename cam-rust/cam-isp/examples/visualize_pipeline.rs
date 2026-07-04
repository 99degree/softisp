//! Example: visualize pipeline topology as DOT/Graphviz.
//!
//! Generates a DOT file that can be rendered with:
//!   dot -Tpng pipeline.dot -o pipeline.png
//!
//! Or paste into https://dreampuf.github.io/GraphvizOnline/
//!
//! Usage:
//!   cargo run --example visualize_pipeline -p cam-isp --features mnn

use cam_isp::pipeline_builder::PipelineBuilder;

fn main() {
    println!("Pipeline Visualization Example");
    println!("==============================\n");

    // Build a photo pipeline
    let builder = PipelineBuilder::new(3840, 2160)
        .unpack()
        .demosaic_mhc()
        .gamma(2.2)
        .sharpen(0.5)
        .contrast(1.3)
        .display();

    // Generate DOT graph
    let dot = builder.to_dot();

    // Save to file
    let path = "pipeline.dot";
    std::fs::write(path, &dot).expect("Failed to write DOT file");
    println!("Saved DOT graph to: {}", path);
    println!("\nRender with:");
    println!("  dot -Tpng {} -o pipeline.png", path);
    println!("  dot -Tsvg {} -o pipeline.svg", path);
    println!("\nOr paste into: https://dreampuf.github.io/GraphvizOnline/\n");

    // Show the DOT content
    println!("DOT content:");
    println!("============");
    println!("{}", dot);

    // Also show cost estimate
    let (flops, mem) = builder.cost();
    println!("\nCost Estimate:");
    println!("  FLOPs: {:.2} G", flops as f64 / 1e9);
    println!("  Memory: {:.2} MB", mem as f64 / 1e6);
}
