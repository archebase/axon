use std::env;
use std::path::Path;

fn main() {
    let crate_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
    let out_dir = Path::new(&crate_dir).join("include").join("axon");
    
    // Create include directory if it doesn't exist
    std::fs::create_dir_all(&out_dir).expect("Unable to create include directory");
    
    // Generate C header using cbindgen.toml configuration
    cbindgen::generate(&crate_dir)
        .expect("Unable to generate bindings")
        .write_to_file(out_dir.join("lance_bridge.h"));
    
    // Rerun if source changes
    println!("cargo:rerun-if-changed=cbindgen.toml");
    println!("cargo:rerun-if-changed=src/lib.rs");
}
