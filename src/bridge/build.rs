use std::env;
use std::path::PathBuf;

fn main() {
    let crate_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    
    cbindgen::Builder::new()
        .with_crate(crate_dir)
        .with_language(cbindgen::Language::C)
        .with_header("/* Lance Writer Bridge C Header - Auto-generated */")
        .generate()
        .expect("Unable to generate bindings")
        .write_to_file("target/lance_bridge.h");
    
    println!("cargo:rerun-if-changed=src/lib.rs");
    println!("cargo:rerun-if-changed=src/writer.rs");
    println!("cargo:rerun-if-changed=src/error.rs");
}

