fn main() {
    // Put `memory.x` in our output directory and ensure it's on the linker search path.
    let out = std::env::var("OUT_DIR").unwrap();
    let out_dir = std::path::Path::new(&out);
    std::fs::copy("memory.x", out_dir.join("memory.x")).unwrap();
    std::fs::copy("psram.x", out_dir.join("psram.x")).unwrap();
    println!("cargo:rustc-link-search={out}");
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=psram.x");

    // `--nmagic` is required if memory section addresses are not aligned to 0x10000,
    // for example the FLASH and RAM sections in your `memory.x`.
    // See https://github.com/rust-embedded/cortex-m-quickstart/pull/95
    println!("cargo:rustc-link-arg=--nmagic");

    println!("cargo:rustc-link-arg=-Tlink.x");
    println!("cargo:rustc-link-arg=-Tpsram.x");

    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
