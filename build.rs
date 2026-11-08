fn main() {
    // Firmware linker script only, so the host sim links normally
    if std::env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("none") {
        println!("cargo:rustc-link-arg-bin=random-laser-pointer=-Tlinkall.x");
    }
}
