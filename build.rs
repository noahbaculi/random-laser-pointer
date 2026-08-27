fn main() {
    println!("cargo:rustc-link-arg-bins=-Tlinkall.x");
    // Required by esp-wifi, provides WiFi ROM symbols
    println!("cargo:rustc-link-arg-bins=-Trom_functions.x");
}
