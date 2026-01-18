//! Build script for sump-server
//!
//! Reads compile-time configuration from environment variables:
//! - SUMP_PORT: HTTP server port (default: 8082)
//! - SUMP_AXI_ADDR: SUMP3 AXI base address (default: 0x43C20000)

fn main() {
    // Read configuration from environment (set during build)
    let port = std::env::var("SUMP_PORT").unwrap_or_else(|_| "8082".to_string());
    let axi_addr = std::env::var("SUMP_AXI_ADDR").unwrap_or_else(|_| "0x43C20000".to_string());
    
    // Parse and validate
    let port: u16 = port.parse().expect("SUMP_PORT must be a valid port number");
    let axi_addr = if axi_addr.starts_with("0x") || axi_addr.starts_with("0X") {
        usize::from_str_radix(&axi_addr[2..], 16).expect("SUMP_AXI_ADDR must be a valid hex address")
    } else {
        axi_addr.parse().expect("SUMP_AXI_ADDR must be a valid address")
    };
    
    // Pass to compiler as cfg values
    println!("cargo:rustc-env=SUMP_DEFAULT_PORT={}", port);
    println!("cargo:rustc-env=SUMP_DEFAULT_AXI_ADDR=0x{:08X}", axi_addr);
    
    // Rebuild if these change
    println!("cargo:rerun-if-env-changed=SUMP_PORT");
    println!("cargo:rerun-if-env-changed=SUMP_AXI_ADDR");
}
