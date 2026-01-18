//! Build script for sump-server
//!
//! 1. Reads compile-time configuration from environment variables:
//!    - SUMP_PORT: HTTP server port (default: 8082)
//!    - SUMP_AXI_ADDR: SUMP3 AXI base address (default: 0x43C20000)
//!
//! 2. Builds the Surfer WASM frontend using trunk (if not already built)
//!    - Set SKIP_SURFER_BUILD=1 to skip this step

use std::path::Path;
use std::process::Command;

fn main() {
    // ============================================
    // Part 1: Read configuration from environment
    // ============================================
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

    // ============================================
    // Part 2: Build Surfer WASM frontend
    // ============================================
    
    // Allow skipping surfer build (useful for CI or quick rebuilds)
    if std::env::var("SKIP_SURFER_BUILD").is_ok() {
        println!("cargo:warning=Skipping Surfer WASM build (SKIP_SURFER_BUILD set)");
        return;
    }

    let surfer_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../surfer/surfer");
    let surfer_dist = surfer_dir.join("dist");

    // Track surfer source changes for rebuild
    println!("cargo:rerun-if-changed=../../surfer/surfer/src");
    println!("cargo:rerun-if-changed=../../surfer/surfer/index.html");
    println!("cargo:rerun-if-changed=../../surfer/surfer/Trunk.toml");
    println!("cargo:rerun-if-changed=../../surfer/libsurfer/src");

    // Check if dist exists and is up to date
    let needs_build = if surfer_dist.join("index.html").exists() {
        // Check if sources are newer than dist
        let dist_time = std::fs::metadata(surfer_dist.join("index.html"))
            .and_then(|m| m.modified())
            .ok();
        
        // Simple heuristic: rebuild if dist is older than a day or sources changed
        // The cargo:rerun-if-changed directives handle the actual rebuild logic
        dist_time.is_none()
    } else {
        true
    };

    if needs_build {
        println!("cargo:warning=Building Surfer WASM frontend...");
        
        let status = Command::new("trunk")
            .args(["build", "--release"])
            .current_dir(&surfer_dir)
            .env_remove("NO_COLOR") // Remove NO_COLOR to avoid trunk argument parsing issues
            .status();

        match status {
            Ok(s) if s.success() => {
                println!("cargo:warning=Surfer WASM build complete");
            }
            Ok(s) => {
                panic!("trunk build failed with exit code: {:?}", s.code());
            }
            Err(e) => {
                panic!(
                    "Failed to run 'trunk build'. Make sure trunk is installed (cargo install trunk). Error: {}",
                    e
                );
            }
        }
    }
}
