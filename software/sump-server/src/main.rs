//! SUMP3 ILA Web Server
//!
//! Standalone HTTP server for the SUMP3 Integrated Logic Analyzer.
//! Provides a REST API and serves the embedded Surfer WASM frontend.
//!
//! ## Build-time Configuration
//! - `SUMP_PORT`: HTTP server port (default: 8082)
//! - `SUMP_AXI_ADDR`: SUMP3 AXI base address in hex (default: 0x43C20000)
//! - `SKIP_SURFER_BUILD`: Set to skip building the Surfer frontend
//!
//! ## Runtime Configuration
//! - `PORT`: Override server port at runtime
//! - `SUMP_AXI_ADDR`: Override AXI address at runtime

mod devmem;
mod ila;

use axum::{
    body::Body,
    http::{header, StatusCode, Uri},
    response::{IntoResponse, Response},
    Router,
};
use rust_embed::Embed;
use std::net::SocketAddr;
use std::sync::Arc;
use tower_http::cors::{Any, CorsLayer};
use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt};

/// Embedded Surfer WASM frontend files (built by trunk during cargo build)
#[derive(Embed)]
#[folder = "../../surfer/surfer/dist/"]
struct Assets;

/// Default port (set at compile time via build.rs)
const DEFAULT_PORT: u16 = {
    match option_env!("SUMP_DEFAULT_PORT") {
        Some(s) => {
            // const-compatible parsing
            let bytes = s.as_bytes();
            let mut result: u16 = 0;
            let mut i = 0;
            while i < bytes.len() {
                result = result * 10 + (bytes[i] - b'0') as u16;
                i += 1;
            }
            result
        }
        None => 8082,
    }
};

/// Default AXI address (set at compile time via build.rs)
const DEFAULT_AXI_ADDR: &str = match option_env!("SUMP_DEFAULT_AXI_ADDR") {
    Some(s) => s,
    None => "0x43C20000",
};

/// Serve embedded static files
async fn serve_static(uri: Uri) -> impl IntoResponse {
    let path = uri.path().trim_start_matches('/');
    
    // Default to index.html for root or missing files (SPA routing)
    let path = if path.is_empty() { "index.html" } else { path };
    
    match Assets::get(path) {
        Some(content) => {
            let mime = mime_guess::from_path(path).first_or_octet_stream();
            Response::builder()
                .status(StatusCode::OK)
                .header(header::CONTENT_TYPE, mime.as_ref())
                .body(Body::from(content.data.into_owned()))
                .unwrap()
        }
        None => {
            // For SPA routing, serve index.html for unknown paths
            match Assets::get("index.html") {
                Some(content) => Response::builder()
                    .status(StatusCode::OK)
                    .header(header::CONTENT_TYPE, "text/html")
                    .body(Body::from(content.data.into_owned()))
                    .unwrap(),
                None => Response::builder()
                    .status(StatusCode::NOT_FOUND)
                    .body(Body::from("Not Found"))
                    .unwrap(),
            }
        }
    }
}

#[tokio::main(flavor = "current_thread")]
async fn main() {
    // Initialize logging
    tracing_subscriber::registry()
        .with(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "sump_server=info,tower_http=info".into()),
        )
        .with(tracing_subscriber::fmt::layer().with_target(false))
        .init();

    tracing::info!("SUMP3 ILA Server starting...");
    tracing::info!("Build defaults: port={}, axi_addr={}", DEFAULT_PORT, DEFAULT_AXI_ADDR);

    // Parse AXI address (runtime override or build-time default)
    let axi_addr_str = std::env::var("SUMP_AXI_ADDR")
        .unwrap_or_else(|_| DEFAULT_AXI_ADDR.to_string());
    
    let axi_addr = if axi_addr_str.starts_with("0x") || axi_addr_str.starts_with("0X") {
        usize::from_str_radix(&axi_addr_str[2..], 16)
            .expect("Invalid SUMP_AXI_ADDR format")
    } else {
        axi_addr_str.parse().expect("Invalid SUMP_AXI_ADDR format")
    };
    
    tracing::info!("Using AXI address: 0x{:08X}", axi_addr);

    // Initialize ILA state
    let ila_state = match ila::IlaState::new(axi_addr) {
        Ok(state) => Arc::new(state),
        Err(e) => {
            tracing::error!("Failed to initialize ILA at 0x{:08X}: {}", axi_addr, e);
            tracing::error!("Make sure you have permission to access /dev/mem (run as root)");
            std::process::exit(1);
        }
    };

    // CORS configuration for development (allows any origin)
    // Useful when running surfer locally against a remote sump-server
    let cors = CorsLayer::new()
        .allow_origin(Any)
        .allow_methods(Any)
        .allow_headers(Any);

    // Build the application router
    let app = Router::new()
        .nest("/api/ila", ila::ila_router(ila_state))
        // Serve embedded static files as fallback
        .fallback(serve_static)
        .layer(cors);

    // Parse port from environment or use compile-time default
    let port: u16 = std::env::var("PORT")
        .ok()
        .and_then(|p| p.parse().ok())
        .unwrap_or(DEFAULT_PORT);

    let addr = SocketAddr::from(([0, 0, 0, 0], port));
    tracing::info!("Listening on http://{}", addr);

    // Create listener
    let listener = match tokio::net::TcpListener::bind(addr).await {
        Ok(l) => l,
        Err(e) => {
            tracing::error!("Failed to bind to {}: {}", addr, e);
            std::process::exit(1);
        }
    };

    // Run server with graceful shutdown
    axum::serve(listener, app)
        .with_graceful_shutdown(shutdown_signal())
        .await
        .unwrap();

    tracing::info!("Server shutdown complete");
}

/// Wait for shutdown signal (Ctrl+C or SIGTERM)
async fn shutdown_signal() {
    let ctrl_c = async {
        tokio::signal::ctrl_c()
            .await
            .expect("Failed to install Ctrl+C handler");
    };

    #[cfg(unix)]
    let terminate = async {
        tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())
            .expect("Failed to install SIGTERM handler")
            .recv()
            .await;
    };

    #[cfg(not(unix))]
    let terminate = std::future::pending::<()>();

    tokio::select! {
        _ = ctrl_c => tracing::info!("Received Ctrl+C, shutting down..."),
        _ = terminate => tracing::info!("Received SIGTERM, shutting down..."),
    }
}
