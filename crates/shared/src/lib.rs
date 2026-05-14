pub mod cave;
pub mod robot;

pub use cave::*;

use std::str::FromStr;

pub fn load_env() {
    dotenvy::dotenv().ok();
}

pub fn env_or<T: FromStr>(key: &str, default: T) -> T {
    std::env::var(key)
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(default)
}
