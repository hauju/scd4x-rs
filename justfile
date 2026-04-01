# Default recipe
default:
    @just --list

# Run library tests
test:
    cargo test

# Run clippy on the library
clippy:
    cargo clippy -- -D warnings

# Format code
fmt:
    cargo fmt

# Check library (default features)
check:
    cargo check
