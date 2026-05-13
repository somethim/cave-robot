build:
	cargo build --workspace

check:
	cargo check --workspace

test:
	cargo test --workspace

generate:
	cargo run -p generator

robot:
	cargo run -p robot

dev:
	cargo run -p generator
	cargo run -p robot