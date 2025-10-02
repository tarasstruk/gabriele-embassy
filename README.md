
# The RP2040 part of Gabriele Project

See: https://github.com/tarasstruk/gabriele/pull/2

Run Example:

```bash
cargo run -- --tty /dev/tty.usbmodem123456781 --text welcome.txt
```

## Linux pre-requisites:


```sh
sudo apt-get install -y  libudev-dev
cargo install elf2uf2-rs

```

