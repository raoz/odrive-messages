# odrive-messages

This is an unofficial Rust implemenation of the [ODrive](https://odriverobotics.com/) [CAN protocol](https://docs.odriverobotics.com/v/latest/manual/can-protocol.html) messages. This is a low-level crate, which only handles decoding and encoding the CAN Frames and does not handle sending them or waiting for responses. A higher-level crate using this one for ergonomic management of ODrives is envisioned.

## Limitations

This crate is not yet finished. While all the ODrive CAN protocol messages are supported, the interface this crate provides is not yet final and may change. In particular, the error handling is currently not particularly good.

### Roadmap:
- [X] Decode and encode any ODrive CAN message
- [ ] Convert ODrive error codes into human-readable errors
- [ ] Implement proper error handling
- [ ] Support `no-std`
- [ ] Provide better facilities for working with the `RxSdo` and `TxSdo` messages

## Usage

The crate provides top-level methods for generating CAN frames, which can be sent using any `can_embedded` compatible crate; for example `socketcan` on Linux. To parse CAN messages, one can use the top-level `parse_frame` function or `messagess::CanMessageWithId::from_frame` directly.

Example using socketcan:
```rust
    let socket = socketcan::CanSocket::open('can0')?;
    socket.set_read_timeout(Duration::from_millis(200))?;

    socket.write_frame(odrive_messages::query_all_addresses());

    loop {
        match socket.read_frame() {
            Ok(frame) => {
                println!("{:?}", odrive_messagess::parse_frame(frame));
            }
            Err(e) => {
                eprintln!("{e:?}");
                break;
            }
        }
    }
```


## License

This crate is published under the [MIT License](LICENSE).