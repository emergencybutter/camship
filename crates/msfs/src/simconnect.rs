//! A minimal SimConnect client speaking the wire protocol directly, so the app
//! needs neither the MSFS SDK nor `SimConnect.dll`.
//!
//! Every message starts with a little-endian header. Client to server:
//! `size, protocol, 0xF0000000 | function, send id`; server to client:
//! `size, protocol, receive id`. The function and receive ids, struct layouts
//! and the MSFS 2024 ("SunRise", protocol 6) handshake follow the open-source
//! `node-simconnect` implementation.

use std::io::{self, Read, Write};

/// MSFS 2024.
pub const PROTOCOL: u32 = 6;
const OPEN_VERSION: (u32, u32, u32, u32) = (12, 2, 282174, 999);
const OPEN_ALIAS: &[u8] = b"RS";

pub const OBJECT_ID_USER: u32 = 0;
pub const UNUSED: u32 = 0xFFFF_FFFF;

pub const DATATYPE_FLOAT64: u32 = 4;
pub const PERIOD_SIM_FRAME: u32 = 3;

pub const POSITION_REFERENTIAL_SIMOBJECT: u32 = 1;
pub const POSITION_REFERENTIAL_WORLD: u32 = 2;

pub const CAMERA_MASK_POSITION: u32 = 1;
pub const CAMERA_MASK_ROTATION: u32 = 2;
pub const CAMERA_MASK_FOV: u32 = 8;
pub const CAMERA_MASK_REFERENTIAL: u32 = 16;

pub const CAMERA_FLAG_ABOVE_GROUND: u32 = 2;

pub const CAMERA_ACQUIRED: u32 = 1;

pub const EXCEPTION_CAMERA_API: u32 = 46;

/// Function ids (client to server).
mod func {
    pub const OPEN: u32 = 0x01;
    pub const ADD_TO_DATA_DEFINITION: u32 = 0x0c;
    pub const REQUEST_DATA_ON_SIM_OBJECT: u32 = 0x0e;
    pub const SUBSCRIBE_TO_SYSTEM_EVENT: u32 = 0x17;
    pub const CAMERA_ACQUIRE: u32 = 0x5f;
    pub const CAMERA_RELEASE: u32 = 0x60;
    pub const CAMERA_SET: u32 = 0x62;
    pub const CAMERA_ENABLE_FLAG: u32 = 0x64;
    pub const SUBSCRIBE_TO_CAMERA_STATUS: u32 = 0x66;
}

/// Receive ids (server to client).
mod recv {
    pub const EXCEPTION: u32 = 1;
    pub const OPEN: u32 = 2;
    pub const QUIT: u32 = 3;
    pub const EVENT: u32 = 4;
    pub const EVENT_FILENAME: u32 = 6;
    pub const EVENT_FRAME: u32 = 7;
    pub const SIMOBJECT_DATA: u32 = 8;
    pub const CAMERA_STATUS: u32 = 41;
}

/// `SIMCONNECT_DATA_CAMERA`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CameraData {
    /// Meters from the object for `SIMOBJECT`; latitude, longitude (degrees)
    /// and altitude (meters) for `WORLD`.
    pub position: [f64; 3],
    pub position_referential: u32,
    pub position_object: u32,
    pub target: [f64; 3],
    /// Pitch, bank, heading, degrees.
    pub pbh: [f32; 3],
    pub rotation_referential: u32,
    pub rotation_object: u32,
    pub fov: f64,
}

/// A message from the simulator.
#[derive(Clone, Debug, PartialEq)]
pub enum Message {
    Open { sim_version: (u32, u32, u32, u32) },
    Quit,
    Exception { exception: u32, send_id: u32, index: u32 },
    Event { id: u32, data: u32 },
    EventFilename { id: u32, filename: String },
    Frame { id: u32 },
    SimObjectData { request_id: u32, values: Vec<f64> },
    CameraStatus { acquired: u32, game_controlled: bool },
    Other(u32),
}

/// Builds one outgoing packet.
struct Packet(Vec<u8>);

impl Packet {
    fn new(function: u32) -> Self {
        let mut p = Packet(Vec::with_capacity(128));
        p.u32(0); // size, set in `finish`
        p.u32(PROTOCOL);
        p.u32(0xF000_0000 | function);
        p.u32(0); // send id, set in `finish`
        p
    }

    fn u32(&mut self, v: u32) -> &mut Self {
        self.0.extend_from_slice(&v.to_le_bytes());
        self
    }

    fn f32(&mut self, v: f32) -> &mut Self {
        self.0.extend_from_slice(&v.to_le_bytes());
        self
    }

    fn f64(&mut self, v: f64) -> &mut Self {
        self.0.extend_from_slice(&v.to_le_bytes());
        self
    }

    fn u8(&mut self, v: u8) -> &mut Self {
        self.0.push(v);
        self
    }

    /// A fixed-size, zero-padded string field (always NUL-terminated).
    fn str(&mut self, s: &str, len: usize) -> &mut Self {
        let bytes: Vec<u8> = s.bytes().take(len - 1).collect();
        self.0.extend_from_slice(&bytes);
        self.0.resize(self.0.len() + len - bytes.len(), 0);
        self
    }

    fn finish(mut self, send_id: u32) -> Vec<u8> {
        let size = self.0.len() as u32;
        self.0[0..4].copy_from_slice(&size.to_le_bytes());
        self.0[12..16].copy_from_slice(&send_id.to_le_bytes());
        self.0
    }
}

/// Reads little-endian fields from a received message body.
struct Reader<'a>(&'a [u8]);

impl Reader<'_> {
    fn u32(&mut self) -> Option<u32> {
        let (head, rest) = self.0.split_first_chunk::<4>()?;
        self.0 = rest;
        Some(u32::from_le_bytes(*head))
    }

    fn f64(&mut self) -> Option<f64> {
        let (head, rest) = self.0.split_first_chunk::<8>()?;
        self.0 = rest;
        Some(f64::from_le_bytes(*head))
    }

    fn skip(&mut self, n: usize) -> Option<()> {
        self.0 = self.0.get(n..)?;
        Some(())
    }

    fn string(&mut self, len: usize) -> Option<String> {
        let raw = self.0.get(..len)?;
        self.0 = &self.0[len..];
        let end = raw.iter().position(|&b| b == 0).unwrap_or(len);
        Some(raw[..end].iter().map(|&b| b as char).collect())
    }
}

/// Parses one complete message (header included).
pub fn parse(msg: &[u8]) -> Option<Message> {
    let mut r = Reader(msg);
    let _size = r.u32()?;
    let _protocol = r.u32()?;
    let id = r.u32()?;
    Some(match id {
        recv::OPEN => {
            r.skip(256 + 16)?; // application name and version
            Message::Open { sim_version: (r.u32()?, r.u32()?, r.u32()?, r.u32()?) }
        }
        recv::QUIT => Message::Quit,
        recv::EXCEPTION => Message::Exception { exception: r.u32()?, send_id: r.u32()?, index: r.u32()? },
        recv::EVENT | recv::EVENT_FILENAME | recv::EVENT_FRAME => {
            let _group = r.u32()?;
            let event = r.u32()?;
            let data = r.u32()?;
            match id {
                recv::EVENT => Message::Event { id: event, data },
                recv::EVENT_FILENAME => Message::EventFilename { id: event, filename: r.string(260)? },
                _ => Message::Frame { id: event },
            }
        }
        recv::SIMOBJECT_DATA => {
            let request_id = r.u32()?;
            r.skip(4 * 5)?; // object, define, flags, entry, out of
            let count = r.u32()? as usize;
            let mut values = Vec::with_capacity(count);
            while let Some(v) = r.f64() {
                values.push(v);
            }
            Message::SimObjectData { request_id, values }
        }
        recv::CAMERA_STATUS => Message::CameraStatus { acquired: r.u32()?, game_controlled: r.u32()? != 0 },
        other => Message::Other(other),
    })
}

/// A byte stream to the simulator that can be polled without blocking.
pub trait Transport: Write {
    /// Reads what's available now; `Ok(0)` when nothing is.
    fn poll_read(&mut self, buf: &mut [u8]) -> io::Result<usize>;
}

pub struct Client<T: Transport> {
    io: T,
    next_send_id: u32,
    inbox: Vec<u8>,
}

impl<T: Transport> Client<T> {
    /// Sends the handshake; the simulator answers with `Message::Open`.
    pub fn open(io: T, app_name: &str) -> io::Result<Self> {
        let mut c = Self { io, next_send_id: 1, inbox: Vec::new() };
        let (major, minor, build_major, build_minor) = OPEN_VERSION;
        let mut p = Packet::new(func::OPEN);
        p.str(app_name, 256).u32(0).u8(0);
        p.0.extend_from_slice(OPEN_ALIAS);
        p.0.resize(p.0.len() + 3 - OPEN_ALIAS.len(), 0);
        p.u32(major).u32(minor).u32(build_major).u32(build_minor);
        c.send(p)?;
        Ok(c)
    }

    /// Sends a packet; returns its send id (exceptions refer to it).
    fn send(&mut self, p: Packet) -> io::Result<u32> {
        let id = self.next_send_id;
        self.next_send_id = self.next_send_id.wrapping_add(1);
        self.io.write_all(&p.finish(id))?;
        Ok(id)
    }

    /// The next complete message, if one has arrived.
    pub fn poll(&mut self) -> io::Result<Option<Message>> {
        loop {
            if self.inbox.len() >= 4 {
                let size = u32::from_le_bytes(self.inbox[0..4].try_into().unwrap()) as usize;
                if size < 12 {
                    return Err(io::Error::new(io::ErrorKind::InvalidData, "bad SimConnect message size"));
                }
                if self.inbox.len() >= size {
                    let msg: Vec<u8> = self.inbox.drain(..size).collect();
                    return Ok(parse(&msg).or(Some(Message::Other(u32::MAX))));
                }
            }
            let mut buf = [0u8; 16384];
            let n = self.io.poll_read(&mut buf)?;
            if n == 0 {
                return Ok(None);
            }
            self.inbox.extend_from_slice(&buf[..n]);
        }
    }

    pub fn add_to_data_definition(&mut self, def: u32, name: &str, units: &str) -> io::Result<u32> {
        let mut p = Packet::new(func::ADD_TO_DATA_DEFINITION);
        p.u32(def).str(name, 256).str(units, 256).u32(DATATYPE_FLOAT64).f32(0.0).u32(UNUSED);
        self.send(p)
    }

    pub fn request_data_on_sim_object(&mut self, request: u32, def: u32, period: u32) -> io::Result<u32> {
        let mut p = Packet::new(func::REQUEST_DATA_ON_SIM_OBJECT);
        p.u32(request).u32(def).u32(OBJECT_ID_USER).u32(period).u32(0).u32(0).u32(0).u32(0);
        self.send(p)
    }

    pub fn subscribe_to_system_event(&mut self, id: u32, name: &str) -> io::Result<u32> {
        let mut p = Packet::new(func::SUBSCRIBE_TO_SYSTEM_EVENT);
        p.u32(id).str(name, 256);
        self.send(p)
    }

    pub fn subscribe_to_camera_status(&mut self) -> io::Result<u32> {
        self.send(Packet::new(func::SUBSCRIBE_TO_CAMERA_STATUS))
    }

    pub fn camera_acquire(&mut self, name: &str) -> io::Result<u32> {
        let mut p = Packet::new(func::CAMERA_ACQUIRE);
        p.str(name, 2048);
        self.send(p)
    }

    /// Gives the camera back; `camera` names the view to return to (empty:
    /// the simulator's choice).
    pub fn camera_release(&mut self, camera: &str) -> io::Result<u32> {
        let mut p = Packet::new(func::CAMERA_RELEASE);
        p.str(camera, 2048);
        self.send(p)
    }

    pub fn camera_enable_flag(&mut self, flag: u32) -> io::Result<u32> {
        let mut p = Packet::new(func::CAMERA_ENABLE_FLAG);
        p.u32(flag);
        self.send(p)
    }

    pub fn camera_set(&mut self, c: &CameraData, mask: u32) -> io::Result<u32> {
        self.send(camera_set_packet(c, mask))
    }
}

fn camera_set_packet(c: &CameraData, mask: u32) -> Packet {
    let mut p = Packet::new(func::CAMERA_SET);
    p.f64(c.position[0]).f64(c.position[1]).f64(c.position[2]);
    p.u32(c.position_referential).u32(c.position_object);
    p.f64(c.target[0]).f64(c.target[1]).f64(c.target[2]);
    p.f32(c.pbh[0]).f32(c.pbh[1]).f32(c.pbh[2]);
    p.u32(c.rotation_referential).u32(c.rotation_object);
    p.f64(c.fov);
    p.u32(mask);
    p
}

/// Named pipe (the default local connection on Windows).
#[cfg(windows)]
pub struct Pipe(std::fs::File);

#[cfg(windows)]
impl Pipe {
    pub const PATH: &'static str = r"\\.\pipe\Microsoft Flight Simulator\SimConnect";

    pub fn connect() -> io::Result<Self> {
        std::fs::OpenOptions::new().read(true).write(true).open(Self::PATH).map(Pipe)
    }
}

#[cfg(windows)]
impl Write for Pipe {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.0.write(buf)
    }

    fn flush(&mut self) -> io::Result<()> {
        self.0.flush()
    }
}

#[cfg(windows)]
impl Transport for Pipe {
    // Synchronous I/O on one pipe handle is serialized by Windows: a reader
    // blocked in ReadFile would stall our writes. So peek first and only
    // read what's there.
    fn poll_read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        use std::os::windows::io::AsRawHandle;
        #[link(name = "kernel32")]
        extern "system" {
            fn PeekNamedPipe(
                pipe: *mut core::ffi::c_void,
                buffer: *mut core::ffi::c_void,
                buffer_size: u32,
                bytes_read: *mut u32,
                total_available: *mut u32,
                bytes_left_this_message: *mut u32,
            ) -> i32;
        }
        let mut available = 0u32;
        // SAFETY: a valid pipe handle owned by `self.0`; only `available` is written.
        let ok = unsafe {
            PeekNamedPipe(
                self.0.as_raw_handle(),
                std::ptr::null_mut(),
                0,
                std::ptr::null_mut(),
                &mut available,
                std::ptr::null_mut(),
            )
        };
        if ok == 0 {
            return Err(io::Error::last_os_error());
        }
        if available == 0 {
            return Ok(0);
        }
        let n = (available as usize).min(buf.len());
        match self.0.read(&mut buf[..n])? {
            0 => Err(io::ErrorKind::UnexpectedEof.into()),
            n => Ok(n),
        }
    }
}

/// TCP, for a simulator configured to listen on IPv4 (`SimConnect.xml`), or
/// one on another machine.
pub struct Tcp(std::net::TcpStream);

impl Tcp {
    pub fn connect(addr: &str) -> io::Result<Self> {
        use std::net::ToSocketAddrs;
        let addr = addr
            .to_socket_addrs()?
            .next()
            .ok_or_else(|| io::Error::new(io::ErrorKind::NotFound, format!("cannot resolve {addr}")))?;
        let s = std::net::TcpStream::connect_timeout(&addr, std::time::Duration::from_secs(2))?;
        s.set_nodelay(true)?;
        s.set_nonblocking(true)?;
        Ok(Tcp(s))
    }
}

impl Write for Tcp {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        loop {
            match self.0.write(buf) {
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => std::thread::yield_now(),
                r => return r,
            }
        }
    }

    fn flush(&mut self) -> io::Result<()> {
        self.0.flush()
    }
}

impl Transport for Tcp {
    fn poll_read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        match self.0.read(buf) {
            Ok(0) => Err(io::ErrorKind::UnexpectedEof.into()),
            Ok(n) => Ok(n),
            Err(e) if e.kind() == io::ErrorKind::WouldBlock => Ok(0),
            Err(e) => Err(e),
        }
    }
}

/// Either connection, so the worker can hold one type.
pub enum AnyTransport {
    #[cfg(windows)]
    Pipe(Pipe),
    Tcp(Tcp),
}

impl Write for AnyTransport {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        match self {
            #[cfg(windows)]
            AnyTransport::Pipe(p) => p.write(buf),
            AnyTransport::Tcp(t) => t.write(buf),
        }
    }

    fn flush(&mut self) -> io::Result<()> {
        match self {
            #[cfg(windows)]
            AnyTransport::Pipe(p) => p.flush(),
            AnyTransport::Tcp(t) => t.flush(),
        }
    }
}

impl Transport for AnyTransport {
    fn poll_read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        match self {
            #[cfg(windows)]
            AnyTransport::Pipe(p) => p.poll_read(buf),
            AnyTransport::Tcp(t) => t.poll_read(buf),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;

    /// Records what's written; serves canned bytes, a few at a time.
    #[derive(Default)]
    struct Fake {
        sent: Vec<u8>,
        incoming: VecDeque<u8>,
    }

    impl Write for Fake {
        fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
            self.sent.extend_from_slice(buf);
            Ok(buf.len())
        }
        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }

    impl Transport for Fake {
        fn poll_read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
            let n = buf.len().min(self.incoming.len()).min(7);
            for b in buf.iter_mut().take(n) {
                *b = self.incoming.pop_front().unwrap();
            }
            Ok(n)
        }
    }

    fn le(words: &[u32]) -> Vec<u8> {
        words.iter().flat_map(|w| w.to_le_bytes()).collect()
    }

    #[test]
    fn open_packet_layout() {
        let c = Client::open(Fake::default(), "Camship").unwrap();
        let p = &c.io.sent;
        assert_eq!(p.len(), 16 + 256 + 4 + 1 + 3 + 16);
        assert_eq!(u32::from_le_bytes(p[0..4].try_into().unwrap()) as usize, p.len());
        assert_eq!(&p[4..8], &PROTOCOL.to_le_bytes());
        assert_eq!(&p[8..12], &0xF000_0001u32.to_le_bytes());
        assert_eq!(&p[16..23], b"Camship");
        assert_eq!(&p[16 + 256 + 5..16 + 256 + 8], b"RS\0");
        assert_eq!(&p[p.len() - 16..p.len() - 12], &12u32.to_le_bytes());
    }

    #[test]
    fn camera_set_layout() {
        let c = CameraData { fov: 1.0, ..Default::default() };
        let p = camera_set_packet(&c, 27).finish(9);
        // header, 3 f64, 2 u32, 3 f64, 3 f32, 2 u32, f64, mask
        assert_eq!(p.len(), 16 + 24 + 8 + 24 + 12 + 8 + 8 + 4);
        assert_eq!(&p[8..12], &0xF000_0062u32.to_le_bytes());
        assert_eq!(&p[12..16], &9u32.to_le_bytes());
        assert_eq!(&p[p.len() - 4..], &27u32.to_le_bytes());
        assert_eq!(&p[p.len() - 12..p.len() - 4], &1.0f64.to_le_bytes());
    }

    #[test]
    fn reassembles_split_messages() {
        let mut data = le(&[0, PROTOCOL, recv::SIMOBJECT_DATA, 5, 0, 1, 0, 1, 1, 2]);
        data.extend_from_slice(&1.5f64.to_le_bytes());
        data.extend_from_slice(&(-2.0f64).to_le_bytes());
        let size = data.len() as u32;
        data[0..4].copy_from_slice(&size.to_le_bytes());
        let mut ev = le(&[0, PROTOCOL, recv::EVENT, 0, 3, 1]);
        let ev_size = ev.len() as u32;
        ev[0..4].copy_from_slice(&ev_size.to_le_bytes());

        let mut fake = Fake::default();
        fake.incoming.extend(data.iter().chain(ev.iter()));
        let mut c = Client { io: fake, next_send_id: 1, inbox: Vec::new() };
        assert_eq!(c.poll().unwrap(), Some(Message::SimObjectData { request_id: 5, values: vec![1.5, -2.0] }));
        assert_eq!(c.poll().unwrap(), Some(Message::Event { id: 3, data: 1 }));
        assert_eq!(c.poll().unwrap(), None);
    }
}
