use deku::prelude::*;
use embedded_can::{Id, StandardId};

use crate::sdo::ConfigValue;

#[derive(Debug, Clone, DekuRead, DekuWrite, PartialEq)]
#[deku(id_type = "u8", bits = 5)]
pub enum CanMessage {
    #[deku(id = "0x00")]
    GetVersion {
        protocol_version: u8,
        hw_version_major: u8,
        hw_version_minor: u8,
        hw_version_variant: u8,
        fw_version_major: u8,
        fw_version_minor: u8,
        fw_version_revision: u8,
        fw_version_unreleased: u8,
    },
    #[deku(id = "0x01")]
    Heartbeat {
        #[deku(endian = "little")]
        axis_error: u32,
        axis_state: AxisState,
        procedure_result: u8,
        trajectory_done_flag: u8,
        _reserved: u8,
    },
    #[deku(id = "0x02")]
    Estop,
    #[deku(id = "0x03")]
    GetError {
        #[deku(endian = "little")]
        active_errors: u32,
        #[deku(endian = "little")]
        disarm_reason: u32,
    },
    #[deku(id = "0x04")]
    RxSdo {
        opcode: SdoOpcode,
        #[deku(endian = "little")]
        endpoint_id: u16,
        reserved: u8,
        #[deku(read_all)]
        value: Vec<u8>,
    },
    #[deku(id = "0x05")]
    TxSdo {
        reserved0: u8,
        #[deku(endian = "little")]
        endpoint_id: u16,
        reserved1: u8,
        #[deku(read_all)]
        value: Vec<u8>,
    },
    #[deku(id = "0x06")]
    Address {
        node_id: u8,
        #[deku(bytes = "6")]
        serial_number: [u8; 6],
        connection_id: u8,
    },
    #[deku(id = "0x07")]
    SetAxisState {
        #[deku(pad_bytes_after = "3")]
        // for some reason, in SetAxisState, the axis state is supposed to be 4 bytes
        axis_requested_state: AxisState,
    },
    #[deku(id = "0x09")]
    GetEncoderEstimates {
        /// Unit: rev
        #[deku(endian = "little")]
        pos_estimate: f32,
        /// Unit: rev/s
        #[deku(endian = "little")]
        vel_estimate: f32,
    },
    #[deku(id = "0x0B")]
    SetControllerMode {
        #[deku(pad_bytes_after = "3")]
        control_mode: ControlMode,
        #[deku(pad_bytes_after = "3")]
        input_mode: InputMode,
    },
    #[deku(id = "0x0C")]
    SetInputPos {
        /// Unit: rev
        #[deku(endian = "little")]
        input_pos: f32,
        /// Unit: 0.001 rev/s (default, configurable)
        #[deku(endian = "little")]
        vel_ff: i16,
        /// Unit: 0.001 Nm (default, configurable)
        #[deku(endian = "little")]
        torque_ff: i16,
    },
    #[deku(id = "0x0D")]
    SetInputVel {
        /// Unit: rev/s
        #[deku(endian = "little")]
        input_vel: f32,
        /// Unit: Nm
        #[deku(endian = "little")]
        input_torque_ff: f32,
    },
    #[deku(id = "0x0E")]
    SetInputTorque {
        /// Unit: Nm
        #[deku(endian = "little")]
        input_torque: f32,
    },
    #[deku(id = "0x0F")]
    SetLimits {
        /// Unit: rev/s
        #[deku(endian = "little")]
        velocity_limit: f32,
        /// Unit: A
        #[deku(endian = "little")]
        current_limit: f32,
    },
    #[deku(id = "0x11")]
    SetTrajVelLimit {
        /// Unit: rev/s
        #[deku(endian = "little")]
        traj_vel_limit: f32,
    },
    #[deku(id = "0x12")]
    SetTrajAccelLimits {
        /// Unit: rev/s²
        #[deku(endian = "little")]
        traj_accel_limit: f32,
        /// Unit: rev/s²
        #[deku(endian = "little")]
        traj_decel_limit: f32,
    },
    #[deku(id = "0x13")]
    SetTrajInertia {
        /// Unit: Nm/(rev/s²)
        #[deku(endian = "little")]
        traj_inertia: f32,
    },
    #[deku(id = "0x14")]
    GetIq {
        /// Unit: A
        #[deku(endian = "little")]
        iq_setpoint: f32,
        /// Unit: A
        #[deku(endian = "little")]
        iq_measured: f32,
    },
    #[deku(id = "0x15")]
    GetTemperature {
        /// Unit: °C
        #[deku(endian = "little")]
        fet_temperature: f32,
        /// Unit: °C
        #[deku(endian = "little")]
        motor_temperature: f32,
    },
    #[deku(id = "0x16")]
    Reboot { action: RebootAction },
    #[deku(id = "0x17")]
    GetBusVoltageCurrent {
        /// Unit: V
        #[deku(endian = "little")]
        bus_voltage: f32,
        /// Unit: A
        #[deku(endian = "little")]
        bus_current: f32,
    },
    #[deku(id = "0x18")]
    ClearErrors {
        #[deku(bits = 8)]
        identify: bool,
    },
    #[deku(id = "0x19")]
    SetAbsolutePosition {
        /// Unit: rev
        #[deku(endian = "little")]
        position: f32,
    },
    #[deku(id = "0x1A")]
    SetPosGain {
        /// Unit: (rev/s)/rev
        #[deku(endian = "little")]
        pos_gain: f32,
    },
    #[deku(id = "0x1B")]
    SetVelGains {
        /// Unit: Nm/(rev/s)
        #[deku(endian = "little")]
        vel_gain: f32,
        /// Unit: Nm/rev
        #[deku(endian = "little")]
        vel_integrator_gain: f32,
    },
    #[deku(id = "0x1C")]
    GetTorques {
        /// Unit: Nm
        #[deku(endian = "little")]
        torque_target: f32,
        /// Unit: Nm
        #[deku(endian = "little")]
        torque_estimate: f32,
    },
    #[deku(id = "0x1D")]
    GetPowers {
        /// Unit: W
        #[deku(endian = "little")]
        electrical_power: f32,
        /// Unit: W
        #[deku(endian = "little")]
        mechanical_power: f32,
    },
    #[deku(id = "0x1F")]
    EnterDfuMode,
}

#[derive(Debug, Clone, DekuRead, DekuWrite, PartialEq)]
pub struct CanMessageWithId {
    #[deku(bits = 6, pad_bits_before = "5")]
    // since the id is actually separate from the data in socketcan, a bit of a hack to let deku handle it
    pub node_id: u8,
    pub message: CanMessage,
}

impl CanMessageWithId {
    pub fn id(&self) -> StandardId {
        let bytes: Vec<u8> = self.to_bytes().unwrap();
        Self::id_from_bytes(&bytes)
    }

    pub fn data(&self) -> Vec<u8> {
        let bytes: Vec<u8> = self.to_bytes().unwrap();
        Self::data_from_bytes(&bytes).to_vec()
    }

    fn id_from_bytes(bytes: &[u8]) -> StandardId {
        StandardId::new(u16::from_be_bytes(bytes[..2].try_into().unwrap())).unwrap()
    }

    fn data_from_bytes(bytes: &[u8]) -> &[u8] {
        &bytes[2..]
    }

    pub fn from_frame<F: embedded_can::Frame>(frame: &F) -> Result<Self, DekuError> {
        let id = match frame.id() {
            Id::Standard(std_id) => std_id.as_raw(),
            _ => unreachable!(), // ODrive uses standard IDs only
        };
        let data = frame.data();

        Self::from_id_and_data(id, data)
    }

    pub fn from_id_and_data(id: u16, data: &[u8]) -> Result<Self, DekuError> {
        // Prepend node_id and cmd_id to the data
        let full_data = [&id.to_be_bytes(), data].concat();

        // Parse using deku
        let (_, message_with_id) = CanMessageWithId::from_bytes((&full_data, 0))?;

        Ok(message_with_id)
    }

    pub fn to_frame<F: embedded_can::Frame>(self) -> F {
        let bytes = self.to_bytes().unwrap();
        F::new(Self::id_from_bytes(&bytes), Self::data_from_bytes(&bytes)).unwrap()
    }

    pub fn to_rtr_frame<F: embedded_can::Frame>(self) -> F {
        F::new_remote(self.id(), 0).unwrap()
    }

    pub fn get_version(node_id: u8) -> Self {
        CanMessageWithId {
            node_id,
            message: CanMessage::GetVersion {
                protocol_version: 0,
                hw_version_major: 0,
                hw_version_minor: 0,
                hw_version_variant: 0,
                fw_version_major: 0,
                fw_version_minor: 0,
                fw_version_revision: 0,
                fw_version_unreleased: 0,
            },
        }
    }

    pub fn heartbeat(node_id: u8) -> Self {
        CanMessageWithId {
            node_id,
            message: CanMessage::Heartbeat {
                axis_error: 0,
                axis_state: AxisState::Undefined,
                procedure_result: 0,
                trajectory_done_flag: 0,
                _reserved: 0,
            },
        }
    }

    pub fn estop(node_id: u8) -> Self {
        CanMessageWithId {
            node_id,
            message: CanMessage::Estop,
        }
    }

    pub fn get_error(node_id: u8) -> Self {
        CanMessageWithId {
            node_id,
            message: CanMessage::GetError {
                active_errors: 0,
                disarm_reason: 0,
            },
        }
    }

    pub fn rx_sdo(node_id: u8, opcode: SdoOpcode, endpoint_id: u16, value: ConfigValue) -> Self {
        CanMessageWithId {
            node_id,
            message: CanMessage::RxSdo {
                opcode,
                endpoint_id,
                reserved: 0,
                value: value.to_le_byte_vec(),
            },
        }
    }

    pub fn set_address(
        node_id: u8,
        set_node_id: u8,
        serial_number: [u8; 6],
        connection_id: u8,
    ) -> Self {
        CanMessageWithId {
            node_id,
            message: CanMessage::Address {
                node_id: set_node_id,
                serial_number,
                connection_id,
            },
        }
    }

    pub fn get_address(node_id: u8) -> Self {
        // should be sent as rtr frame
        CanMessageWithId {
            node_id,
            message: CanMessage::Address {
                node_id: 0,
                serial_number: [0; 6],
                connection_id: 0,
            },
        }
    }

    pub fn get_all_addresses() -> Self {
        // should be sent as rtr frame
        CanMessageWithId {
            node_id: 0x3f,
            message: CanMessage::Address {
                node_id: 0,
                serial_number: [0; 6],
                connection_id: 0,
            },
        }
    }

    pub fn set_axis_state(node_id: u8, state: AxisState) -> Self {
        CanMessageWithId {
            node_id,
            message: CanMessage::SetAxisState {
                axis_requested_state: state,
            },
        }
    }

    pub fn get_encoder_estimates(node_id: u8) -> Self {
        CanMessageWithId {
            node_id,
            message: CanMessage::GetEncoderEstimates {
                pos_estimate: 0.0,
                vel_estimate: 0.0,
            },
        }
    }

    pub fn set_controller_mode(
        node_id: u8,
        control_mode: ControlMode,
        input_mode: InputMode,
    ) -> Self {
        CanMessageWithId {
            node_id,
            message: CanMessage::SetControllerMode {
                control_mode,
                input_mode,
            },
        }
    }

    pub fn set_input_pos(node_id: u8, input_pos: f32, vel_ff: i16, torque_ff: i16) -> Self {
        CanMessageWithId {
            node_id,
            message: CanMessage::SetInputPos {
                input_pos,
                vel_ff,
                torque_ff,
            },
        }
    }

    pub fn set_input_vel(node_id: u8, input_vel: f32, input_torque_ff: f32) -> Self {
        CanMessageWithId {
            node_id,
            message: CanMessage::SetInputVel {
                input_vel,
                input_torque_ff,
            },
        }
    }

    pub fn set_input_torque(node_id: u8, input_torque: f32) -> Self {
        CanMessageWithId {
            node_id,
            message: CanMessage::SetInputTorque { input_torque },
        }
    }

    pub fn set_limits(node_id: u8, velocity_limit: f32, current_limit: f32) -> Self {
        CanMessageWithId {
            node_id,
            message: CanMessage::SetLimits {
                velocity_limit,
                current_limit,
            },
        }
    }

    pub fn set_traj_vel_limit(node_id: u8, traj_vel_limit: f32) -> Self {
        CanMessageWithId {
            node_id,
            message: CanMessage::SetTrajVelLimit { traj_vel_limit },
        }
    }

    pub fn set_traj_accel_limits(
        node_id: u8,
        traj_accel_limit: f32,
        traj_decel_limit: f32,
    ) -> Self {
        CanMessageWithId {
            node_id,
            message: CanMessage::SetTrajAccelLimits {
                traj_accel_limit,
                traj_decel_limit,
            },
        }
    }

    pub fn set_traj_inertia(node_id: u8, traj_inertia: f32) -> Self {
        CanMessageWithId {
            node_id,
            message: CanMessage::SetTrajInertia { traj_inertia },
        }
    }

    pub fn get_iq(node_id: u8) -> Self {
        CanMessageWithId {
            node_id,
            message: CanMessage::GetIq {
                iq_setpoint: 0.0,
                iq_measured: 0.0,
            },
        }
    }

    pub fn get_temperature(node_id: u8) -> Self {
        CanMessageWithId {
            node_id,
            message: CanMessage::GetTemperature {
                fet_temperature: 0.0,
                motor_temperature: 0.0,
            },
        }
    }

    pub fn reboot(node_id: u8, action: RebootAction) -> Self {
        CanMessageWithId {
            node_id,
            message: CanMessage::Reboot { action },
        }
    }

    pub fn get_bus_voltage_current(node_id: u8) -> Self {
        CanMessageWithId {
            node_id,
            message: CanMessage::GetBusVoltageCurrent {
                bus_voltage: 0.0,
                bus_current: 0.0,
            },
        }
    }

    pub fn clear_errors(node_id: u8, identify: bool) -> Self {
        CanMessageWithId {
            node_id,
            message: CanMessage::ClearErrors { identify },
        }
    }

    pub fn set_absolute_position(node_id: u8, position: f32) -> Self {
        CanMessageWithId {
            node_id,
            message: CanMessage::SetAbsolutePosition { position },
        }
    }

    pub fn set_pos_gain(node_id: u8, pos_gain: f32) -> Self {
        CanMessageWithId {
            node_id,
            message: CanMessage::SetPosGain { pos_gain },
        }
    }

    pub fn set_vel_gains(node_id: u8, vel_gain: f32, vel_integrator_gain: f32) -> Self {
        CanMessageWithId {
            node_id,
            message: CanMessage::SetVelGains {
                vel_gain,
                vel_integrator_gain,
            },
        }
    }

    pub fn get_torques(node_id: u8) -> Self {
        CanMessageWithId {
            node_id,
            message: CanMessage::GetTorques {
                torque_target: 0.0,
                torque_estimate: 0.0,
            },
        }
    }

    pub fn get_powers(node_id: u8) -> Self {
        CanMessageWithId {
            node_id,
            message: CanMessage::GetPowers {
                electrical_power: 0.0,
                mechanical_power: 0.0,
            },
        }
    }

    pub fn enter_dfu_mode(node_id: u8) -> Self {
        CanMessageWithId {
            node_id,
            message: CanMessage::EnterDfuMode,
        }
    }
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, DekuWrite, DekuRead)]
#[deku(id_type = "u8")]
pub enum AxisState {
    Undefined = 0,
    Idle = 1,
    StartupSequence = 2,
    FullCalibration = 3,
    MotorCalibration = 4,
    EncoderIndexSearch = 6,
    EncoderOffsetCalibration = 7,
    ClosedLoopControl = 8,
    LockinSpin = 9,
    EncoderDirFind = 10,
    Homing = 11,
    EncoderHallPolarityCalibration = 12,
    EncoderHallPhaseCalibration = 13,
    AnticoggingCalibration = 14,
    HarmonicCalibration = 15,
    HarmonicCalibrationCommutation = 16,
}

impl std::fmt::Display for AxisState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            AxisState::Undefined => write!(f, "UNDEFINED"),
            AxisState::Idle => write!(f, "IDLE"),
            AxisState::StartupSequence => write!(f, "STARTUP_SEQUENCE"),
            AxisState::FullCalibration => write!(f, "FULL_CALIBRATION"),
            AxisState::MotorCalibration => write!(f, "MOTOR_CALIBRATION"),
            AxisState::EncoderIndexSearch => write!(f, "ENCODER_INDEX_SEARCH"),
            AxisState::EncoderOffsetCalibration => write!(f, "ENCODER_OFFSET_CALIBRATION"),
            AxisState::ClosedLoopControl => write!(f, "CLOSED_LOOP_CONTROL"),
            AxisState::LockinSpin => write!(f, "LOCKIN_SPIN"),
            AxisState::EncoderDirFind => write!(f, "ENCODER_DIR_FIND"),
            AxisState::Homing => write!(f, "HOMING"),
            AxisState::EncoderHallPolarityCalibration => {
                write!(f, "ENCODER_HALL_POLARITY_CALIBRATION")
            }
            AxisState::EncoderHallPhaseCalibration => write!(f, "ENCODER_HALL_PHASE_CALIBRATION"),
            AxisState::AnticoggingCalibration => write!(f, "ANTICOGGING_CALIBRATION"),
            AxisState::HarmonicCalibration => write!(f, "HARMONIC_CALIBRATION"),
            AxisState::HarmonicCalibrationCommutation => {
                write!(f, "HARMONIC_CALIBRATION_COMMUTATION")
            }
        }
    }
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, DekuWrite, DekuRead)]
#[deku(id_type = "u8")]
pub enum RebootAction {
    Reboot = 0,
    SaveConfiguration = 1,
    EraseConfiguration = 2,
    EnterDfuMode2 = 3,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, DekuWrite, DekuRead)]
#[deku(id_type = "u8")]
pub enum ControlMode {
    VoltageControl = 0,
    TorqueControl = 1,
    VelocityControl = 2,
    PositionControl = 3,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, DekuWrite, DekuRead)]
#[deku(id_type = "u8")]
pub enum InputMode {
    Inactive = 0,
    Passthrough = 1,
    VelRamp = 2,
    PosFilter = 3,
    MixChannels = 4,
    TrapTraj = 5,
    TorqueRamp = 6,
    Mirror = 7,
    Tuning = 8,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, DekuWrite, DekuRead)]
#[deku(id_type = "u8", endian = "little")]
pub enum SdoOpcode {
    Read = 0,
    Write = 1,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    pub fn test_set_closed_loop_control() {
        let msg = CanMessageWithId::set_axis_state(1, AxisState::ClosedLoopControl);
        let id = msg.id().as_raw();
        let data = msg.data();
        assert_eq!(id, 1 << 5 | 0x07);
        assert_eq!(data, vec![0x08, 0x00, 0x00, 0x00]);
        assert_eq!(msg, CanMessageWithId::from_id_and_data(id, &data).unwrap());
    }

    #[test]
    pub fn test_docs_example_read_vel_integrator_limit() {
        let msg = CanMessageWithId::rx_sdo(0, SdoOpcode::Read, 0x0182, ConfigValue::Empty);
        let id = msg.id().as_raw();
        let data = msg.data();
        assert_eq!(id, 0x04);
        assert_eq!(data, vec![0x00, 0x82, 0x01, 0x00]); // read has no data
        assert_eq!(msg, CanMessageWithId::from_id_and_data(id, &data).unwrap());
    }

    #[test]
    pub fn test_docs_example_read_vel_integrator_limit_response() {
        let resp_id = 0x05;
        // Raw messages, assuming node_id = 0, endpoint_id = 0x0182, return_value = inf:
        let resp_data = vec![0x00, 0x82, 0x01, 0x00, 0x00, 0x00, 0x80, 0x7f];
        let resp_msg = CanMessageWithId::from_id_and_data(resp_id, &resp_data).unwrap();
        match resp_msg.message {
            CanMessage::TxSdo {
                endpoint_id, value, ..
            } => {
                assert_eq!(endpoint_id, 0x0182);
                let config_value = ConfigValue::from_le_bytes(&value, "float").unwrap();
                match config_value {
                    ConfigValue::Float(f) => {
                        assert!(f.is_infinite() && f.is_sign_positive());
                    }
                    _ => panic!("Expected float ConfigValue"),
                }
            }
            _ => panic!("Expected TxSdo message"),
        }
    }

    #[test]
    pub fn test_docs_example_function_call() {
        let msg = CanMessageWithId::rx_sdo(0, SdoOpcode::Write, 0x0253, ConfigValue::List(vec![]));
        let id = msg.id().as_raw();
        let data = msg.data();
        assert_eq!(id, 0x04);
        assert_eq!(data, vec![0x01, 0x53, 0x02, 0x00]);
        assert_eq!(msg, CanMessageWithId::from_id_and_data(id, &data).unwrap());
    }

    #[test]
    pub fn test_docs_example_write_vel_integrator_limit() {
        let msg = CanMessageWithId::rx_sdo(0, SdoOpcode::Write, 0x0182, ConfigValue::Float(1.234));
        let id = msg.id().as_raw();
        let data = msg.data();
        assert_eq!(id, 0x04);
        assert_eq!(data, vec![0x01, 0x82, 0x01, 0x00, 0xb6, 0xf3, 0x9d, 0x3f]); // the docs have an error here, the first byte should be 0x01 for write
        assert_eq!(msg, CanMessageWithId::from_id_and_data(id, &data).unwrap());
    }

    #[test]
    pub fn test_set_control_mode() {
        let msg = CanMessageWithId::set_controller_mode(
            2,
            ControlMode::TorqueControl,
            InputMode::Passthrough,
        );
        let id = msg.id().as_raw();
        let data = msg.data();
        assert_eq!(id, 2 << 5 | 0x0B);
        assert_eq!(data, vec![0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00])
    }
}
