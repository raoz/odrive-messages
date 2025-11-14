use deku::DekuError;

use crate::{
    messages::{AxisState, CanMessageWithId, ControlMode, InputMode, RebootAction},
    sdo::ConfigValue,
};

pub mod messages;
pub mod sdo;

pub const UNADDRESSED_NODE_ID: u8 = 0x3f;

pub fn parse_frame<F: embedded_can::Frame>(frame: &F) -> Result<CanMessageWithId, DekuError> {
    CanMessageWithId::from_frame(frame)
}

pub fn estop<F: embedded_can::Frame>(node_id: u8) -> F {
    CanMessageWithId::estop(node_id).to_frame()
}

/// Reboots the ODrive node with the specified reboot action.
pub fn reboot<F: embedded_can::Frame>(node_id: u8, reboot_action: RebootAction) -> F {
    CanMessageWithId::reboot(node_id, reboot_action).to_frame()
}

/// Equivalent to calling `clear_errors()`. If `identify` is true, the
/// node will flash its LED to help identify it on the bus.
pub fn clear_errors<F: embedded_can::Frame>(node_id: u8, identify: bool) -> F {
    CanMessageWithId::clear_errors(node_id, identify).to_frame()
}

pub fn query_version<F: embedded_can::Frame>(node_id: u8) -> F {
    CanMessageWithId::get_version(node_id).to_rtr_frame()
}

pub fn query_heartbeat<F: embedded_can::Frame>(node_id: u8) -> F {
    CanMessageWithId::heartbeat(node_id).to_rtr_frame()
}

pub fn query_error<F: embedded_can::Frame>(node_id: u8) -> F {
    CanMessageWithId::get_error(node_id).to_rtr_frame()
}

pub fn query_address<F: embedded_can::Frame>(node_id: u8) -> F {
    CanMessageWithId::get_address(node_id).to_rtr_frame()
}

pub fn query_all_addresses<F: embedded_can::Frame>() -> F {
    CanMessageWithId::get_all_addresses().to_rtr_frame()
}

pub fn query_encoder_estimates<F: embedded_can::Frame>(node_id: u8) -> F {
    CanMessageWithId::get_encoder_estimates(node_id).to_rtr_frame()
}

pub fn query_iq<F: embedded_can::Frame>(node_id: u8) -> F {
    CanMessageWithId::get_iq(node_id).to_rtr_frame()
}

pub fn query_temperature<F: embedded_can::Frame>(node_id: u8) -> F {
    CanMessageWithId::get_temperature(node_id).to_rtr_frame()
}

pub fn query_bus_voltage_current<F: embedded_can::Frame>(node_id: u8) -> F {
    CanMessageWithId::get_bus_voltage_current(node_id).to_rtr_frame()
}

pub fn query_torques<F: embedded_can::Frame>(node_id: u8) -> F {
    CanMessageWithId::get_torques(node_id).to_rtr_frame()
}

pub fn query_powers<F: embedded_can::Frame>(node_id: u8) -> F {
    CanMessageWithId::get_powers(node_id).to_rtr_frame()
}

pub fn set_address_by_serial_number<F: embedded_can::Frame>(
    serial_number: [u8; 6],
    new_address: u8,
) -> F {
    CanMessageWithId::set_address(UNADDRESSED_NODE_ID, new_address, serial_number, 0).to_frame()
}

pub fn set_axis_state<F: embedded_can::Frame>(node_id: u8, state: AxisState) -> F {
    CanMessageWithId::set_axis_state(node_id, state).to_frame()
}

pub fn set_controller_mode<F: embedded_can::Frame>(
    node_id: u8,
    control_mode: ControlMode,
    input_mode: InputMode,
) -> F {
    CanMessageWithId::set_controller_mode(node_id, control_mode, input_mode).to_frame()
}

/// # Units
/// position: rev
/// vel_feedforward: 0.001 rev/s (default, configurable)
/// torque_feedforward: 0.001 Nm (default, configurable)
pub fn set_input_pos<F: embedded_can::Frame>(
    node_id: u8,
    position: f32,
    vel_feedforward: i16,
    torque_feedforward: i16,
) -> F {
    CanMessageWithId::set_input_pos(node_id, position, vel_feedforward, torque_feedforward)
        .to_frame()
}

/// # Units
/// velocity: rev/s
/// torque_feedforward: Nm
pub fn set_input_vel<F: embedded_can::Frame>(
    node_id: u8,
    velocity: f32,
    torque_feedforward: f32,
) -> F {
    CanMessageWithId::set_input_vel(node_id, velocity, torque_feedforward).to_frame()
}

/// # Units
/// torque: Nm
pub fn set_input_torque<F: embedded_can::Frame>(node_id: u8, torque: f32) -> F {
    CanMessageWithId::set_input_torque(node_id, torque).to_frame()
}

/// # Units
/// velocity_limit: rev/s
/// current_limit: A
pub fn set_limits<F: embedded_can::Frame>(
    node_id: u8,
    velocity_limit: f32,
    current_limit: f32,
) -> F {
    CanMessageWithId::set_limits(node_id, velocity_limit, current_limit).to_frame()
}

/// # Units
/// traj_vel_limit: rev/s
pub fn set_traj_vel_limit<F: embedded_can::Frame>(node_id: u8, traj_vel_limit: f32) -> F {
    CanMessageWithId::set_traj_vel_limit(node_id, traj_vel_limit).to_frame()
}

/// # Units
/// traj_accel_limit: rev/s²
/// traj_decel_limit: rev/s²
pub fn set_traj_accel_limits<F: embedded_can::Frame>(
    node_id: u8,
    traj_accel_limit: f32,
    traj_decel_limit: f32,
) -> F {
    CanMessageWithId::set_traj_accel_limits(node_id, traj_accel_limit, traj_decel_limit).to_frame()
}

/// # Units
/// traj_inertia: Nm/(rev/s²)
pub fn set_traj_inertia<F: embedded_can::Frame>(node_id: u8, traj_inertia: f32) -> F {
    CanMessageWithId::set_traj_inertia(node_id, traj_inertia).to_frame()
}

/// # Units
/// position: rev
pub fn set_absolute_position<F: embedded_can::Frame>(node_id: u8, position: f32) -> F {
    CanMessageWithId::set_absolute_position(node_id, position).to_frame()
}

/// # Units
/// pos_gain: (rev/s)/rev
pub fn set_pos_gain<F: embedded_can::Frame>(node_id: u8, pos_gain: f32) -> F {
    CanMessageWithId::set_pos_gain(node_id, pos_gain).to_frame()
}

/// # Units
/// vel_gain: Nm/(rev/s)
/// vel_integrator_gain: Nm/rev
pub fn set_vel_gains<F: embedded_can::Frame>(
    node_id: u8,
    vel_gain: f32,
    vel_integrator_gain: f32,
) -> F {
    CanMessageWithId::set_vel_gains(node_id, vel_gain, vel_integrator_gain).to_frame()
}

pub fn read_config<F: embedded_can::Frame>(node_id: u8, endpoint_id: u16) -> F {
    CanMessageWithId::rx_sdo(
        node_id,
        messages::SdoOpcode::Read,
        endpoint_id,
        ConfigValue::Empty,
    )
    .to_frame()
}

pub fn write_config<F: embedded_can::Frame>(
    node_id: u8,
    endpoint_id: u16,
    value: ConfigValue,
) -> F {
    CanMessageWithId::rx_sdo(node_id, messages::SdoOpcode::Write, endpoint_id, value).to_frame()
}

/// Puts the ODrive into DFU mode for firmware updates. Equivalent to calling `enter_dfu_mode2()`.
pub fn enter_dfu_mode<F: embedded_can::Frame>(node_id: u8) -> F {
    CanMessageWithId::enter_dfu_mode(node_id).to_frame()
}
