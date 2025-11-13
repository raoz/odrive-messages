#[derive(Debug, Clone)]
pub enum ConfigValue {
    Float(f32),
    UInt8(u8),
    UInt32(u32),
    UInt64(u64),
    Int32(i32),
    Bool(bool),
    List(Vec<ConfigValue>),
    Empty,
}

impl ConfigValue {
    pub fn to_le_byte_vec(&self) -> Vec<u8> {
        match self {
            ConfigValue::Float(f) => f.to_le_bytes().to_vec(),
            ConfigValue::UInt8(v) => vec![*v],
            ConfigValue::UInt32(v) => v.to_le_bytes().to_vec(),
            ConfigValue::UInt64(v) => v.to_le_bytes().to_vec(),
            ConfigValue::Int32(v) => v.to_le_bytes().to_vec(),
            ConfigValue::Bool(b) => {
                if *b {
                    vec![1]
                } else {
                    vec![0]
                }
            }
            ConfigValue::List(v) => v.iter().flat_map(|item| item.to_le_byte_vec()).collect(),
            ConfigValue::Empty => vec![],
        }
    }

    // Parse from u32 based on type string
    pub fn from_le_bytes(value: &[u8], type_str: &str) -> Result<Self, String> {
        match type_str {
            "float" => Ok(ConfigValue::Float(f32::from_le_bytes(
                value.try_into().map_err(|e| format!("{e}"))?,
            ))),
            "uint8" => Ok(ConfigValue::UInt8(value[0])),
            "uint32" => Ok(ConfigValue::UInt32(u32::from_le_bytes(
                value.try_into().map_err(|e| format!("{e}"))?,
            ))),
            "uint64" => Ok(ConfigValue::UInt64(u64::from_le_bytes(
                value.try_into().map_err(|e| format!("{e}"))?,
            ))),
            "int32" => Ok(ConfigValue::Int32(i32::from_le_bytes(
                value.try_into().map_err(|e| format!("{e}"))?,
            ))),
            "bool" => Ok(ConfigValue::Bool(value[0] != 0)),
            // TODO: implement list parsing
            _ => Err(format!("Unknown type: {}", type_str)),
        }
    }
}
