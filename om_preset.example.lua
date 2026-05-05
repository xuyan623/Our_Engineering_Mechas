local preset = {
  board = {name = "rm-a-board"},
  os = {name = "freertos"},

  toolchain_default = {
    name = "gnu-rm",
  },

  toolchain_presets = {
    ["gnu-rm"] = {
      sdk = "D:/path/to/arm-gnu-toolchain",
      bin = "D:/path/to/arm-gnu-toolchain/bin",
    },
    ["armclang"] = {
      sdk = "D:/path/to/ARMCLANG",
      bin = "D:/path/to/ARMCLANG/bin",
    },
  },

  flash = {
    jlink = {
      device = "STM32F427II",
      interface = "swd",
      speed = 4000,
      program = "D:/path/to/JLink/JLink.exe",
      target = "robot_project",
      firmware = nil,
      prefer_hex = true,
      reset = true,
      run = true,
      native_output = false,
    },
  },
}

function get_preset()
  return preset
end
