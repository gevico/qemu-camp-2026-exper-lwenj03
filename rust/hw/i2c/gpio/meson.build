_libi2c_gpio_rs = static_library(
  'i2c_gpio',
  files('src/lib.rs', 'src/device.rs'),
  override_options: ['rust_std=2021', 'build.rust_std=2021'],
  rust_abi: 'rust',
  link_with: [
    _libi2c_rs,
    _util_rs,
    _bql_rs,
    _qom_rs,
    _system_rs,
    _hwcore_rs,
  ],
  dependencies: [
    common_rs,
    glib_sys_rs,
  ],
)

rust_devices_ss.add(when: 'CONFIG_X_I2C_RUST', if_true: [declare_dependency(
  link_whole: [_libi2c_gpio_rs],
  variables: {'crate': 'i2c_gpio'},
)])
