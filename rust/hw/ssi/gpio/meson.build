_librspi_rs = static_library(
  'rspi',
  files('src/lib.rs', 'src/device.rs'),
  override_options: ['rust_std=2021', 'build.rust_std=2021'],
  rust_abi: 'rust',
  link_with: [
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

rust_devices_ss.add(when: 'CONFIG_X_SPI_RUST', if_true: [declare_dependency(
  link_whole: [_librspi_rs],
  variables: {'crate': 'rspi'},
)])
