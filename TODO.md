# TODO

## Entity fixtures and tests

Add focused fixtures and CTest coverage under `tests/entities`. Prefer one source
file for each classname recognized by the compiler. Generic runtime entities
only need a dedicated contract when they exercise special compiler keys or
brush behavior.

### CSG

- [ ] `worldspawn`
- [ ] `func_group`
- [ ] `func_detail`
- [ ] `info_hullshape`
- [ ] `info_compile_parameters` rejection
- [ ] Generic brush entity using `zhlt_usemodel`
- [ ] Generic brush entity using an origin brush
- [ ] Generic brush entity using a brush that defines bounds

### BSP and VIS

- [ ] `info_player_start`
- [ ] `info_overview_point`
- [ ] `info_portal`
- [ ] `info_leaf`

### RAD lights

- [ ] `light`
- [ ] `light_spot`
- [ ] `light_environment`
- [ ] `info_sunlight`
- [ ] `light_surface`
- [ ] `light_shadow`
- [ ] `light_bounce`
- [ ] Custom `light*` entity carrying `_tex`

### RAD texture tables

- [ ] `info_texlights`
- [ ] `info_minlights`
- [ ] `info_chopscale`
- [ ] `info_smoothvalue`
- [ ] `info_translucent`
- [ ] `info_angularfade`

### Recognized but unavailable

- [ ] `env_static` reports the unsupported studio shadow behavior
- [ ] An entity using `zhlt_studioshadow` reports the unsupported behavior

### Existing indirect coverage to replace or promote

- [ ] Promote `worldspawn` coverage from general format and CSG tests
- [ ] Promote `info_hullshape` coverage from the CSG brush check
- [ ] Promote `func_door` and `func_train` brush behavior where appropriate
- [ ] Register the generated features fixture with CTest
- [ ] Split `func_door_rotating`, `func_wall`, `func_illusionary`, and
      `func_wall_toggle` behavior out of the generated features fixture when a
      special compiler contract exists
- [ ] Promote the generated fixture coverage for `func_group`, `func_detail`,
      `light`, `light_environment`, `info_overview_point`, `info_portal`,
      `info_leaf`, and `info_player_start`
