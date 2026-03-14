# midway-univiewer

A 3D viewer for Midway arcade game universe files (`.UNI`).
Written in C99 with SDL2. Runs on Linux and Windows.

## What it does

`.UNI` files define a 3D scene used by several Midway TMS34010-based arcade games
(Mortal Kombat, NBA Jam, etc.).  Each file references one or more `.IMG` sprite
containers and places named sprites at fixed world-space coordinates.  The viewer
reads these files, projects the sprites with perspective, and lets you explore the
scene interactively.

## Building

### Linux

```bash
sudo apt install libsdl2-dev cmake build-essential
cmake -B build && cmake --build build
./build/univiewer path/to/SCENE.UNI
```

### Windows (Visual Studio 2022)

```powershell
powershell -ExecutionPolicy Bypass -File build.ps1
```

SDL2 is downloaded automatically and shared with other midway-* tools under
`%LOCALAPPDATA%\midway-build\deps`.  The resulting `univiewer.exe` and `SDL2.dll`
are written to `%LOCALAPPDATA%\univiewer-build\build\Release\`.

## Usage

```
univiewer <file.UNI>
```

The `.UNI` file and all referenced `.IMG` files must be in the same directory.

## Controls

### 3D view

| Key | Action |
|-----|--------|
| Arrow keys | Pan camera left/right, move forward/back |
| Shift + arrows | Same, 2× speed |
| Ctrl + Up/Down | Raise / lower eye height |
| Mouse drag (LMB) | Pan camera |
| Scroll wheel / `+` / `-` | Zoom (adjust focal length) |
| `Home` | Reset camera to default position |
| `TAB` | Switch to 2D overhead map |
| `Escape` | Quit |
| Shift+T | Toggle perspective grid |
| Shift+B | Toggle sprite border outlines |
| Shift+O | Toggle sprite rendering |
| Shift+S | Toggle sky/ground background |

### 2D map (TAB)

Shows a top-down overhead view of all object positions.
Near objects are at the bottom; far objects at the top.
The yellow crosshair marks the current camera position.
Arrow keys, Shift+arrows, and mouse drag still move the camera;
the crosshair updates in real time.

Hover the mouse over a dot to see the object's name, coordinates,
sprite size, and flags in a tooltip.

## File formats

### UNI binary layout

| Offset | Size | Field |
|--------|------|-------|
| 0 | 2 | Constant (270) |
| 2 | 2 | `num_imgs` — number of IMG file references |
| 4 | 2 | `num_objs` — number of object records |
| 6 | 2 | `halfy` — screen Y of the horizon (signed) |
| 10 | 2 | Sky colour (BGR555) |
| 12 | 4 | `zmin` — signed 16.16 fixed-point |
| 16 | 4 | `world_y` — floor/eye Y, signed 16.16 fixed-point |
| 32 | 2 | Ground colour (BGR555) |
| 48 | `num_imgs × 64` | IMG path table (Windows absolute path, null-terminated, zero-padded to 64 bytes) |
| 48 + `num_imgs×64` | `num_objs × 48` | Object records |

Each object record:

| Offset | Size | Field |
|--------|------|-------|
| 0 | 2 | IMG file index (0-based) |
| 2 | 16 | Image name (null-terminated) |
| 20 | 4 | X — signed 16.16 fixed-point |
| 24 | 4 | Y — signed 16.16 fixed-point |
| 28 | 4 | Z — signed 16.16 fixed-point (depth; larger = farther) |
| 32 | 2 | Flags: bit 4 = horizontal flip, bit 5 = vertical flip |

### IMG container layout

See [midway-imgtool](https://github.com/Asure/midway-imgtool) for the full
`wmpstruc.inc` struct definitions.  Key points:

- `LIB_HDR`: `IMGCNT[2]` + `PALCNT[2]` + `OSET[4]` + …
- `IMAGE` struct = 50 bytes; pixel rows padded to a multiple of 4 bytes
- `PALETTE` struct = 26 bytes; colours are BGR555 (`XRRRRRGGGGGBBBBB`)
- `NUMDEFPAL = 3`: only `PALCNT − 3` palettes are stored in the file;
  `IMAGE.PALNUM − 3` is the index into the stored palette array

## Perspective projection

Matches the original TMS34010 engine (`gxd.asm`):

```
screen_x = (world_x - cam_x) / dz * focal + cx
screen_y = (world_y - cam_y) / dz * focal + halfy
sprite_w = native_w * focal / dz
sprite_h = native_h * focal / dz
```

where `dz = object_z − cam_z` and `focal ≈ min_Z` of the scene (computed
automatically at load time).

## Related tools

- [midway-bddtool](https://github.com/Asure/midway-bddtool) — BDD/BDB background viewer/editor
- [midway-imgtool](https://github.com/Asure/midway-imgtool) — IMG container editor (Dosbox)
