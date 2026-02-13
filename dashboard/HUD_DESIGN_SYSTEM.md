# HUD Design System - BillyBot Dashboard

Applied to the control dashboard and base layout to create a refined "command center" aesthetic. The design balances readability with a technical HUD feel -- using title case for natural reading flow while reserving uppercase for status readouts and section labels.

---

## Color Palette

### Background Colors (dark-to-darker gradients)
| Token | Hex | Usage |
|---|---|---|
| `slate-950` | `#020617` | Body/deep background |
| `slate-900` | `#0f172a` | Sidebar, header gradient endpoints |
| `slate-800` | `#1e293b` | Header gradient midpoint, panel backgrounds |
| `slate-700/50` | Semi-transparent | Panel/card backgrounds (`.hud-panel`) |
| `slate-700` | `#334155` | Input backgrounds, scrollbar thumbs |

### Accent Colors (cyan family)
| Token | Hex | Usage |
|---|---|---|
| `cyan-500` | `#06b6d4` | Primary accent (joystick color, range input) |
| `cyan-400` | `#22d3ee` | Pulsing indicator dots, active toggle knobs, active nav border |
| `cyan-300` | `#67e8f9` | Outline button text, velocity readout values |
| `cyan-200` | `#a5f3fc` | Panel section headings |
| `cyan-100` | `#cffafe` | Active/selected button text, status bar values |
| `cyan-500/30` | 30% opacity | Header bottom border, outline button borders |
| `cyan-500/20` | 20% opacity | Subtle borders (panels, joystick zones, dividers), hover backgrounds |
| `cyan-500/10` | 10% opacity | Active nav background tint |
| `cyan-400/70` | 70% opacity | Sidebar subtitle text |

### Status Colors
| Color | Hex | Usage |
|---|---|---|
| `emerald-500` | `#10b981` | Connection status indicator (header + control bar) |
| `red-600` | `#dc2626` | E-STOP button background |
| `red-500` | `#ef4444` | Disconnected indicator |

---

## Typography

- **Font family**: JetBrains Mono (monospace) -- loaded via Google Fonts, with Fira Code and Cascadia Code as fallbacks
- **Text casing**: Title case for navigation, headings, buttons, and labels. Uppercase reserved for status readouts (STATUS: OPERATIONAL, MODE: MANUAL) and sidebar section dividers (ROS 2, System)
- **Tracking**: `tracking-wider` on panel section headings, `tracking-widest` on sidebar section dividers
- **Title sizes**: `text-lg` for page title in header, `text-sm` for panel section titles, `text-xs` for labels and metadata
- **Colors**: `text-cyan-200` for section headings, `text-white` for page title and sidebar logo, `text-slate-400/500` for muted/secondary info, `text-cyan-300` for inline data values

---

## Layout Structure

### Sidebar
- Gradient background: `bg-gradient-to-b from-slate-900 via-slate-900 to-slate-950`
- Right border: `border-r border-cyan-500/20`
- Logo: Cyan gradient rounded square (`bg-gradient-to-br from-cyan-500 to-cyan-700`) with monitor icon, "BillyBot" title + "Robot Dashboard" subtitle
- Nav items in title case with `text-sm font-medium`
- Active state: `bg-cyan-500/10 text-white border-l-2 border-cyan-400` (left cyan accent border)
- Hover state: `hover:bg-slate-700/40 hover:text-white`
- Section dividers ("ROS 2", "System") in `text-cyan-500/50 uppercase tracking-widest`

### HUD Header Bar
- Full-width gradient: `bg-gradient-to-r from-slate-900 via-slate-800 to-slate-900`
- Bottom border: `border-b border-cyan-500/30`
- Pulsing cyan indicator dot (`w-3 h-3 bg-cyan-400 animate-pulse`) before page title
- Connection status on right: emerald dot + "Connected" text
- Status sub-bar below with `border-t border-cyan-500/20`: shows STATUS and MODE in uppercase monospace

### Content Area
- `p-6` padding around page content
- Background inherits `bg-slate-950` from body

---

## Key UI Components

### 1. Pulsing Indicator Dot
```
w-3 h-3 bg-cyan-400 rounded-full animate-pulse
```
Used in header before page title. Smaller variant (`w-2 h-2`) with `pulse-dot` animation for connection status.

### 2. HUD Panel (`.hud-panel`)
```css
background: rgba(30, 41, 59, 0.5);
backdrop-filter: blur(12px);
border: 1px solid rgba(6, 182, 212, 0.2);
```
- Hover increases border opacity to 0.35
- Used with `rounded-xl` for card containers on the control page
- Provides glass-morphism depth effect

### 3. Section Headings
Panel section titles use a consistent pattern:
```html
<h2 class="text-sm font-semibold text-cyan-200 uppercase tracking-wider mb-2">Drive</h2>
<p class="text-xs text-slate-500 mb-4">Forward / Backward / Turn</p>
```
Heading text is in `text-cyan-200` with `uppercase tracking-wider`. Subtitles are in natural case `text-slate-500`.

### 4. Buttons

**Outline/default (cyan border):**
```
border border-cyan-500/30 text-cyan-300 hover:bg-cyan-500/20 text-sm
```

**Preset buttons (default state):**
```
bg-slate-700/50 text-slate-300 border-cyan-500/20 hover:bg-slate-600/50 hover:border-cyan-500/30
```

**Preset buttons (active/selected state):**
```
bg-cyan-500/20 text-cyan-100 border-cyan-500 ring-2 ring-cyan-400/30
```

**E-STOP (danger):**
```
bg-red-600 hover:bg-red-700 text-white font-bold uppercase tracking-wider rounded-xl
box-shadow: 0 0 30px rgba(239, 68, 68, 0.3)  /* red glow */
```

### 5. Toggle Switch
- Off: `bg-slate-600 border-slate-500` with `bg-slate-400` knob
- On: `bg-cyan-500/20 border-cyan-500/50` with `bg-cyan-400` knob

### 6. Range Input
```
bg-slate-700 rounded-lg accent-cyan-500
```

### 7. Connection Status Bar
Simple inline layout (not wrapped in a panel):
```html
<div class="flex items-center gap-4">
  <!-- Emerald pulsing dot + "Bridge connected" text -->
  <!-- Lin: value | Ang: value in text-cyan-300 -->
</div>
```

### 8. Joystick Zones
```
w-52 h-52 rounded-full bg-slate-800/60 border border-cyan-500/20
```
NippleJS color set to `#06b6d4` (cyan-500) for both drive and pan/tilt joysticks.

---

## Changes Made

### Revision 2 (current) -- Refined to match design mockup

#### `base.html`
1. **Font**: JetBrains Mono (monospace) via Google Fonts -- Fira Code and Cascadia Code as fallbacks
2. **Body**: `font-mono` base, `text-slate-300` for default text
3. **Sidebar logo**: Cyan gradient rounded square (`from-cyan-500 to-cyan-700`) with monitor SVG icon, "BillyBot" heading + "Robot Dashboard" subtitle in `text-cyan-400/70`
4. **Nav items**: Title case ("Dashboard", "Control", "Telemetry") -- not all caps
5. **Active nav**: Left cyan border accent `border-l-2 border-cyan-400` with subtle `bg-cyan-500/10` tint
6. **Hover nav**: `hover:bg-slate-700/40 hover:text-white` -- warmer, more natural
7. **Section dividers**: "ROS 2" and "System" in `text-cyan-500/50 uppercase tracking-widest`
8. **Collapse button**: Title case "Collapse", `text-slate-400` default
9. **Header**: `text-lg` page title in title case with pulsing cyan dot
10. **Connection status**: Emerald green dot (`bg-emerald-500`) + "Connected" text in `text-slate-400`
11. **Status sub-bar**: STATUS/MODE readouts in uppercase (these are instrument readouts)
12. **Borders**: All dividers use `border-cyan-500/20`, header bottom uses `border-cyan-500/30`

#### `control.html`
1. **Title**: Title case "Robot Control" (not "ROBOT CONTROL")
2. **Connection bar**: Simple flex layout (no panel wrapper), emerald status dot, "Bridge connected/disconnected" in natural case
3. **Velocity readouts**: `Lin:` / `Ang:` labels in `text-slate-500`, values in `text-cyan-300`
4. **E-STOP**: Red glow button with `uppercase tracking-wider`, `rounded-xl`
5. **Panels**: `.hud-panel rounded-xl` with `p-6` padding
6. **Section headings**: `text-cyan-200 uppercase tracking-wider` -- compact uppercase for the label only
7. **Subtitles/descriptions**: Natural case ("Forward / Backward / Turn", "Pan Left/Right / Tilt Up/Down")
8. **Camera placeholder**: Original wording -- "No camera connected", "Requires camera driver (Stage 4)"
9. **Button labels**: Title case ("Bumper", "Ten-Hut", "Look Up") using `x-text="preset.label"` directly -- no `.toUpperCase()`
10. **Preset buttons (default)**: `bg-slate-700/50 text-slate-300 border-cyan-500/20` with softer hover
11. **Preset buttons (active)**: `bg-cyan-500/20 text-cyan-100 border-cyan-500 ring-2 ring-cyan-400/30`
12. **Voice toggle**: Cyan-accented on state (`bg-cyan-500/20 border-cyan-500/50`), neutral off state (`bg-slate-600 border-slate-500`)
13. **Home Position button**: Outline style `border border-cyan-500/30 text-cyan-300 hover:bg-cyan-500/20`
14. **Joystick zones**: `border-cyan-500/20` borders, NippleJS color `#06b6d4` (cyan)
15. **Range slider**: `accent-cyan-500` with `bg-slate-700` track

#### `nav_tags.py`
- Active state: `bg-cyan-500/10 text-white border-l-2 border-cyan-400` (left accent border)
- Hover state: `hover:bg-slate-700/40 hover:text-white`

### Revision 3 -- Full dashboard HUD conversion

Applied the HUD design system to all 7 remaining dashboard pages. Every page now uses the unified component classes defined in `base.html`.

#### New CSS utility classes added to `base.html`
- `.hud-panel` -- Glass-morphism panel with cyan border (replaces `.glass`)
- `.hud-btn` -- Base button styling
- `.hud-btn-primary` -- Cyan accent button (border + text)
- `.hud-btn-secondary` -- Slate button (neutral)
- `.hud-btn-danger` -- Red accent button
- `.hud-btn-warning` -- Amber accent button
- `.hud-input` -- Input/select styling with cyan focus ring
- `.hud-section-title` -- Consistent section headings (cyan, uppercase, tracking-widest)
- `.hud-data-row` -- Data display rows with slate-800 background

#### Pages updated
| Page | Key changes |
|---|---|
| `dashboard.html` | `.glass` → `.hud-panel`, `text-brand-*` → `text-cyan-*`, quick actions use `hud-btn-*` classes, status cards get `hud-glow` |
| `telemetry.html` | `.glass` → `.hud-panel`, chart colors updated (FL line now cyan `#06b6d4`), `text-brand-*` → `text-cyan-*`, VAD/Robot state badges use cyan accents |
| `topics.html` | `.glass` → `.hud-panel`, inputs use `hud-input`, selected topic highlight cyan, echo button `hud-btn-primary` |
| `nodes.html` | `.glass` → `.hud-panel`, selected node highlight cyan, code output uses `border-cyan-500/10` |
| `chat.html` | `.glass` → `.hud-panel`, user messages use `bg-cyan-600/20`, thinking dots cyan, quick commands use cyan border hover |
| `logs.html` | `.glass` → `.hud-panel`, form controls use `hud-input`, checkbox uses `text-cyan-500`, INFO lines highlighted in `text-cyan-300/70` |
| `settings.html` | `.glass` → `.hud-panel`, env var keys in `text-cyan-400/60`, system info values in `text-cyan-300`, restart buttons use `hud-btn-warning` |

### Revision 1 (initial)
1. Replaced Inter (sans-serif) with JetBrains Mono (monospace)
2. Changed body from `font-sans` to `font-mono`
3. Updated `.glass` border to cyan tint `rgba(6, 182, 212, 0.2)`
4. Added `.hud-glow` utility class
5. Changed all sidebar/header borders from `border-slate-800` to `border-cyan-500/20`
6. Added header gradient `bg-gradient-to-r from-slate-900 via-slate-800 to-slate-900`
7. Added status sub-bar with STATUS/MODE readouts
8. Changed joystick NippleJS colors from blue/purple to cyan `#06b6d4`
9. Changed slider accent from `accent-brand-500` to `accent-cyan-500`

---

## Design Principles

- **Dark theme only** -- deep slates and near-blacks everywhere, no light mode
- **Cyan as signature accent** -- borders, text highlights, indicators, hover states (all at reduced opacity for subtlety)
- **Monospace everywhere** -- JetBrains Mono gives a technical/instrument readout feel
- **Title case for readability** -- nav items, button labels, headings use natural casing; uppercase reserved for status readouts and section dividers
- **Semi-transparent layers** -- backgrounds use `/50`, `/30`, `/20`, `/10` opacity variants for depth
- **Backdrop blur** -- panels use `backdrop-filter: blur(12px)` for glass-morphism
- **Gradient backgrounds** -- linear gradients on header (`to-r`) and sidebar (`to-b`)
- **Animated indicators** -- pulsing dots for live/active states
- **Left border accent** -- active nav uses `border-l-2 border-cyan-400` for clear visual indicator
- **Consistent border radius** -- `rounded-xl` for panels, `rounded-lg` for buttons, `rounded-full` for indicator dots and joystick zones
- **Consistent spacing** -- `gap-6` for main grids, `gap-3` for button groups, `p-5`/`p-6` for panel padding
