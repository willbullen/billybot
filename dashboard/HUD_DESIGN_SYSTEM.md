# HUD Design System - BillyBot Dashboard

Applied to the control dashboard and base layout to create a unified "command center" aesthetic across all pages.

---

## Color Palette

### Background Colors (dark-to-darker gradients)
| Token | Hex | Usage |
|---|---|---|
| `slate-950` | `#020617` | Body/deep background |
| `slate-900` | `#0f172a` | Sidebar, header gradient endpoints |
| `slate-800` | `#1e293b` | Header gradient midpoint |
| `slate-700/50` | Semi-transparent | Panel/card backgrounds (`.hud-panel`) |
| `slate-700` | `#334155` | Input backgrounds, scrollbar thumbs |

### Accent Colors (cyan family)
| Token | Hex | Usage |
|---|---|---|
| `cyan-500` | `#06b6d4` | Primary accent (joystick color, range input) |
| `cyan-400` | `#22d3ee` | Pulsing indicator dots, active toggle knobs |
| `cyan-300` | `#67e8f9` | Button text, hover text |
| `cyan-200` | `#a5f3fc` | Section headings, status labels |
| `cyan-100` | `#cffafe` | Active values, selected state text |
| `cyan-500/30` | 30% opacity | Border color for panels, buttons |
| `cyan-500/20` | 20% opacity | Hover backgrounds, subtle borders, selected state bg |

### Status/Severity Colors
| Color | Hex | Usage |
|---|---|---|
| `red-600` | `#dc2626` | E-STOP button background |
| `red-500/30` | 30% opacity | E-STOP border, danger glow |
| `green-500` | `#38a169` | Success actions |
| `orange-500` | `#dd6b20` | Warning/unsaved changes |
| `yellow-500` | `#d69e2e` | Caution/edit mode |

---

## Typography

- **Font family**: JetBrains Mono (monospace) throughout -- loaded via Google Fonts, with Fira Code and Cascadia Code as fallbacks
- **All text is UPPERCASE** -- labels, headings, button text, status indicators
- **Tracking**: `tracking-wider` on most text, `tracking-widest` on section headings
- **Title sizes**: `text-xl` for page title in header, `text-sm` for panel section titles, `text-xs` for labels and metadata
- **Colors**: Cyan shades for labels (`text-cyan-200`), white (`text-white`) for primary title, `text-slate-400/500` for muted info

---

## Layout Structure

### Sidebar
- Gradient background: `bg-gradient-to-b from-slate-900 via-slate-900 to-slate-950`
- Right border: `border-r border-cyan-500/20`
- Pulsing cyan dot as logo indicator (replaces previous gradient square)
- All nav items uppercase with `tracking-wider`
- Active state: `bg-cyan-500/20 text-cyan-100 border border-cyan-500/30`
- Hover state: `hover:bg-cyan-500/10 hover:text-cyan-200`
- Section dividers ("ROS 2", "SYSTEM") in `text-cyan-500/50`

### HUD Header Bar
- Full-width gradient: `bg-gradient-to-r from-slate-900 via-slate-800 to-slate-900`
- Bottom border: `border-b border-cyan-500/30`
- Pulsing cyan indicator dot before page title
- Status sub-bar below main header with `border-t border-cyan-500/20`
- Displays STATUS and MODE readouts in monospace

### Content Area
- `p-6` padding around page content
- Background inherits `bg-slate-950` from body

---

## Key UI Components

### 1. Pulsing Indicator Dot
```
w-3 h-3 bg-cyan-400 rounded-full animate-pulse
```
Used in header, sidebar logo, and connection status to indicate "live" state.

### 2. HUD Panel (`.hud-panel`)
```css
background: rgba(30, 41, 59, 0.5);
backdrop-filter: blur(12px);
border: 1px solid rgba(6, 182, 212, 0.2);
```
- Hover increases border opacity to 0.35
- Used for all card/panel containers on the control page
- Replaces the previous `.glass` class styling

### 3. Section Headers
Each panel section has a consistent header pattern:
```html
<div class="flex items-center gap-2 mb-4">
  <div class="w-2 h-2 bg-cyan-400 rounded-full"></div>
  <h2 class="text-sm font-mono font-semibold text-cyan-200 uppercase tracking-widest">SECTION NAME</h2>
</div>
```

### 4. Buttons

**Outline/default (cyan border):**
```
border border-cyan-500/30 text-cyan-300 hover:bg-cyan-500/20 font-mono uppercase tracking-wider
```

**Active/selected state:**
```
bg-cyan-500/20 text-cyan-100 border-cyan-500 ring-1 ring-cyan-500/50
```

**E-STOP (danger):**
```
bg-red-600 hover:bg-red-700 border border-red-500/30 font-mono font-bold uppercase tracking-widest
box-shadow: 0 0 30px rgba(239, 68, 68, 0.3)  /* red glow */
```

### 5. Toggle Switch
- Off: `bg-slate-700 border-slate-600` with `bg-slate-400` knob
- On: `bg-cyan-500/20 border-cyan-500/50` with `bg-cyan-400` knob

### 6. Range Input
```
bg-slate-700 rounded-lg accent-cyan-500
```

### 7. Connection Status Bar
```html
<div class="hud-panel rounded-lg p-4">
  <!-- Pulsing dot + BRIDGE ONLINE/OFFLINE label -->
  <!-- Divider: border-l border-cyan-500/20 -->
  <!-- LIN: value | ANG: value readouts -->
</div>
```

### 8. Joystick Zones
```
w-52 h-52 rounded-full bg-slate-800/60 border border-cyan-500/20
```
NippleJS color changed from `#3b82f6` (blue) to `#06b6d4` (cyan) to match the HUD accent.

---

## Changes Made

### `base.html`
1. **Font**: Replaced Inter (sans-serif) with JetBrains Mono (monospace) -- both in Tailwind config and Google Fonts import
2. **Body**: Changed from `font-sans` to `font-mono`, text color from `text-slate-200` to `text-slate-300`
3. **`.glass` class**: Updated border from `rgba(148, 163, 184, 0.1)` (slate) to `rgba(6, 182, 212, 0.2)` (cyan)
4. **Added `.hud-glow`**: New utility class for subtle cyan glow effects
5. **Sidebar**:
   - Background changed to gradient: `bg-gradient-to-b from-slate-900 via-slate-900 to-slate-950`
   - All borders changed from `border-slate-800` to `border-cyan-500/20`
   - Logo replaced: gradient square with "B" replaced by pulsing cyan dot
   - Title text: now `text-cyan-100 uppercase tracking-wider`
   - Nav items: all uppercase, using cyan accent colors
   - Section labels: `text-cyan-500/50 tracking-widest`
   - Collapse button: cyan-accented hover state
6. **Header**:
   - Background: changed to `bg-gradient-to-r from-slate-900 via-slate-800 to-slate-900`
   - Border: `border-cyan-500/30` (was `border-slate-800`)
   - Added pulsing cyan dot before page title
   - Title: `text-xl uppercase tracking-wider`
   - Connection status: cyan dot and `text-cyan-200` (was emerald/slate)
   - Added status sub-bar with STATUS/MODE readouts

### `control.html`
1. **Title**: Uppercased to "ROBOT CONTROL"
2. **Connection bar**: Wrapped in `.hud-panel`, changed from emerald to cyan indicator, added `BRIDGE ONLINE/OFFLINE` label, velocity readouts styled as `LIN:` / `ANG:` with cyan labels
3. **E-STOP button**: Added `border border-red-500/30`, wider tracking (`tracking-widest`), increased padding
4. **All panels**: Changed from `.glass rounded-xl` to `.hud-panel rounded-lg`
5. **Section headers**: Added cyan dot indicator before each heading, increased tracking to `tracking-widest`, text color to `text-cyan-200`
6. **All text**: Uppercased -- labels, descriptions, button text
7. **Camera placeholder**: Icon color changed to `text-cyan-500/30`, text to `text-cyan-200/60`
8. **Voice toggle**: Restyled with cyan border/background states instead of brand blue
9. **Home Position button**: Changed from filled `bg-slate-700` to outline `border border-cyan-500/30` style
10. **Preset/Bearing buttons**: Changed from brand-blue active states to cyan-accented borders with `ring-1 ring-cyan-500/50`, all text uppercased via `.toUpperCase()`
11. **Joystick borders**: Changed from `border-slate-700/50` to `border-cyan-500/20`
12. **NippleJS joystick colors**: Changed from `#3b82f6` (blue) and `#8b5cf6` (purple) to `#06b6d4` (cyan) for both
13. **Range slider**: Changed accent from `accent-brand-500` to `accent-cyan-500`

### `nav_tags.py`
- Active state: `bg-cyan-500/20 text-cyan-100 border border-cyan-500/30` (was `bg-slate-700/60 text-white`)
- Hover state: `hover:bg-cyan-500/10 hover:text-cyan-200` (was `hover:bg-slate-700/40 hover:text-white`)

---

## Design Principles

- **Dark theme only** -- deep slates and near-blacks, no light mode
- **Cyan as signature accent** -- borders, text highlights, indicators, hover states (all at reduced opacity)
- **Monospace everywhere** -- JetBrains Mono gives a technical/instrument readout feel
- **ALL CAPS text** -- labels, statuses, button text, headings, all uppercase
- **Semi-transparent layers** -- backgrounds use `/50`, `/30`, `/20` opacity variants for depth
- **Backdrop blur** -- floating panels use `backdrop-filter: blur(12px)` for glass-morphism
- **Gradient backgrounds** -- linear gradients on header (`to-r`) and sidebar (`to-b`)
- **Animated indicators** -- pulsing dots for live/active states
- **Minimal border radius** -- `rounded-lg` for panels, `rounded-full` only for indicator dots and joystick zones
- **Consistent spacing** -- `gap-4` throughout grids, `p-4`/`p-5`/`p-6` for panel padding
