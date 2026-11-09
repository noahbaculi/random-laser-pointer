# random-laser-pointer

Domain terms for the laser pointer station: two servos aim a laser around a room outline, with manual control from a phone page.

## Rooms and outlines

**Room**:
One of four named slots, each holding an outline. One room is active at a time.
_Avoid_: Preset, slot

**Outline**:
A closed, non-self-crossing polygon of up to 16 points that bounds Random.
_Avoid_: Shape, boundary, rectangle

**Point**:
A servo position in duty counts on both axes, and a vertex of an outline.
_Avoid_: Corner, vertex (in UI copy)

**Area**:
An outline with at least 3 points, the minimum Random and Preview need.

**Travel**:
The servo duty range, the limit for every position and point.
_Avoid_: Range, limits

## Modes and motion

**Mode**:
What the phone asks the station to do: Random, Manual, or Off. It is never saved, and the station boots into Random.

**Random**:
The mode that walks the laser between random targets inside the active outline.
_Avoid_: Auto, play

**Manual**:
The mode where the phone moves the laser with the D-pad or a goto.

**Off**:
The mode with the laser off and the servos held.

**Activity**:
What the station is doing: Random, Previewing, Manual, or Off. It differs from the mode only during a preview, which the page shows under Random.

**Glide**:
The per-count delay range of Random's walk, which sets how fast the laser moves between targets.
_Avoid_: Speed (that is the D-pad and goto rate)

**Phase**:
Random's duty cycle, alternating Active (laser on) and Rest (laser off).
_Avoid_: Cycle, sleep

**Idle**:
Random in an active phase without an area, so the laser stays off and the servos hold. It is derived from the outline and never reported.

**Walk**:
Random's motion, either moving straight to a target or dwelling at one.

**Target**:
The point the laser is currently heading for in Random, a goto, or a preview.

**Dwell**:
A random pause at a target before the next one.
_Avoid_: Wait, hold

**Preview**:
A single trace of the active outline from the current position, ending in Random at the first point.
_Avoid_: Trace mode, demo

**Leg**:
One segment of a preview, toward the next point of the outline.

**Goto**:
A straight glide to a chosen point in Manual, used when opening a point to edit.
_Avoid_: Jump, seek

## Control

**D-pad**:
The page's 8-way direction pad for Manual. Held buttons resend as keepalives.

**Fine**:
Quarter-speed D-pad, offered only in the point editor.

**Keepalive**:
A repeated D-pad command that keeps a held direction moving.

**Deadman**:
The stop that ends D-pad motion when keepalives stop arriving.

## Storage and page

**Settings record**:
The versioned, checksummed copy of all rooms and tuning values kept in flash.

**Would-be outline**:
The page's draft of the outline with the laser's current position added or replacing a point.

**Offline**:
The page's state after repeated failed polls, when controls go inert.
